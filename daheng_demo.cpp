// ============================================================================
// daheng_demo.cpp — 大恒 USB3 工业相机拉流 + 装甲板识别 demo
//                    （Galaxy SDK 2.6 适配版）
//
// 流程: 初始化库 -> 枚举设备 -> 按序列号打开 -> 配置 -> 取流 -> Bayer转BGR
//       -> 装甲板识别(灯条BGR颜色分割+几何配对) -> 绘制外框与角点 -> 实时FPS
// 所有调试信息统一使用 rm_log（基于 spdlog），不再出现 std::cout / printf。
//
// 用法:
//   ./daheng_demo                    按默认序列号 NE0190010001 打开相机
//   ./daheng_demo <相机序列号>        按指定序列号打开相机
//
// 键盘控制:
//   c      切换识别颜色: red -> blue -> both（默认 red）
//   b      显示/隐藏颜色分割二值图调试窗口
//   p      保存当前帧与颜色掩码到 PNG（离线调参用）
//   f      灯条过曝一键修复: 按过曝占比自动压低曝光（可连按）
//   x      颜色不对时轮换 Bayer 转换码（BG/GR/GB/RG）
//   e / d  增大 / 减小曝光时间（步长 25 us）
//   a / q  增大 / 减小增益（步长 0.1 dB）
//   s / w  增大 / 减小伽马（步长 0.1，相机不支持该节点时会提示失败）
//   ESC    退出（退出前把当前曝光写入 exposure_us.txt, 下次启动自动恢复,
//          这样相机断电/重开后不需要重新调曝光）
//
// 新版 Galaxy SDK 2.6 与官方旧 demo 的差异（务必适配，否则编译/运行出错）:
//   1. 没有 GXOpenDeviceByIndex / GXOpenDeviceBySN，只能调用
//      GXOpenDevice(GX_OPEN_PARAM*, GX_DEV_HANDLE*)；
//      GX_OPEN_PARAM 的输入参数成员名是 pszContent（char*）；
//      GX_OPEN_INDEX 方式的序号从 1 开始（不是 0）
//   2. 枚举设备用 GXUpdateAllDeviceList（旧名 GXUpdateDeviceList 已废弃）
//   3. GX_FRAME_DATA 没有 nSize 字段；且 GXGetImage 要求调用者预分配 pImgBuf，
//      传 NULL 会返回 -5（GX_STATUS_INVALID_PARAMETER）
//   4. 不使用 DxImageProc，Bayer 转 BGR 用 OpenCV 的 cv::cvtColor
// ============================================================================

#include "GxIAPI.h"
#include "armor_detector.h"
#include "rm_log.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

// 默认相机序列号（无命令行参数时使用，可用 argv[1] 覆盖）
constexpr char kDefaultSerial[] = "NE0190010001";

// ----------------------------------------------------------------------------
// 错误码转人类可读描述。GXGetLastError 需要调用两次:
//   第一次传 NULL 拿长度, 第二次拿内容（见 SDK 文档）
// ----------------------------------------------------------------------------
std::string GetErrorString(GX_STATUS emErrorStatus) {
    size_t size = 0;
    if (GXGetLastError(&emErrorStatus, nullptr, &size) != GX_STATUS_SUCCESS) {
        return "<GXGetLastError 调用失败>";
    }
    std::vector<char> buffer(size);
    GX_STATUS emStatus = GXGetLastError(&emErrorStatus, buffer.data(), &size);
    return emStatus == GX_STATUS_SUCCESS ? std::string(buffer.data())
                                         : "<GXGetLastError 调用失败>";
}

struct DeviceInfo {
    std::string serial;
    std::string model;
};

// ----------------------------------------------------------------------------
// 枚举设备并打印序列号。
// 注意: 大恒设备序号从 1 开始（和海康的 0 开始不一样）!
// ----------------------------------------------------------------------------
std::vector<DeviceInfo> EnumerateDevices() {
    uint32_t device_num = 0;
    // 第二个参数是枚举超时(ms): 官方建议至少 1000, USB3 相机一般 1s 内可枚举完
    GX_STATUS emStatus = GXUpdateAllDeviceList(&device_num, 1000);
    if (emStatus != GX_STATUS_SUCCESS || device_num == 0) {
        RM_LOG_ERROR("枚举设备失败({}): {}", static_cast<int>(emStatus), GetErrorString(emStatus));
        RM_LOG_ERROR("请检查: 1) 相机是否上电 2) 是否插在 USB3.0 口 3) 设备权限");
        return {};
    }

    std::vector<DeviceInfo> devices;
    RM_LOG_INFO("共枚举到 {} 台大恒设备:", device_num);
    for (uint32_t i = 1; i <= device_num; ++i) {  // 序号从 1 开始!
        GX_DEVICE_INFO info;
        std::memset(&info, 0, sizeof(info));
        if (GXGetDeviceInfo(i, &info) != GX_STATUS_SUCCESS) {
            RM_LOG_WARN("读取第 {} 台设备信息失败", i);
            continue;
        }
        // 本项目只支持 USB3 Vision 相机（GigE 的序列号在 stGigEDevInfo 里）
        if (info.emDevType == GX_DEVICE_CLASS_U3V) {
            auto &u3v = info.DevInfo.stU3VDevInfo;
            // SDK 字段是 unsigned char[64], 打印前要显式转成 std::string
            std::string serial(reinterpret_cast<const char *>(u3v.chSerialNumber));
            std::string model(reinterpret_cast<const char *>(u3v.chModelName));
            RM_LOG_INFO("  [{}] 型号: {}  序列号: {}", i, model, serial);
            devices.push_back({serial, model});
        }
    }
    return devices;
}

// ----------------------------------------------------------------------------
// 调参三件套: 同一套"读-改-限幅-写"模式（与官方示例一致）
// ----------------------------------------------------------------------------
bool AddExposureTime(GX_DEV_HANDLE device, double delta_us) {
    GX_FLOAT_VALUE node;
    std::memset(&node, 0, sizeof(node));
    GX_STATUS emStatus = GXGetFloatValue(device, "ExposureTime", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("读取曝光时间失败: {}", GetErrorString(emStatus));
        return false;
    }
    double value = node.dCurValue + delta_us;
    value = value < 1.0 ? 1.0 : (value > 10000.0 ? 10000.0 : value);
    emStatus = GXSetFloatValue(device, "ExposureTime", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("设置曝光时间失败: {}", GetErrorString(emStatus));
        return false;
    }
    RM_LOG_INFO("曝光时间 -> {} us", value);
    return true;
}

bool AddGain(GX_DEV_HANDLE device, double delta_db) {
    GX_FLOAT_VALUE node;
    std::memset(&node, 0, sizeof(node));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gain", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("读取增益失败: {}", GetErrorString(emStatus));
        return false;
    }
    double value = node.dCurValue + delta_db;
    value = value < 0.0 ? 0.0 : (value > 32.0 ? 32.0 : value);
    emStatus = GXSetFloatValue(device, "Gain", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("设置增益失败: {}", GetErrorString(emStatus));
        return false;
    }
    RM_LOG_INFO("增益 -> {} dB", value);
    return true;
}

bool AddGamma(GX_DEV_HANDLE device, double delta) {
    GX_FLOAT_VALUE node;
    std::memset(&node, 0, sizeof(node));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gamma", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("读取伽马失败(可能该型号无此节点): {}", GetErrorString(emStatus));
        return false;
    }
    double value = node.dCurValue + delta;
    value = value < 0.1 ? 0.1 : (value > 3.0 ? 3.0 : value);
    emStatus = GXSetFloatValue(device, "Gamma", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("设置伽马失败: {}", GetErrorString(emStatus));
        return false;
    }
    RM_LOG_INFO("伽马 -> {}", value);
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    // 确保日志目录存在（spdlog 不会自动创建目录, 目录缺失时 INIT_LOG 会抛异常）
    std::filesystem::create_directories("logs");
    // 初始化 rm_log: (日志文件路径, 控制台级别, 文件级别, 全局级别)
    // 级别可选: trace / debug / info / warn / error / critical
    INIT_LOG("logs/daheng_camera.log", "debug", "info", "debug");
    RM_LOG_INFO("======== 大恒相机拉流 + 装甲板识别 demo 启动 ========");

    // 1. 初始化 Galaxy SDK（必须最先调用, 与最后的 GXCloseLib 配对）
    GX_STATUS emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("GXInitLib 失败: {}", GetErrorString(emStatus));
        utils::RMLOG::instance().Close();
        return -1;
    }
    RM_LOG_INFO("Galaxy SDK 初始化成功, SDK 版本: {}", GXGetLibVersion());

    // 2. 枚举设备, 打印序列号
    std::vector<DeviceInfo> devices = EnumerateDevices();
    if (devices.empty()) {
        GXCloseLib();
        utils::RMLOG::instance().Close();
        return -1;
    }

    // 3. 按序列号打开。
    //    新版 SDK 只能走 GXOpenDevice + GX_OPEN_PARAM（没有 ByIndex / BySN）
    std::string serial = (argc >= 2) ? argv[1] : kDefaultSerial;
    bool serial_found = false;
    for (const auto &d : devices) {
        if (d.serial == serial) {
            serial_found = true;
            break;
        }
    }
    if (!serial_found) {
        if (argc < 2 && !devices.empty()) {
            // 默认序列号(培训资料里的相机)与实际插的相机不符, 没显式传参时
            // 自动改用枚举到的第一台, 保证换相机也能直接跑起来
            RM_LOG_WARN("默认序列号 {} 不在枚举列表中, 改用枚举到的第一台: {}",
                        kDefaultSerial, devices.front().serial);
            serial = devices.front().serial;
        } else {
            RM_LOG_ERROR("序列号 {} 不在枚举列表里, 请核对后重试", serial);
            GXCloseLib();
            utils::RMLOG::instance().Close();
            return -1;
        }
    }

    GX_DEV_HANDLE device = nullptr;
    GX_OPEN_PARAM open_param;
    std::memset(&open_param, 0, sizeof(open_param));
    open_param.openMode = GX_OPEN_SN;                             // 按序列号打开
    open_param.pszContent = const_cast<char *>(serial.c_str());   // 新版成员名是 pszContent!
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;                  // 独占模式（可读可写）
    emStatus = GXOpenDevice(&open_param, &device);
    if (emStatus != GX_STATUS_SUCCESS || device == nullptr) {
        RM_LOG_ERROR("打开相机 {} 失败: {}", serial, GetErrorString(emStatus));
        RM_LOG_WARN("若报 -5(INVALID_PARAMETER): 多为 pszContent 为空或格式不对; "
                    "若报 -14(TIMEOUT): USB3 Vision 传输层未生效, 重启系统或重插相机后再试");
        GXCloseLib();
        utils::RMLOG::instance().Close();
        return -1;
    }
    RM_LOG_INFO("打开相机 {} 成功", serial);

    // 4. 配置相机
    // 4.1 自动挡全关（手动调参前必须关自动, 否则参数会被相机自动覆盖）
    GXSetEnumValueByString(device, "ExposureAuto", "Off");
    GXSetEnumValueByString(device, "GainAuto", "Off");
    GXSetEnumValueByString(device, "BalanceWhiteAuto", "Continuous");  // 白平衡保持自动, 颜色才正常

    // 4.2 连续采集 + 关触发
    emStatus = GXSetEnumValueByString(device, "AcquisitionMode", "Continuous");
    if (emStatus != GX_STATUS_SUCCESS) RM_LOG_WARN("设置连续采集失败: {}", GetErrorString(emStatus));
    emStatus = GXSetEnumValueByString(device, "TriggerMode", "Off");
    if (emStatus != GX_STATUS_SUCCESS) RM_LOG_WARN("关闭触发模式失败: {}", GetErrorString(emStatus));

    // 4.3 输出传感器原始 Bayer RG8（之后用 OpenCV 转 BGR）
    emStatus = GXSetEnumValue(device, "PixelFormat", GX_PIXEL_FORMAT_BAYER_RG8);
    if (emStatus != GX_STATUS_SUCCESS) RM_LOG_WARN("设置 PixelFormat 失败: {}", GetErrorString(emStatus));

    // 4.4 大恒 USB3 相机出厂默认限速! 拉到上限, 否则高分辨率下帧率上不去
    GX_INT_VALUE limit_node;
    std::memset(&limit_node, 0, sizeof(limit_node));
    if (GXGetIntValue(device, "DeviceLinkThroughputLimit", &limit_node) == GX_STATUS_SUCCESS) {
        emStatus = GXSetIntValue(device, "DeviceLinkThroughputLimit", limit_node.nMax);
        if (emStatus != GX_STATUS_SUCCESS) {
            RM_LOG_WARN("放开传输限速失败: {}", GetErrorString(emStatus));
        } else {
            RM_LOG_INFO("传输限速已放开到上限 {}", limit_node.nMax);
        }
    }

    // 4.5 决定 cvtColor 用哪个 Bayer 转换码（防颜色错乱）。
    // 注意: 本机 MER2-230-168U3C-L 实测 SDK 上报的 PixelColorFilter(=RG) 与传感器
    // 实际 CFA 排列成对角互换——按上报的 RG 直接转会红蓝颠倒。因此查表时取"对角码"。
    // 换相机若颜色仍异常, 运行中按 x 键轮换四个转换码即可现场找到正确的一个。
    const std::vector<int> cvt_codes = {cv::COLOR_BayerBG2BGR, cv::COLOR_BayerGR2BGR,
                                        cv::COLOR_BayerGB2BGR, cv::COLOR_BayerRG2BGR};
    const std::vector<const char *> cvt_names = {"BG", "GR", "GB", "RG"};
    size_t cvt_index = 0;  // 相机报 RG -> 实际用 BG(本机实测)
    int cv_conversion_code = cvt_codes[cvt_index];
    GX_ENUM_VALUE filter_node;
    std::memset(&filter_node, 0, sizeof(filter_node));
    if (GXGetEnumValue(device, "PixelColorFilter", &filter_node) == GX_STATUS_SUCCESS) {
        switch (filter_node.stCurValue.nCurValue) {
            case GX_COLOR_FILTER_BAYER_GB: cvt_index = 1; break;  // 报 GB -> 用 GR
            case GX_COLOR_FILTER_BAYER_GR: cvt_index = 2; break;  // 报 GR -> 用 GB
            case GX_COLOR_FILTER_BAYER_BG: cvt_index = 3; break;  // 报 BG -> 用 RG
            case GX_COLOR_FILTER_BAYER_RG:
            default: cvt_index = 0; break;                        // 报 RG -> 用 BG
        }
        cv_conversion_code = cvt_codes[cvt_index];
        RM_LOG_INFO("相机 Bayer 滤镜排列值: {}, 转换码按对角取 {}",
                    static_cast<int>(filter_node.stCurValue.nCurValue), cvt_names[cvt_index]);
    } else {
        RM_LOG_WARN("无法查询 PixelColorFilter, 默认按 BG 转换; 若颜色异常按 x 键轮换转换码");
    }

    // 4.6 分辨率要在设置 PixelFormat 之后再读（分辨率会随像素格式变化）
    GX_INT_VALUE width_node, height_node;
    std::memset(&width_node, 0, sizeof(width_node));
    std::memset(&height_node, 0, sizeof(height_node));
    GXGetIntValue(device, "Width", &width_node);
    GXGetIntValue(device, "Height", &height_node);
    RM_LOG_INFO("当前分辨率: {}x{}", width_node.nCurValue, height_node.nCurValue);

    // 5. 取流前的准备: 数据流句柄 -> payload 大小 -> 预分配帧缓冲
    //    注意: 新版 GXGetImage 不会自动分配缓冲, pImgBuf 传 NULL 直接报 -5!
    uint32_t stream_num = 0;
    GX_DS_HANDLE stream_handle = nullptr;
    if (GXGetDataStreamNumFromDev(device, &stream_num) != GX_STATUS_SUCCESS || stream_num < 1) {
        RM_LOG_ERROR("获取数据流句柄失败");
        GXCloseDevice(device);
        GXCloseLib();
        utils::RMLOG::instance().Close();
        return -1;
    }
    GXGetDataStreamHandleFromDev(device, 1, &stream_handle);  // 数据流序号同样从 1 开始
    uint32_t payload_size = 0;
    emStatus = GXGetPayLoadSize(stream_handle, &payload_size);
    if (emStatus != GX_STATUS_SUCCESS || payload_size == 0) {
        RM_LOG_ERROR("获取 payload 大小失败: {}", GetErrorString(emStatus));
        GXCloseDevice(device);
        GXCloseLib();
        utils::RMLOG::instance().Close();
        return -1;
    }
    std::vector<unsigned char> frame_buffer(payload_size);  // 预分配（vector 自动释放, 无泄漏）
    RM_LOG_INFO("单帧 payload: {} 字节", payload_size);

    // 6. 开始取流
    emStatus = GXStreamOn(device);
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_ERROR("GXStreamOn 失败: {}", GetErrorString(emStatus));
        GXCloseDevice(device);
        GXCloseLib();
        utils::RMLOG::instance().Close();
        return -1;
    }
    RM_LOG_INFO("取流开始, 按 c 切换识别颜色、b 开调试二值图、e/d 调曝光、ESC 退出");

    // 7.5 恢复上次退出时保存的手动曝光。相机断电/重开会丢失手动曝光设置,
    //     所以 ESC 退出前把当前曝光写入 exposure_us.txt, 启动时读回来直接生效;
    //     首次运行没有该文件时跳过, 用相机当前曝光(灯条过曝发白就按 f 压低)。
    {
        std::ifstream fin("exposure_us.txt");
        double saved_us = 0.0;
        if (fin >> saved_us && saved_us > 0) {
            GX_FLOAT_VALUE node;
            std::memset(&node, 0, sizeof(node));
            if (GXGetFloatValue(device, "ExposureTime", &node) == GX_STATUS_SUCCESS) {
                double target = std::max(node.dMin, std::min(node.dMax, saved_us));
                GXSetFloatValue(device, "ExposureTime", target);
                RM_LOG_INFO("已恢复上次保存的曝光: {:.0f} us", target);
            }
        }
    }

    cv::namedWindow("daheng_camera", cv::WINDOW_NORMAL);

    // 7. 识别器: 红/蓝各一个, 按需切换（both 模式两个都跑）
    armor::ArmorDetector red_detector(armor::Color::RED);
    armor::ArmorDetector blue_detector(armor::Color::BLUE);
    enum class DetectMode { RED, BLUE, BOTH };
    DetectMode detect_mode = DetectMode::RED;
    bool show_debug_mask = false;
    bool mask_window_shown = false;  // "mask" 窗口当前是否已创建
    size_t last_armor_count = static_cast<size_t>(-1);  // 检测到新数量的装甲板时打日志

    // 8. 取帧循环: 取帧 -> Bayer转BGR -> 装甲板识别 -> 绘制 -> FPS
    GX_FRAME_DATA frame_data;
    int cam_frame_count = 0;
    bool window_ever_visible = false;  // 窗口是否曾显示过（见循环末尾的关闭检测）
    auto fps_window_start = std::chrono::steady_clock::now();
    double cam_fps = 0.0;
    double detect_fps = 0.0;  // 识别帧率（EMA: 指数滑动平均, 比瞬时值稳定）

    while (true) {
        // 1000ms 超时取一帧; 新版 GX_FRAME_DATA 没有 nSize 字段, 直接传结构体即可
        std::memset(&frame_data, 0, sizeof(frame_data));
        frame_data.pImgBuf = frame_buffer.data();  // 必须预分配, 见上文注释
        emStatus = GXGetImage(device, &frame_data, 1000);
        if (emStatus != GX_STATUS_SUCCESS) {
            RM_LOG_WARN("GXGetImage 失败({}): {}, 退出取流",
                        static_cast<int>(emStatus), GetErrorString(emStatus));
            break;
        }

        // 残帧/坏帧直接跳过（丢包帧的内容不可信）
        if (frame_data.nStatus != GX_FRAME_STATUS_SUCCESS) {
            RM_LOG_WARN("收到异常帧, 状态: {}, 跳过", frame_data.nStatus);
            continue;
        }

        const int width = static_cast<int>(frame_data.nWidth);
        const int height = static_cast<int>(frame_data.nHeight);
        if (width <= 0 || height <= 0) {
            RM_LOG_WARN("无效分辨率 {}x{}, 跳过本帧", width, height);
            continue;
        }

        // Bayer RG8（单通道）-> BGR（三通道）, 转换码由 PixelColorFilter 决定
        cv::Mat raw(height, width, CV_8UC1, frame_data.pImgBuf);
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv_conversion_code);

        // ---- 装甲板识别 ----
        auto t_detect_begin = std::chrono::steady_clock::now();
        std::vector<armor::Armor> armors;
        if (detect_mode == DetectMode::BOTH) {
            auto reds = red_detector.detect(bgr);
            auto blues = blue_detector.detect(bgr);
            armors = reds;
            armors.insert(armors.end(), blues.begin(), blues.end());
        } else {
            auto &active = (detect_mode == DetectMode::RED) ? red_detector : blue_detector;
            armors = active.detect(bgr);
        }
        auto t_detect_end = std::chrono::steady_clock::now();
        double dt_detect = std::chrono::duration<double>(t_detect_end - t_detect_begin).count();
        if (dt_detect > 1e-9) {
            detect_fps = detect_fps <= 0.0 ? 1.0 / dt_detect
                                           : detect_fps * 0.9 + (1.0 / dt_detect) * 0.1;
        }

        // 检测到的装甲板数量变化时, 把四角坐标打到日志（可供串口/上位机对接）
        if (armors.size() != last_armor_count) {
            last_armor_count = armors.size();
            RM_LOG_INFO("识别到 {} 个装甲板:", armors.size());
            for (size_t k = 0; k < armors.size(); ++k) {
                const auto &a = armors[k];
                RM_LOG_INFO("  [{}] {} score={:.2f} w/h={:.2f} "
                            "corners: TL({:.1f},{:.1f}) TR({:.1f},{:.1f}) "
                            "BR({:.1f},{:.1f}) BL({:.1f},{:.1f})",
                            k, a.color == armor::Color::RED ? "RED" : "BLUE",
                            a.score, a.width / a.height,
                            a.corners[0].x, a.corners[0].y,
                            a.corners[1].x, a.corners[1].y,
                            a.corners[2].x, a.corners[2].y,
                            a.corners[3].x, a.corners[3].y);
            }
        }

        // 绘制识别结果: 外框 + 四角 + 灯条中点 + 标签
        armor::ArmorDetector::drawArmors(bgr, armors);

        // 相机帧率统计: 每满 1s 计算一次并打日志
        ++cam_frame_count;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_window_start).count();
        if (elapsed >= 1.0) {
            cam_fps = cam_frame_count / elapsed;
            cam_frame_count = 0;
            fps_window_start = now;
            RM_LOG_INFO("相机帧率: {:.1f} FPS | 识别帧率: {:.1f} FPS | 当前装甲板: {}",
                        cam_fps, detect_fps, armors.size());
        }

        // 叠加文字: 识别帧率 / 装甲板数量 / 识别颜色
        std::string mode_text =
            detect_mode == DetectMode::BOTH ? "both" :
            detect_mode == DetectMode::RED ? "red" : "blue";
        cv::putText(bgr, "detect FPS: " + std::to_string(detect_fps).substr(0, 5) +
                             "  armor: " + std::to_string(armors.size()) +
                             "  color: " + mode_text,
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // 诊断行: 颜色掩码像素数 / 灯条候选数。
        // mask=0 -> 颜色分割没抓到(曝光/颜色问题); bars<2 -> 分割到了但灯条筛选没过
        std::string diag;
        if (detect_mode == DetectMode::BOTH) {
            diag = "red mask:" + std::to_string(cv::countNonZero(red_detector.lastMask())) +
                   " bars:" + std::to_string(red_detector.lastBarCount()) +
                   " | blue mask:" + std::to_string(cv::countNonZero(blue_detector.lastMask())) +
                   " bars:" + std::to_string(blue_detector.lastBarCount());
        } else {
            auto &d = (detect_mode == DetectMode::RED) ? red_detector : blue_detector;
            diag = "mask:" + std::to_string(cv::countNonZero(d.lastMask())) +
                   " px  bars:" + std::to_string(d.lastBarCount());
        }
        diag += "  cvt:" + std::string(cvt_names[cvt_index]);
        cv::putText(bgr, diag, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

        cv::imshow("daheng_camera", bgr);

        // 调试窗口: 当前颜色分割二值图（both 模式红蓝左右拼接）
        if (show_debug_mask) {
            if (detect_mode == DetectMode::BOTH) {
                cv::Mat dr = red_detector.lastMask();
                cv::Mat db = blue_detector.lastMask();
                cv::Mat dbg(dr.rows, dr.cols * 2, CV_8UC1, cv::Scalar(0));
                dr.copyTo(dbg(cv::Rect(0, 0, dr.cols, dr.rows)));
                db.copyTo(dbg(cv::Rect(dr.cols, 0, db.cols, db.rows)));
                cv::imshow("mask", dbg);
            } else {
                auto &active = (detect_mode == DetectMode::RED) ? red_detector : blue_detector;
                cv::imshow("mask", active.lastMask());
            }
            mask_window_shown = true;
        } else if (mask_window_shown) {
            // 只在从"显示"切换到"不显示"时销毁一次;
            // 若每帧都调 destroyWindow, GTK 会对未注册的窗口反复报错
            cv::destroyWindow("mask");
            mask_window_shown = false;
        }

        bool quit = false;
        switch (cv::waitKey(1) & 0xFF) {
            case 27: quit = true; break;                     // ESC 退出
            case 'c':                                        // 切换识别颜色 red->blue->both
                detect_mode = detect_mode == DetectMode::RED ? DetectMode::BLUE :
                              detect_mode == DetectMode::BLUE ? DetectMode::BOTH :
                                                                DetectMode::RED;
                last_armor_count = static_cast<size_t>(-1);  // 强制重打一次识别日志
                RM_LOG_INFO("识别颜色切换到 {}",
                            detect_mode == DetectMode::BOTH ? "both" :
                            detect_mode == DetectMode::RED ? "red" : "blue");
                break;
            case 'b': show_debug_mask = !show_debug_mask; break;  // 调试二值图
            case 'p': {  // 保存当前帧与掩码到文件, 供离线调参
                cv::imwrite("armor_capture.png", bgr);
                if (detect_mode == DetectMode::BOTH) {
                    cv::imwrite("armor_mask_red.png", red_detector.lastMask());
                    cv::imwrite("armor_mask_blue.png", blue_detector.lastMask());
                } else {
                    auto &d = (detect_mode == DetectMode::RED) ? red_detector : blue_detector;
                    cv::imwrite("armor_mask.png", d.lastMask());
                }
                RM_LOG_INFO("已保存当前帧: armor_capture.png / armor_mask*.png");
                break;
            }
            case 'x':  // 颜色不对时轮换 Bayer 转换码, 现场找正确排列
                cvt_index = (cvt_index + 1) % cvt_codes.size();
                cv_conversion_code = cvt_codes[cvt_index];
                RM_LOG_INFO("Bayer 转换码切换为 {}", cvt_names[cvt_index]);
                break;
            case 'f': {  // 灯条过曝一键修复: 按当前帧过曝占比把曝光压低一档, 可连按
                cv::Mat over;
                cv::inRange(bgr, cv::Scalar(220, 220, 220), cv::Scalar(255, 255, 255), over);
                double ratio = cv::countNonZero(over) /
                               static_cast<double>(std::max(1, bgr.rows * bgr.cols));
                GX_FLOAT_VALUE node;
                std::memset(&node, 0, sizeof(node));
                if (GXGetFloatValue(device, "ExposureTime", &node) == GX_STATUS_SUCCESS) {
                    double factor = ratio > 0.05 ? 0.4 : (ratio > 0.002 ? 0.65 : 1.0);
                    double target = std::max(node.dMin, node.dCurValue * factor);
                    GXSetFloatValue(device, "ExposureTime", target);
                    RM_LOG_INFO("过曝像素占比 {:.1f}%, 曝光 {:.0f}us -> {:.0f}us"
                                "{}", ratio * 100.0, node.dCurValue, target,
                                factor >= 1.0 ? " (已达标, 无需再调)" : ", 若灯条仍发白可再按 f");
                }
                break;
            }
            case 'e': AddExposureTime(device, 25.0); break;  // 曝光 +25us
            case 'd': AddExposureTime(device, -25.0); break; // 曝光 -25us
            case 'a': AddGain(device, 0.1); break;           // 增益 +0.1dB
            case 'q': AddGain(device, -0.1); break;          // 增益 -0.1dB
            case 's': AddGamma(device, 0.1); break;          // 伽马 +0.1
            case 'w': AddGamma(device, -0.1); break;         // 伽马 -0.1
            default: break;
        }
        // 窗口被鼠标点掉也算退出。注意: 首帧时 GTK 可能还没完成窗口映射,
        // 直接判断会误报"窗口关闭", 所以要等它显示过一次再启用检测
        double window_visible = cv::getWindowProperty("daheng_camera", cv::WND_PROP_VISIBLE);
        if (window_visible >= 1.0) {
            window_ever_visible = true;
        } else if (window_ever_visible) {
            RM_LOG_INFO("显示窗口被关闭, 退出取流");
            quit = true;
        }
        if (quit) break;
    }

    // 9. 退出前保存当前曝光, 下次启动直接恢复（相机重开会丢失手动曝光设置）
    {
        GX_FLOAT_VALUE node;
        std::memset(&node, 0, sizeof(node));
        if (GXGetFloatValue(device, "ExposureTime", &node) == GX_STATUS_SUCCESS) {
            std::ofstream fout("exposure_us.txt", std::ios::trunc);
            fout << node.dCurValue;
            RM_LOG_INFO("已保存当前曝光: {:.0f} us -> exposure_us.txt", node.dCurValue);
        }
    }

    // 10. 逆序清理: 停流 -> 关设备 -> 关库（与 InitLib 配对）
    GXStreamOff(device);
    GXCloseDevice(device);
    GXCloseLib();
    RM_LOG_INFO("相机已关闭, demo 结束");
    utils::RMLOG::instance().Close();
    return 0;
}
