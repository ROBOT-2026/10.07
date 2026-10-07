// ============================================================================
// armor_detector.h — 装甲板传统识别核心
// 基于装甲板灯条的 BGR 颜色信息（通道差阈值）+ 几何特征，
// 识别装甲板、输出外框与四角，并支持绘制。
//
// 流程: BGR 颜色分割 -> 形态学清理 -> 轮廓/灯条筛选 -> 灯条配对 -> 四角解算 -> NMS
// ============================================================================

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

namespace armor {

// 灯条颜色
enum class Color {
    RED = 0,
    BLUE = 1,
};

// ----------------------------------------------------------------------------
// 灯条（Light Bar）：装甲板左右两侧（板正放）或上下两侧（板横置）的发光条形目标。
// 几何量全部从最小外接矩形本身推导，与灯条朝向无关（横竖都成立）：
//   axis  = 长轴单位向量，统一翻到指向下半平面（y>=0）
//   edgeA = -perp 侧长边的两端点（perp=(axis.y,-axis.x)：竖条时是左侧边，横条时是下边）
//   edgeB = +perp 侧长边的两端点（竖条时是右侧边，横条时是上边）
//   两端点按 axis 方向排序（[0]=起点，[1]=终点）
// ----------------------------------------------------------------------------
struct LightBar {
    cv::RotatedRect rect;      // 灯条最小外接矩形
    cv::Point2f center;        // 灯条中心（外接矩形中心）
    cv::Point2f axis;          // 长轴单位向量（指向下半平面）
    cv::Point2f edgeA[2];      // -perp 侧长边两端点
    cv::Point2f edgeB[2];      // +perp 侧长边两端点
    double length = 0.0;       // 灯条长边长度（像素）
    double width = 0.0;        // 灯条短边宽度（像素）
    double angle = 0.0;        // 灯条长轴相对竖直方向的角度（度，0=竖直，90=水平）
    double area = 0.0;         // 轮廓面积
    double aspectRatio = 0.0;  // 长/宽
    double solidity = 0.0;     // 矩形度 = 轮廓面积 / 外接矩形面积
};

// ----------------------------------------------------------------------------
// 装甲板（Armor）：由左右两根灯条匹配而成
// ----------------------------------------------------------------------------
struct Armor {
    Color color = Color::RED;
    LightBar left;             // 左侧灯条
    LightBar right;            // 右侧灯条
    cv::Point2f corners[4];    // 装甲板四角：0=左上 1=右上 2=右下 3=左下
    cv::RotatedRect rect;      // 装甲板外接矩形（用于绘制外框）
    double width = 0.0;        // 装甲板宽度（两灯条中心间距）
    double height = 0.0;       // 装甲板高度（两灯条长度均值）
    double centerAngle = 0.0;  // 装甲板中心倾角（度）
    double score = 0.0;        // 匹配得分（越高越可信）
};

// ----------------------------------------------------------------------------
// 颜色分割参数（BGR 通道差阈值）
// 灯条 LED 颜色饱和度高: 红(R 明显大于 G/B)、蓝(B 明显大于 R/G)
// ----------------------------------------------------------------------------
struct ColorParams {
    // 注意: 实际生效的默认值按颜色区分, 在 ArmorDetector 构造函数里设置
    // (MER2 相机蓝灯条绿泄漏严重, 蓝色需要更小的 channelDiff, 详见 .cpp)
    int channelDiff = 80;   // 主通道与另两个通道的最小差值
    int minBright = 90;     // 主通道亮度下限（滤除暗区/阴影）
};

// ----------------------------------------------------------------------------
// 灯条筛选参数（按需调节，配合不同距离/相机分辨率）
// ----------------------------------------------------------------------------
struct LightBarParams {
    double minArea = 80.0;       // 灯条轮廓最小面积
    double maxArea = 15000.0;    // 灯条轮廓最大面积
    double minAspect = 2.0;      // 长宽比下限（灯条应为细长条）
    double maxAspect = 12.0;     // 长宽比上限
    // 注意: 不按"倾角"筛单条灯条——板横置时灯条是水平的(与竖直差 90°)，
    // 倾角约束由配对时的"两灯条角度差"承担。
    double minSolidity = 0.35;   // 矩形度下限（滤除不规则形状）
};

// ----------------------------------------------------------------------------
// 灯条匹配（成对组合成装甲板）参数
// ----------------------------------------------------------------------------
struct ArmorMatchParams {
    double maxHeightRatio = 0.6;   // 两灯条长度差 / 较长者，上限
    double maxAngleDiff = 15.0;    // 两灯条角度差上限（度）
    double minVOverlap = 0.4;      // 轴向投影重合度下限（重叠长度/平均长度，沿灯条长轴）
    double minWH = 0.6;            // 装甲板宽高比下限（中心垂距/灯条长）
    double maxWH = 3.2;            // 装甲板宽高比上限
    double maxCenterYDiff = 1.0;   // 两灯条中心沿轴向错位 / 平均长度，上限
    double targetWH = 2.0;         // 期望宽高比（用于打分，标准装甲板约 2~2.6）
};

// ----------------------------------------------------------------------------
// 识别器
// ----------------------------------------------------------------------------
class ArmorDetector {
public:
    explicit ArmorDetector(Color color = Color::RED);

    // 主接口：输入 BGR 帧，输出识别到的装甲板（按得分降序，已做 NMS 去重）
    std::vector<Armor> detect(const cv::Mat& bgr);

    // 绘制：外框 + 角点 + 灯条中点 + 标签
    static void drawArmors(cv::Mat& img, const std::vector<Armor>& armors);

    // 返回最近一次颜色分割的二值图（用于调试显示）
    cv::Mat lastMask() const;

    // 返回最近一次检测找到的灯条候选数（筛选后、配对前，调试用）
    int lastBarCount() const { return barCount_; }

    void setColor(Color c) { color_ = c; }
    Color color() const { return color_; }
    void setColorParams(const ColorParams& p) { cp_ = p; }
    void setLightBarParams(const LightBarParams& p) { lbp_ = p; }
    void setMatchParams(const ArmorMatchParams& p) { amp_ = p; }

private:
    // 直接用 BGR 通道差阈值得到颜色二值掩码（含形态学清理）
    cv::Mat colorMask(const cv::Mat& bgr, Color color) const;

    // 在二值图上找灯条候选
    std::vector<LightBar> findLightBars(const cv::Mat& mask) const;

    // 灯条两两匹配成装甲板
    void matchArmors(const std::vector<LightBar>& bars, std::vector<Armor>& armors) const;

    // 非极大值抑制：去掉重叠的重复装甲板
    static void nms(std::vector<Armor>& armors, double iouThresh = 0.35);

    Color color_;
    ColorParams cp_;
    LightBarParams lbp_;
    ArmorMatchParams amp_;
    cv::Mat lastMask_;
    int barCount_ = 0;
};

}  // namespace armor
