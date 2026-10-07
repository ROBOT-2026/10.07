# daheng_camera — 大恒工业相机拉流 + 装甲板识别

培训任务「复现配置并调用大恒工业相机，将调试信息更换为 rm_log 格式」+「装甲板识别」的
可运行工程。基于官方示例流程实现，并做了**新版 SDK 2.6 适配** + **rm_log 集成** +
**BGR 灯条装甲板识别**（外框 + 角点 + 识别帧率）。

## 一、功能

| 步骤 | 说明 |
|---|---|
| 1. 初始化 | `GXInitLib()` / `GXCloseLib()` |
| 2. 枚举 | `GXUpdateAllDeviceList()` + `GXGetDeviceInfo()`，打印每台相机的型号与序列号 |
| 3. 打开 | `GXOpenDevice(GX_OPEN_PARAM*)` 按序列号打开（新版 API） |
| 4. 配置 | 关自动曝光/增益、连续采集、关触发、Bayer RG8 原始输出、放开传输限速 |
| 5. 取流 | `GXStreamOn` + `GXGetImage` 循环（预分配帧缓冲） |
| 6. 显示 | OpenCV `cvtColor` 做 Bayer→BGR（按 `PixelColorFilter` 查对角转换码，颜色异常可按 x 轮换），叠加实时 FPS |
| 7. 装甲板识别 | BGR 通道差颜色分割 → 灯条筛选 → 配对 → 四角解算 → NMS，绘制外框与角点 |
| 8. 调参 | 键盘实时调曝光 / 增益 / 伽马、切换识别颜色、f 一键压曝光、p 存图离线调参 |
| 9. 日志 | 全部调试信息换成 `RM_LOG_INFO / WARN / ERROR`（封装 spdlog），无 `std::cout` |

## 二、构建

```bash
cd ~/Downloads/daheng_camera
mkdir build && cd build
cmake .. -DDAHENG_SDK_DIR=/home/robot/Downloads   # 不指定则默认 ~/Downloads
make -j
```

## 三、运行（相机已连好）

```bash
./daheng_demo                  # 按默认序列号 NE0190010001 打开
./daheng_demo NE0190010001     # 按指定序列号打开
```

默认序列号不在枚举列表里（比如换了台相机）且没传参数时，会自动改用枚举到的
第一台相机并打 WARN 提示；传了参数但序列号不对则直接报错退出。

已通过 rpath 链接 SDK 库，无需再 `export LD_LIBRARY_PATH`。
打开窗口后的按键：

| 按键 | 功能 |
|---|---|
| **c** | 切换识别颜色：红 → 蓝 → 红蓝双色（默认红） |
| **b** | 开关颜色分割二值图调试窗口 |
| **p** | 保存当前帧与掩码到 PNG（`armor_capture.png` / `armor_mask*.png`，离线调参用） |
| **f** | 灯条过曝一键修复：按过曝像素占比自动压低曝光，可连按 |
| **x** | 颜色不对时轮换 Bayer 转换码（BG/GR/GB/RG），现场找到正确排列 |
| **e / d** | 曝光 ±25 us（**a/q** 增益 ±0.1 dB，**s/w** 伽马 ±0.1） |
| **ESC** | 退出 |

**曝光持久化**：相机断电/重开会丢失手动曝光设置，因此 ESC 正常退出前会把当前曝光
写入 `exposure_us.txt`，下次启动自动恢复（首次运行没有该文件时用相机当前曝光）。
注意只有 ESC 干净退出才保存，Ctrl+C 强杀不保存。

日志同时输出到彩色终端和 `logs/daheng_camera.log`（info 级别走 stdio 缓冲，
按 ESC 正常退出或出现 WARN 以上级别时才会强制落盘）。

## 四、装甲板识别（Task 2）

识别核心在 `armor_detector.h / .cpp`（`armor` 命名空间），只依赖 OpenCV，可独立复用。
利用装甲板灯条的 **BGR 颜色信息**（不转 HSV）做分割，流程：

```
BGR 通道差阈值分割 → 形态学开/闭 → findContours → 灯条筛选(minAreaRect)
→ 灯条配对(几何约束 + 互最优) → 四角解算 → NMS → 绘制外框 + 角点 + 标签
```

| 环节 | 关键逻辑 |
|---|---|
| 颜色分割 | 通道差阈值 + 主通道亮度下限，**红/蓝参数分开**（见下表）。灯条 LED 饱和度高，通道差比 HSV 更快更稳 |
| 灯条筛选 | 面积 80~45000、长宽比 2~12、矩形度≥0.25。几何量（长轴方向/长边端点）从外接矩形棱边直接推导，不按"相对竖直倾角"筛单条灯条——板横置时灯条是水平的 |
| 配对 | 长度差≤60%、角度差≤15°（两灯条朝向一致）、沿长轴投影重合≥40%、轴向错位≤1×平均长度、宽高比(中心垂距/灯条长) 0.6~3.2，互最优去跨板错配。约束全部沿灯条长轴方向推广，**板正放（左右布局）与板横置（上下布局）都能配对** |
| 四角 | 按两灯条中心的相对位置分布局：左右布局取左条外侧边+右条外侧边；上下布局取上条上边+下条下边 → 左上/右上/右下/左下，角点变化时打日志（可对接串口/上位机） |
| 帧率 | 识别帧率用 EMA（指数滑动平均）平滑，每秒与相机帧率一起打日志 |

**分颜色默认参数**（MER2 相机实测：蓝灯条绿泄漏严重、B-G 只有 25~70，且中心过曝发白，
按统一阈值会全部漏检）：

| 参数 | 红色灯条 | 蓝色灯条 | 原因 |
|---|---|---|---|
| `channelDiff` | 70 | 35 | 蓝灯条绿泄漏，蓝绿差按 80 全被滤掉；白墙差≈0 不会误检 |
| `minBright` | 80 | 100 | 蓝色亮度下限稍高，滤掉暗部噪点 |
| `minSolidity` | 0.35 | 0.25 | 蓝灯条过曝有晕边，矩形度偏低（实测 0.30 上下） |
| `maxArea` | 45000 | 35000 | 近距/过曝发胖的灯条面积可达 29k~37k，默认 15k 会误杀 |

参数都在 `armor_detector.h` 的三个结构体里（`ColorParams / LightBarParams /
ArmorMatchParams`），可按距离、光照、分辨率调。画面左下角第二行有诊断信息：
`mask:xxxx px bars:x`——mask=0 是分割没抓到（曝光/颜色问题，按 f 压曝光或按 x 换
Bayer 码），bars<2 是分割到了但筛选没过。实测 1920x1200 下：
相机 ~99 FPS、识别 300+ FPS，识别不拖相机帧率。

## 五、新版 SDK 2.6 与官方旧版 demo 的差异（关键）

官方旧 demo 用旧版 API，直接照抄会编译不过。本工程的适配点：

| 功能 | 旧版（官方 demo） | 新版（本工程） |
|---|---|---|
| 打开设备 | `GXOpenDeviceByIndex(i, &dev)` | `GXOpenDevice(&op, &dev)`，`GX_OPEN_PARAM.pszContent` 传序列号 |
| 枚举 | `GXUpdateDeviceList` | `GXUpdateAllDeviceList`（旧名已废弃） |
| 帧结构 | `GX_FRAME_DATA` 有 `nSize` | 新版无 `nSize`，且 `GXGetImage` 要求**调用者预分配 `pImgBuf`** |
| Bayer 转 RGB | `DxImageProc` 的 `DxRaw8toRGB24Ex` | OpenCV `cvtColor`（`COLOR_BayerXX2BGR`） |
| 头文件 / 库 | `GxIAPI.h` / `libgxiapi.so` | 不变（SDK 的 `inc/` 与 `lib/x86_64/` 下） |

## 六、目录结构

```
daheng_camera/
├── CMakeLists.txt
├── daheng_demo.cpp        # 主程序（新版 SDK + rm_log + 识别调度）
├── armor_detector.h/.cpp  # 装甲板识别核心（只依赖 OpenCV，可独立复用）
└── util/
    ├── rm_log/            # 官方 rm_log 封装
    │   ├── include/rm_log.h
    │   └── src/rm_log.cpp
    └── spdlog/            # spdlog 头文件库（header-only，含 fmt）
```

## 七、常见问题（踩坑记录）

- **`GXOpenDevice` 无限挂起（日志停在枚举之后）**：多为相机固件还残留着「幽灵独占」状态
  ——典型场景：GalaxyView / demo 被 kill 时没走正常关闭流程，相机侧连接仲裁没释放，之后
  EXCLUSIVE 模式打开会在固件上死等（重插相机也可能不恢复）。解决：用 SDK 自带的
  **Python 版 SDK（gxipy）正常 open + close 一次**（`open_device_by_index` 默认 CONTROL
  模式即可成功），干净断开会把残留状态清掉；再不行就重启系统。
- **打开报 `-5`（INVALID_PARAMETER）**：`GX_OPEN_PARAM.pszContent` 为空或格式不对；`GXGetImage` 报 `-5` 则是 `pImgBuf` 传了 NULL（本工程已预分配，不会出现）。
- **打开报 `-14`（TIMEOUT）**：装完 `Galaxy_camera.run` 后**必须重启系统**，USB3 Vision 传输层才生效；或重新插拔相机。
- **打开报 `-1004`（TL Error: The device has been open）**：相机被别的进程独占（GalaxyView 查看器、另一个 demo 实例等），关掉占用进程即可。
- **颜色错乱 / 偏色**：Bayer 排列转错了。注意本机 MER2-230-168U3C-L 实测 SDK 上报的
  `PixelColorFilter`（=RG）与实际 CFA 排列**成对角互换**，按上报值直转会红蓝颠倒，
  所以本工程查表时取对角码（报 RG 用 BG）；换相机仍异常就在运行中按 **x** 键轮换
  BG/GR/GB/RG，颜色正确即所得。
- **灯条不出框（识别不到）**：先看画面左下角黄色诊断行。`mask=0 px` 是颜色分割没抓到
  ——多半灯条过曝发白（LED 饱和后颜色烧白），按 **f** 压低曝光（可连按）；`bars:x<2`
  是分割到了但灯条筛选没过，按 **p** 存图后用 `/tmp/armor_sim.py` 离线复算各项
  指标。另外确认相机重启后曝光已恢复（见「三、运行」的曝光持久化）。
- **帧率上不去**：相机出厂默认限速，本工程已把 `DeviceLinkThroughputLimit` 设为最大值；另外确认插在 **USB3.0 蓝色口**上。
- **枚举不到设备**：检查 `lsusb` 是否出现 Daheng Imaging；无权限时用 `sudo` 或配 udev 规则。
- **日志文件没生成**：`INIT_LOG` 之前先 `std::filesystem::create_directories("logs")`（本工程已处理），spdlog 不会自动建目录。
- **日志文件里 info 不更新**：spdlog 异步 + stdio 缓冲，info 级别攒满缓冲或遇到 WARN 才落盘；正常退出（ESC）会全部写入，用 `timeout` 强杀会丢尾部 info。
