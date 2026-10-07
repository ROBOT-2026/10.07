// ============================================================================
// armor_detector.cpp — 装甲板传统识别核心实现
// 流程: BGR 颜色分割 -> 形态学清理 -> 轮廓/灯条筛选 -> 灯条配对 -> 四角解算 -> NMS
// ============================================================================

#include "armor_detector.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace armor {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

// ----------------------------------------------------------------------------
ArmorDetector::ArmorDetector(Color color) : color_(color) {
    // MER2 相机实测: 蓝色灯条绿泄漏严重(B-G 只有 25~70, 蓝绿差按默认 80 全被滤掉),
    // 且灯条中心会过曝发白; 红色灯条则正常。因此按颜色给不同的默认阈值:
    if (color == Color::BLUE) {
        cp_.channelDiff = 35;    // 蓝灯条: B-G/B-R 差 35 即可(白墙差≈0 不会误检)
        cp_.minBright = 100;     // 亮度下限稍高, 滤掉暗部蓝色噪点
        lbp_.minSolidity = 0.25; // 蓝灯条过曝有晕边, 矩形度偏低(实测 0.30 上下)
        lbp_.maxArea = 35000.0;  // 近距离蓝灯条面积可达 29k(默认 15k 会误杀)
    } else {
        cp_.channelDiff = 70;    // 红灯条: 比默认略放宽, 缓解 LED PWM 闪烁造成的丢帧
        cp_.minBright = 80;
        lbp_.maxArea = 45000.0;  // 红灯条过曝发胖时面积可达 37k(横置板实测, 默认 15k 会误杀)
    }
}

cv::Mat ArmorDetector::lastMask() const { return lastMask_; }

// ----------------------------------------------------------------------------
// 1. 颜色分割：直接用 BGR 通道差阈值。
//    装甲板灯条是 LED，颜色饱和度很高：
//      红色灯条: R 明显大于 G 和 B（R-G、R-B 都超过阈值）
//      蓝色灯条: B 明显大于 R 和 G（B-R、B-G 都超过阈值）
//    不转 HSV 也能很好地把灯条从场景里抠出来，且少一次色彩空间转换。
//    若现场光照导致分割不稳定，可改回 HSV inRange（红: 0~10 与 155~180 两段）。
// ----------------------------------------------------------------------------
cv::Mat ArmorDetector::colorMask(const cv::Mat& bgr, Color color) const {
    std::vector<cv::Mat> ch;  // ch[0]=B, ch[1]=G, ch[2]=R
    cv::split(bgr, ch);

    cv::Mat diff1, diff2;
    if (color == Color::RED) {
        cv::subtract(ch[2], ch[1], diff1);  // R - G
        cv::subtract(ch[2], ch[0], diff2);  // R - B
    } else {                                 // BLUE
        cv::subtract(ch[0], ch[2], diff1);  // B - R
        cv::subtract(ch[0], ch[1], diff2);  // B - G
    }
    cv::threshold(diff1, diff1, cp_.channelDiff, 255, cv::THRESH_BINARY);
    cv::threshold(diff2, diff2, cp_.channelDiff, 255, cv::THRESH_BINARY);

    cv::Mat mask;
    cv::bitwise_and(diff1, diff2, mask);

    // 主通道亮度下限：滤掉阴影、暗部等误检
    const cv::Mat& main = (color == Color::RED) ? ch[2] : ch[0];
    cv::Mat bright;
    cv::threshold(main, bright, cp_.minBright, 255, cv::THRESH_BINARY);
    cv::bitwise_and(mask, bright, mask);

    // 形态学清理：
    // 1) 先开运算（腐蚀后膨胀）去除离散噪点，避免噪点先被连通成块；
    // 2) 再闭运算（膨胀后腐蚀）连接灯条内部因反光产生的碎洞。
    cv::Mat kOpen = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kClose = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kOpen);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kClose);
    return mask;
}

// ----------------------------------------------------------------------------
// 2. 灯条提取：轮廓 -> 面积/长宽比/矩形度筛选。
//    几何量（长轴方向、两条长边的端点）直接从最小外接矩形的棱边推导，与朝向无关。
//    不能按 y 排序顶点：那只对竖直灯条成立；板横置时灯条水平，按 y 排序会错把
//    左右两端当上下边、倾角算成 ~78° 被误杀（横置板实测踩过）。
// ----------------------------------------------------------------------------
std::vector<LightBar> ArmorDetector::findLightBars(const cv::Mat& mask) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> bars;
    bars.reserve(contours.size());

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < lbp_.minArea || area > lbp_.maxArea) continue;

        cv::RotatedRect rr = cv::minAreaRect(c);
        double w = rr.size.width, h = rr.size.height;
        if (h < w) std::swap(w, h);  // h = 长边, w = 短边
        if (h <= 1e-6) continue;

        double aspect = h / w;
        if (aspect < lbp_.minAspect || aspect > lbp_.maxAspect) continue;

        double solidity = area / (h * w);
        if (solidity < lbp_.minSolidity) continue;

        // 长轴单位向量：取矩形相邻两边中较长的那条（不依赖 minAreaRect 的
        // angle 约定），统一翻到指向下半平面（y>=0）
        cv::Point2f pts[4];
        rr.points(pts);
        cv::Point2f e1 = pts[1] - pts[0];
        cv::Point2f e2 = pts[2] - pts[1];
        cv::Point2f axis = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
        axis *= 1.0 / std::max(1e-6, static_cast<double>(cv::norm(axis)));
        if (axis.y < 0) axis = -axis;

        // 两条长边（长度≈h）：perp 坐标小的一侧记 edgeA、大的一侧记 edgeB。
        // perp = (axis.y, -axis.x)：竖条时指向右（edgeA=左边），横条时指向上（edgeA=下边）
        cv::Point2f perp(axis.y, -axis.x);
        cv::Point2f longEdges[2][2];
        int nLong = 0;
        for (int i = 0; i < 4; ++i) {
            cv::Point2f d = pts[(i + 1) % 4] - pts[i];
            if (cv::norm(d) > h * 0.5) {
                longEdges[nLong][0] = pts[i];
                longEdges[nLong][1] = pts[(i + 1) % 4];
                ++nLong;
            }
        }
        if (nLong != 2) continue;  // 长宽比>=2 时必有两条长边, 防御

        cv::Point2f mid0 = (longEdges[0][0] + longEdges[0][1]) * 0.5f;
        cv::Point2f mid1 = (longEdges[1][0] + longEdges[1][1]) * 0.5f;
        int iA = (mid0.dot(perp) <= mid1.dot(perp)) ? 0 : 1;
        int iB = 1 - iA;

        LightBar bar;
        bar.rect = rr;
        bar.center = rr.center;
        bar.axis = axis;
        // 每条长边的两端点按 axis 排序：[0]=轴坐标小的一端（竖条时是上端，横条时是左端）
        auto sortedEnds = [&axis](cv::Point2f p, cv::Point2f q, cv::Point2f out[2]) {
            if (p.dot(axis) <= q.dot(axis)) {
                out[0] = p;
                out[1] = q;
            } else {
                out[0] = q;
                out[1] = p;
            }
        };
        sortedEnds(longEdges[iA][0], longEdges[iA][1], bar.edgeA);
        sortedEnds(longEdges[iB][0], longEdges[iB][1], bar.edgeB);
        bar.length = h;
        bar.width = w;
        bar.angle = std::fabs(std::atan2(axis.x, axis.y)) * 180.0 / kPi;  // 0=竖直, 90=水平
        bar.area = area;
        bar.aspectRatio = aspect;
        bar.solidity = solidity;

        bars.push_back(bar);
    }
    return bars;
}

// ----------------------------------------------------------------------------
// 3. 灯条配对：长度接近、角度接近、轴向投影重合、轴向错位小、垂直间距合理。
//    所有几何约束沿灯条长轴方向（而非固定竖直方向）推广，板正放（左右布局）
//    与板横置（上下布局）都能配成对。
//    配对后做"互最优"过滤：一根灯条只保留得分最高的配对，
//    可排除相邻两块板"左灯条配左灯条"这类跨板错配。
// ----------------------------------------------------------------------------
void ArmorDetector::matchArmors(const std::vector<LightBar>& bars,
                                std::vector<Armor>& armors) const {
    struct Candidate {
        int i, j;
        Armor armor;
    };
    std::vector<Candidate> cands;
    const int n = static_cast<int>(bars.size());

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const LightBar& a = bars[i];
            const LightBar& b = bars[j];

            // 长度差
            double lenMax = std::max(a.length, b.length);
            double hRatio = std::fabs(a.length - b.length) / lenMax;
            if (hRatio > amp_.maxHeightRatio) continue;

            // 角度差（angle 都是长轴相对竖直的角度，横竖朝向均可比较）
            if (std::fabs(a.angle - b.angle) > amp_.maxAngleDiff) continue;

            double avgLen = (a.length + b.length) * 0.5;
            cv::Point2f delta = b.center - a.center;
            double centerDist = cv::norm(delta);

            // 把中心连线分解到 a 的长轴方向（along）与垂直方向（perp）。
            //   沿轴向错位 along：竖条时≈中心竖直错位，与旧逻辑一致；
            //   横置板上下灯条错位≈0，也能通过
            double along = std::fabs(delta.x * a.axis.x + delta.y * a.axis.y);
            if (along / avgLen > amp_.maxCenterYDiff) continue;

            // 轴向投影重合（旧"竖直重合"沿长轴方向的推广）：
            // 竖条=竖直重叠；横条=水平重叠（横置板的上下灯条在水平方向完全重叠）
            double projA = a.center.x * a.axis.x + a.center.y * a.axis.y;
            double projB = b.center.x * a.axis.x + b.center.y * a.axis.y;
            double overlap = std::min(projA + a.length * 0.5, projB + b.length * 0.5) -
                             std::max(projA - a.length * 0.5, projB - b.length * 0.5);
            if (overlap / avgLen < amp_.minVOverlap) continue;

            // 垂直间距 -> 宽高比
            double perp = std::sqrt(std::max(0.0, centerDist * centerDist - along * along));
            double wh = perp / avgLen;
            if (wh < amp_.minWH || wh > amp_.maxWH) continue;

            // ---- 通过全部约束，生成装甲板 ----
            Candidate c;
            c.i = i;
            c.j = j;
            Armor& armor = c.armor;
            armor.color = color_;
            armor.left = a;
            armor.right = b;
            // 若 a 在 b 右边则交换，保证 left/right 语义
            if (a.center.x > b.center.x) {
                armor.left = b;
                armor.right = a;
            }
            armor.height = (armor.left.length + armor.right.length) * 0.5;
            armor.width = cv::norm(armor.left.center - armor.right.center);
            armor.centerAngle =
                std::atan2(armor.right.center.y - armor.left.center.y,
                           armor.right.center.x - armor.left.center.x) *
                180.0 / kPi;

            // ---- 四角解算 ----
            // 按两灯条中心的相对位置分两种布局（edge 端点已按 axis 排序，
            // [0]=轴坐标小的一端，排列正好是 左上/右上/右下/左下）：
            //   左右布局（板正放, 竖条）: 左条取 edgeA(外侧左边), 右条取 edgeB(外侧右边)
            //   上下布局（板横置, 横条）: 上条取 edgeB(外侧上边), 下条取 edgeA(外侧下边)
            bool stacked = std::fabs(armor.right.center.y - armor.left.center.y) >
                           std::fabs(armor.right.center.x - armor.left.center.x);
            if (stacked) {
                const LightBar& top = armor.left.center.y <= armor.right.center.y
                                          ? armor.left
                                          : armor.right;
                const LightBar& bot = armor.left.center.y <= armor.right.center.y
                                          ? armor.right
                                          : armor.left;
                armor.corners[0] = top.edgeB[0];  // 左上
                armor.corners[1] = top.edgeB[1];  // 右上
                armor.corners[2] = bot.edgeA[1];  // 右下
                armor.corners[3] = bot.edgeA[0];  // 左下
            } else {
                armor.corners[0] = armor.left.edgeA[0];   // 左上
                armor.corners[1] = armor.right.edgeB[0];  // 右上
                armor.corners[2] = armor.right.edgeB[1];  // 右下
                armor.corners[3] = armor.left.edgeA[1];   // 左下
            }

            armor.rect = cv::minAreaRect(
                std::vector<cv::Point2f>({armor.corners[0], armor.corners[1],
                                          armor.corners[2], armor.corners[3]}));

            // ---- 打分：高度差小、角度差小、宽高比贴近目标值 ----
            double s = 0.0;
            s += (1.0 - hRatio) * 0.4;
            s += (1.0 - std::fabs(a.angle - b.angle) / amp_.maxAngleDiff) * 0.3;
            s += (1.0 - std::min(std::fabs(wh - amp_.targetWH), 1.0)) * 0.3;
            armor.score = s;

            cands.push_back(std::move(c));
        }
    }

    // 互最优过滤：每根灯条只允许出现在它得分最高的配对中
    std::vector<double> best(n, -1.0);
    for (const auto& c : cands) {
        best[c.i] = std::max(best[c.i], c.armor.score);
        best[c.j] = std::max(best[c.j], c.armor.score);
    }
    for (const auto& c : cands) {
        if (std::fabs(c.armor.score - best[c.i]) < 1e-9 &&
            std::fabs(c.armor.score - best[c.j]) < 1e-9) {
            armors.push_back(c.armor);
        }
    }
}

// ----------------------------------------------------------------------------
// 4. NMS：按得分降序，抑制重叠装甲板
// ----------------------------------------------------------------------------
void ArmorDetector::nms(std::vector<Armor>& armors, double iouThresh) {
    std::sort(armors.begin(), armors.end(),
              [](const Armor& a, const Armor& b) { return a.score > b.score; });

    std::vector<Armor> result;
    std::vector<bool> removed(armors.size(), false);

    for (size_t i = 0; i < armors.size(); ++i) {
        if (removed[i]) continue;
        result.push_back(armors[i]);
        for (size_t j = i + 1; j < armors.size(); ++j) {
            if (removed[j]) continue;
            // 用两个装甲板外接矩形的交集/并集近似 IoU
            cv::Rect ri = armors[i].rect.boundingRect();
            cv::Rect rj = armors[j].rect.boundingRect();
            cv::Rect inter = ri & rj;
            double interArea = inter.area();
            double unionArea = ri.area() + rj.area() - interArea;
            if (unionArea > 1e-6 && interArea / unionArea > iouThresh) removed[j] = true;
        }
    }
    armors = std::move(result);
}

// ----------------------------------------------------------------------------
// 5. 对外主接口
// ----------------------------------------------------------------------------
std::vector<Armor> ArmorDetector::detect(const cv::Mat& bgr) {
    std::vector<Armor> armors;

    cv::Mat mask = colorMask(bgr, color_);
    lastMask_ = mask;

    std::vector<LightBar> bars = findLightBars(mask);
    barCount_ = static_cast<int>(bars.size());
    if (bars.size() >= 2) {
        matchArmors(bars, armors);
        if (armors.size() > 1) nms(armors);
    }
    return armors;
}

// ----------------------------------------------------------------------------
// 6. 绘制：外框 + 角点 + 灯条中点 + 颜色标签
// ----------------------------------------------------------------------------
void ArmorDetector::drawArmors(cv::Mat& img, const std::vector<Armor>& armors) {
    for (const auto& a : armors) {
        cv::Scalar color = a.color == Color::RED
                               ? cv::Scalar(0, 0, 255)    // BGR: 红
                               : cv::Scalar(255, 120, 0); // BGR: 蓝

        // 装甲板外框（由四角生成的外接矩形，顺时针连线）
        cv::Point2f pts[4];
        a.rect.points(pts);
        for (int i = 0; i < 4; ++i)
            cv::line(img, pts[i], pts[(i + 1) % 4], color, 2, cv::LINE_AA);

        // 四个角点：0=左上 1=右上 2=右下 3=左下
        for (int i = 0; i < 4; ++i)
            cv::circle(img, a.corners[i], 4, color, -1, cv::LINE_AA);

        // 灯条中点（调试用）
        cv::circle(img, a.left.center, 3, cv::Scalar(0, 255, 255), -1);
        cv::circle(img, a.right.center, 3, cv::Scalar(0, 255, 255), -1);

        // 标签：颜色 + 宽高比 + 中心角
        std::string label = (a.color == Color::RED ? "RED" : "BLUE") +
                            std::string(" w/h=") +
                            std::to_string(a.width / a.height).substr(0, 4) +
                            std::string(" ang=") +
                            std::to_string(a.centerAngle).substr(0, 4);
        cv::putText(img, label, a.corners[0] + cv::Point2f(0, -12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
    }
}

}  // namespace armor
