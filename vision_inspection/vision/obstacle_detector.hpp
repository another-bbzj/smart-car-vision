#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace obstacle {

// 路障颜色类别。Unknown 表示候选形状通过筛选，但无法归入三种目标颜色。
enum class Color { Unknown, Green, Yellow, Red };

// 检测参数集中在这里，便于按相机和现场灯光调节，不需要改算法代码。
struct Config {
    // HSV 色相 H 的范围是 0..179；以下颜色阈值都是初值，尚未用实物校准。
    int redLowMaxH = 10;
    int redHighMinH = 170;
    int yellowMinH = 15;
    int yellowMaxH = 37;
    int greenMinH = 38;
    int greenMaxH = 90;
    int minS = 80;
    int minV = 60;
    int morphKernel = 3;
    int minAreaPx = 25;
    int maxAreaPx = 20000;
    double minCircularity = 0.30;
    double minExtent = 0.25;
    double maxAspectRatio = 2.8; // allow perspective ellipses, reject long strips
    double maxCenterXRatio = 0.45; // reserve a margin for frame-edge false positives
    double minCenterYRatio = 0.08;
    int confirmationFrames = 3;
    int maxMissedFrames = 5;
    double associationDistancePx = 45;

    // 检查阈值范围和形态学参数是否合法，避免错误配置导致 OpenCV 异常或结果失真。
    bool valid() const {
        return redLowMaxH >= 0 && redLowMaxH < 30 && redHighMinH > 150 && redHighMinH <= 179 &&
            yellowMinH >= 0 && yellowMaxH <= 179 && yellowMinH < yellowMaxH &&
            greenMinH >= 0 && greenMaxH <= 179 && greenMinH < greenMaxH && yellowMaxH < greenMinH &&
            minS >= 0 && minS <= 255 && minV >= 0 && minV <= 255 &&
            morphKernel >= 1 && morphKernel <= 11 && morphKernel % 2 == 1 &&
            minAreaPx > 0 && maxAreaPx >= minAreaPx && maxAreaPx <= 1000000 &&
            std::isfinite(minCircularity) && minCircularity > 0 && minCircularity <= 1 &&
            std::isfinite(minExtent) && minExtent > 0 && minExtent <= 1 &&
            std::isfinite(maxAspectRatio) && maxAspectRatio >= 1 && maxAspectRatio <= 10 &&
            std::isfinite(maxCenterXRatio) && maxCenterXRatio >= 0.1 && maxCenterXRatio < 0.5 &&
            std::isfinite(minCenterYRatio) && minCenterYRatio >= 0 && minCenterYRatio < 1 &&
            confirmationFrames > 0 && confirmationFrames <= 30 && maxMissedFrames >= 0 && maxMissedFrames <= 60 &&
            std::isfinite(associationDistancePx) && associationDistancePx > 0;
    }
};

// 单个路障候选的测量结果。坐标和面积都是图像像素单位，不是车辆坐标或米。
struct Detection {
    Color color = Color::Unknown; // 识别出的颜色类别。
    cv::Point2f center;            // 轮廓质心，原图像素坐标；原点在左上角。
    cv::Rect bbox;                 // 候选轮廓的外接矩形，单位为像素。
    double areaPx = 0;             // 轮廓面积，单位为像素平方。
    double circularity = 0;        // 圆形度，越接近 1 越像圆；受透视和轮廓噪声影响。
    double extent = 0;             // 轮廓面积 / 外接框面积，用于排除稀疏或细长形状。
    double colorPurity = 0;        // 轮廓区域中命中颜色掩膜的比例。
    double confidence = 0;         // 由形状、颜色和大小特征加权得到的启发式分数，不是概率。
    bool stable = false;           // 是否已连续多帧关联确认。
    int hits = 0;                  // 当前轨迹连续匹配到的帧数。
    int missed = 0;                // 当前轨迹连续未匹配的帧数。
};

// 单帧的路障检测汇总：候选列表、当前主目标、三种颜色掩膜和可视化叠加图。
struct Observation {
    std::vector<Detection> candidates; // 本帧当前可见且通过形状筛选的候选。
    Detection primary;                 // 画面中最靠近车辆的候选；按图像纵坐标近似排序。
    bool hasPrimary = false;           // primary 是否有效。
    cv::Mat redMask, yellowMask, greenMask, overlay; // 调试用掩膜和标注图。
};

// 将颜色枚举转换成日志使用的稳定英文短名。
inline const char* colorName(Color color) {
    switch (color) {
    case Color::Green: return "GREEN";
    case Color::Yellow: return "YELLOW";
    case Color::Red: return "RED";
    default: return "UNKNOWN";
    }
}

// 清理颜色分割产生的孤立噪点和小缺口，返回新掩膜，不修改传入图像。
inline cv::Mat cleanMask(const cv::Mat& source, int kernel) {
    if (kernel <= 1) return source.clone();
    cv::Mat result;
    const auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel, kernel));
    cv::morphologyEx(source, result, cv::MORPH_OPEN, element);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, element);
    return result;
}

// 从一种颜色的二值掩膜中提取轮廓，依次按大小、圆形度、长宽比和位置过滤。
// 合格候选会追加到 output；本函数不负责跨帧确认。
inline void addColorCandidates(const cv::Mat& hsv, const cv::Mat& mask, Color color,
                               const Config& c, std::vector<Detection>& output) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < c.minAreaPx || area > c.maxAreaPx || contour.size() < 5) continue;
        const cv::Rect box = cv::boundingRect(contour);
        if (box.width <= 0 || box.height <= 0) continue;
        const double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 1e-6) continue;
        const double circularity = std::clamp(4.0 * CV_PI * area / (perimeter * perimeter), 0.0, 1.0);
        const double extent = area / static_cast<double>(box.area());
        const double aspect = static_cast<double>(std::max(box.width, box.height)) /
            static_cast<double>(std::min(box.width, box.height));
        if (circularity < c.minCircularity || extent < c.minExtent || aspect > c.maxAspectRatio) continue;
        const cv::Moments moments = cv::moments(contour);
        if (std::abs(moments.m00) <= 1e-6) continue;
        const cv::Point2f center(static_cast<float>(moments.m10 / moments.m00),
                                 static_cast<float>(moments.m01 / moments.m00));
        if (std::abs(center.x / hsv.cols - 0.5) > c.maxCenterXRatio ||
            center.y / hsv.rows < c.minCenterYRatio) continue;
        cv::Mat contourMask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(contourMask, std::vector<std::vector<cv::Point>>{contour}, 0, 255, cv::FILLED);
        const double purity = cv::mean(mask, contourMask)[0] / 255.0;
        const double sizeScore = std::clamp(std::sqrt(area / 500.0), 0.0, 1.0);
        Detection d;
        d.color = color; d.center = center; d.bbox = box; d.areaPx = area;
        d.circularity = circularity; d.extent = extent; d.colorPurity = purity;
        d.confidence = std::clamp(0.35 * purity + 0.35 * circularity + 0.20 * extent + 0.10 * sizeScore, 0.0, 1.0);
        output.push_back(d);
    }
}

// 单帧检测入口：BGR 转 HSV，生成红/黄/绿掩膜，提取形状候选并画调试框。
// 返回的坐标仍是相机像素；本函数不估算实际距离，也不决定绕行/停车。
inline Observation detect(const cv::Mat& bgr, const Config& c = {}) {
    if (!c.valid()) throw std::invalid_argument("Invalid obstacle detector configuration");
    Observation out;
    if (bgr.empty()) return out;
    if (bgr.type() != CV_8UC3) throw std::invalid_argument("Obstacle detector requires CV_8UC3 BGR input");
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat redLow, redHigh;
    cv::inRange(hsv, cv::Scalar(0, c.minS, c.minV), cv::Scalar(c.redLowMaxH, 255, 255), redLow);
    cv::inRange(hsv, cv::Scalar(c.redHighMinH, c.minS, c.minV), cv::Scalar(179, 255, 255), redHigh);
    cv::bitwise_or(redLow, redHigh, out.redMask);
    cv::inRange(hsv, cv::Scalar(c.yellowMinH, c.minS, c.minV), cv::Scalar(c.yellowMaxH, 255, 255), out.yellowMask);
    cv::inRange(hsv, cv::Scalar(c.greenMinH, c.minS, c.minV), cv::Scalar(c.greenMaxH, 255, 255), out.greenMask);
    out.redMask = cleanMask(out.redMask, c.morphKernel);
    out.yellowMask = cleanMask(out.yellowMask, c.morphKernel);
    out.greenMask = cleanMask(out.greenMask, c.morphKernel);
    addColorCandidates(hsv, out.greenMask, Color::Green, c, out.candidates);
    addColorCandidates(hsv, out.yellowMask, Color::Yellow, c, out.candidates);
    addColorCandidates(hsv, out.redMask, Color::Red, c, out.candidates);
    if (!out.candidates.empty()) {
        out.primary = *std::max_element(out.candidates.begin(), out.candidates.end(), [](const Detection& a, const Detection& b) {
            if (a.center.y != b.center.y) return a.center.y < b.center.y; // nearest image target first
            return a.confidence < b.confidence;
        });
        out.hasPrimary = true;
    }
    out.overlay = bgr.clone();
    for (const auto& d : out.candidates) {
        const cv::Scalar color = d.color == Color::Red ? cv::Scalar(0, 0, 255) :
            d.color == Color::Yellow ? cv::Scalar(0, 220, 220) : cv::Scalar(0, 220, 0);
        cv::rectangle(out.overlay, d.bbox, color, 2);
        cv::putText(out.overlay, std::string(colorName(d.color)) + cv::format(" %.2f", d.confidence),
                    d.bbox.tl() + cv::Point(0, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    return out;
}

// 跨帧稳定器：把相邻帧中颜色相同、位置接近的候选关联成同一个目标。
// 它用于去掉单帧误检；目标是否绕行或停车由任务/控制层另行决定。
class Stabilizer {
public:
    // 使用同一套面积、颜色和关联距离参数构造稳定器。
    explicit Stabilizer(Config config = {}) : config_(config) {}

    // 新一段视频或任务开始时清除旧轨迹，防止上一段目标残留。
    void reset() {
        tracks_.clear(); // 删除所有历史轨迹，下一帧从新目标开始累计命中数。
    }

    // 更新本帧检测结果：增加命中/漏检计数、标记稳定目标并移除过期轨迹。
    // 返回列表只包含当前帧仍能看见的目标，不会把短暂消失的目标伪装成可见。
    Observation update(Observation observation) {
        if (!config_.valid()) throw std::invalid_argument("Invalid obstacle stabilizer configuration");
        for (auto& track : tracks_) track.missed++;
        for (auto& candidate : observation.candidates) {
            auto found = std::find_if(tracks_.begin(), tracks_.end(), [&](const Detection& t) {
                return t.color == candidate.color && cv::norm(t.center - candidate.center) <= config_.associationDistancePx;
            });
            if (found == tracks_.end()) {
                // 没找到附近同色旧轨迹：将其作为新目标，从第 1 次命中开始。
                candidate.hits = 1;
                candidate.missed = 0;
                tracks_.push_back(candidate);
            } else {
                // 找到同一目标：延续命中计数，并在达到帧数门槛后标记稳定。
                candidate.hits = found->hits + 1;
                candidate.missed = 0;
                candidate.stable = candidate.hits >= config_.confirmationFrames;
                *found = candidate;
            }
        }
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Detection& t) {
            return t.missed > config_.maxMissedFrames;
        }), tracks_.end());
        observation.candidates.clear();
        for (const auto& track : tracks_) if (track.missed == 0) observation.candidates.push_back(track);
        if (!observation.candidates.empty()) {
            observation.primary = *std::max_element(observation.candidates.begin(), observation.candidates.end(), [](const Detection& a, const Detection& b) {
                if (a.center.y != b.center.y) return a.center.y < b.center.y;
                return a.confidence < b.confidence;
            });
            observation.hasPrimary = true;
        }
        return observation;
    }
private:
    Config config_;                  // 当前筛选和关联参数。
    std::vector<Detection> tracks_;  // 最近几帧仍有效的目标轨迹。
};

} // namespace obstacle
