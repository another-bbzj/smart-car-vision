#pragma once

#include "bird_view.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

/*
 * 逆透视查找表，移植自 HFK_STC32G144K 的 Image.c：
 *   - `reflection[120][188][2]` 把原始图像坐标映射到鸟瞰坐标；
 *   - getGodRoadLine() 逐点查表；
 *   - displayGodImage() 逐像素查表生成鸟瞰图。
 *
 * 本工程的标定表是 UndistInverseMapH/W[480][640]（float），
 * 这里在首次使用时把它们打包成连续的点表，避免每次转换重复读两个大数组，
 * 并提供逐点映射和整幅灰度图映射两种接口。
 *
 * 坐标约定与 bird::transformPoints 一致：Point.x 是列，Point.y 是行。
 * 本文件只做几何映射，不修改全局图像。
 */
namespace bird::lut {

struct InverseLut {
    int width = kInputWidth;
    int height = kInputHeight;
    std::vector<cv::Point2f> map; // row-major：map[y * width + x] -> 鸟瞰坐标
    bool valid = false;

    void build() {
        map.assign(static_cast<std::size_t>(width) * height, cv::Point2f(0.f, 0.f));
        valid = true;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const float bx = UndistInverseMapW[y][x];
                const float by = UndistInverseMapH[y][x];
                map[static_cast<std::size_t>(y) * width + x] = cv::Point2f(bx, by);
                if (!std::isfinite(bx) || !std::isfinite(by)) valid = false;
            }
        }
    }

    // 单点查表；越界或表未建立时返回 false。
    bool lookup(const cv::Point& src, cv::Point2f& dst) const {
        if (!valid || src.x < 0 || src.x >= width || src.y < 0 || src.y >= height)
            return false;
        dst = map[static_cast<std::size_t>(src.y) * width + src.x];
        return std::isfinite(dst.x) && std::isfinite(dst.y);
    }

    // 逐点映射（对应 HFK getGodRoadLine）。
    std::vector<cv::Point2f> mapPoints(const std::vector<cv::Point>& src) const {
        std::vector<cv::Point2f> out;
        out.reserve(src.size());
        for (const auto& p : src) {
            cv::Point2f q;
            if (lookup(p, q)) out.push_back(q);
        }
        return out;
    }

    // 逐像素生成鸟瞰灰度图（对应 HFK displayGodImage）。
    cv::Mat warpGray(const cv::Mat& single) const {
        if (!valid || single.empty() || single.type() != CV_8UC1)
            throw std::invalid_argument("lut::warpGray requires valid LUT and CV_8UC1 input");
        cv::Mat out(height, width, CV_8UC1, cv::Scalar(0));
        for (int y = 0; y < std::min(height, single.rows); ++y) {
            for (int x = 0; x < std::min(width, single.cols); ++x) {
                const auto& q = map[static_cast<std::size_t>(y) * width + x];
                const int bx = cvRound(q.x);
                const int by = cvRound(q.y);
                if (bx >= 0 && bx < width && by >= 0 && by < height)
                    out.at<uchar>(by, bx) = single.at<uchar>(y, x);
            }
        }
        return out;
    }
};

// 进程内只建立一次，标定表是编译期常量。
inline const InverseLut& instance() {
    static const InverseLut lut = [] {
        InverseLut value;
        value.build();
        return value;
    }();
    return lut;
}

} // namespace bird::lut
