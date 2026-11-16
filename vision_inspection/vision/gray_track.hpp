#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

/*
 * 灰度巡线算法，移植自 HFK_STC32G144K 的 Image.c：
 *   - getPointThreshold()：block×block 局部均值减 cap 的动态阈值；
 *   - getLStart()/getRStart()：从图像底部向上扫描黑白跳变，取边线内侧起点；
 *   - getLBorder()/getRBorder()：8 邻域方向状态机沿边线爬行；
 *   - findCorner()：用前后向量夹角找折点。
 *
 * 与原实现的差异：
 *   - 原图 120×188，本工程 640×480，全部尺寸由输入图像推导；
 *   - 用积分图一次算完整幅局部均值，避免逐点重复求和；
 *   - 输出 std::vector<cv::Point>，由调用者决定是否再做逆透视。
 * 本文件只做图像分析，不修改全局状态、不发送控制量。
 */
namespace gray {

struct Config {
    int block = 5;        // 局部阈值窗口边长，须为奇数且 >= 3
    int cap = 6;          // 局部均值减去的余量，越大越容易判为黑
    int maxPoints = 400;  // 单条边线最多保留点数
    int minTurn = 4;      // 连续转向次数上限，达到即认为边线中断

    void validate() const {
        if (block < 3 || block > 31 || block % 2 == 0 || cap < 0 || cap > 100 ||
            maxPoints < 8 || maxPoints > 2000 || minTurn < 1 || minTurn > 10)
            throw std::invalid_argument("Invalid gray track configuration");
    }
};

struct EdgeResult {
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    bool leftFound = false;
    bool rightFound = false;
};

// 积分图：CV_64F，尺寸 (rows+1)×(cols+1)。
inline cv::Mat integral(const cv::Mat& single) {
    cv::Mat sum;
    cv::integral(single, sum, CV_64F);
    return sum;
}

// 单点局部均值阈值，越界时钳制窗口。
inline int localThreshold(const cv::Mat& sum, int x, int y, int block, int cap) {
    const int half = block / 2;
    const int x0 = std::max(0, x - half);
    const int y0 = std::max(0, y - half);
    const int x1 = std::min(sum.cols - 2, x + half);
    const int y1 = std::min(sum.rows - 2, y + half);
    const double area = static_cast<double>((x1 - x0 + 1) * (y1 - y0 + 1));
    const double total = sum.at<double>(y1 + 1, x1 + 1) - sum.at<double>(y0, x1 + 1) -
                         sum.at<double>(y1 + 1, x0) + sum.at<double>(y0, x0);
    return static_cast<int>(total / area) - cap;
}

// 调试用：按局部阈值二值化整幅图。
inline cv::Mat binarizeLocal(const cv::Mat& g, const Config& c) {
    c.validate();
    cv::Mat sum = integral(g), out(g.size(), CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < g.rows; ++y) {
        for (int x = 0; x < g.cols; ++x) {
            if (g.at<uchar>(y, x) < localThreshold(sum, x, y, c.block, c.cap))
                out.at<uchar>(y, x) = 255;
        }
    }
    return out;
}

// 左右起点：从底部向上扫描，找到黑色边线内侧（白色赛道一侧）的起点。
// 对应 HFK getLIndexStart/getRIndexStart 的“当前点及向内 8/25 像素都黑”判据，
// 返回值向内偏移 26 像素，保证爬线器从白区贴边开始而不是钻进黑带内部。
inline bool findStart(const cv::Mat& g, const cv::Mat& sum, const Config& c,
                      bool leftSide, int& sx, int& sy) {
    const int w = g.cols, h = g.rows;
    const int yStart = h - c.block - 2;
    const int yStop = h / 2;
    const int margin = c.block + 1;
    for (int y = yStart; y >= yStop; y -= 3) {
        int last = -1;
        if (leftSide) {
            for (int x = margin; x + 26 < w / 2; ++x) {
                const int t = localThreshold(sum, x, y, c.block, c.cap);
                if (g.at<uchar>(y, x) < t &&
                    g.at<uchar>(y, x + 8) < t &&
                    g.at<uchar>(y, x + 25) < t) {
                    last = x;
                } else if (last >= 0) {
                    sx = last + 26; sy = y; return true;
                }
            }
        } else {
            for (int x = w - margin - 1; x - 26 >= w / 2; --x) {
                const int t = localThreshold(sum, x, y, c.block, c.cap);
                if (g.at<uchar>(y, x) < t &&
                    g.at<uchar>(y, x - 8) < t &&
                    g.at<uchar>(y, x - 25) < t) {
                    last = x;
                } else if (last >= 0) {
                    sx = last - 26; sy = y; return true;
                }
            }
        }
    }
    return false;
}

// 8 邻域方向状态机，移植自 HFK getLBorder/getRBorder。
// 内部用 (row, col) 方便查图，输出统一为 W 视觉约定 cv::Point(col, row)。
inline std::vector<cv::Point> crawl(const cv::Mat& g, const cv::Mat& sum,
                                    const Config& c, cv::Point start, bool leftSide) {
    static const int fx[4][2] = {{-2, 0}, {0, 2}, {2, 0}, {0, -2}}; // (row, col)
    static const int lx[4][2] = {{-2, -2}, {-2, 2}, {2, 2}, {2, -2}};
    static const int rx[4][2] = {{-2, 2}, {2, 2}, {2, -2}, {-2, -2}};
    const int (*diag)[2] = leftSide ? lx : rx;

    std::vector<cv::Point> out;
    out.reserve(static_cast<std::size_t>(c.maxPoints));
    int row = start.y, col = start.x, dir = 0, turn = 0;
    const int w = g.cols, h = g.rows;

    while (row > c.block + 1 && row < h - c.block - 1 &&
           col > c.block + 1 && col < w - c.block - 1 &&
           static_cast<int>(out.size()) < c.maxPoints && turn < c.minTurn) {
        const int t = localThreshold(sum, col, row, c.block, c.cap);
        const int fr = row + fx[dir][0], fc = col + fx[dir][1];
        const int dr = row + diag[dir][0], dc = col + diag[dir][1];
        if (fr < 0 || fr >= h || fc < 0 || fc >= w ||
            dr < 0 || dr >= h || dc < 0 || dc >= w) break;

        if (g.at<uchar>(fr, fc) < t) {
            // 正前方更暗（墙）：原地转向，连续转满 minTurn 视为边线中断。
            dir = leftSide ? (dir + 1) % 4 : (dir + 3) % 4;
            ++turn;
        } else if (g.at<uchar>(dr, dc) < t) {
            // 正前方可走、斜前方是墙：直走，贴着边线。
            row = fr;
            col = fc;
            out.emplace_back(col, row);
            turn = 0;
        } else {
            // 斜前方也可走：斜走一步并调整方向。
            row = dr;
            col = dc;
            dir = leftSide ? (dir + 3) % 4 : (dir + 1) % 4;
            out.emplace_back(col, row);
            turn = 0;
        }
    }
    return out;
}

// 左右边线跟踪入口。
inline EdgeResult track(const cv::Mat& single) {
    if (single.empty() || single.type() != CV_8UC1)
        throw std::invalid_argument("gray::track requires CV_8UC1 input");
    Config c;
    c.validate();
    const cv::Mat sum = integral(single);
    EdgeResult out;
    int x = 0, y = 0;
    if (findStart(single, sum, c, true, x, y)) {
        out.leftFound = true;
        out.left = crawl(single, sum, c, cv::Point(x, y), true);
    }
    if (findStart(single, sum, c, false, x, y)) {
        out.rightFound = true;
        out.right = crawl(single, sum, c, cv::Point(x, y), false);
    }
    return out;
}

// 角点检测：前后向量夹角明显，取夹角最小者。
inline int findCorner(const std::vector<cv::Point>& line, int span = 5) {
    if (line.size() < static_cast<std::size_t>(2 * span + 3) || span < 1) return -1;
    int best = -1;
    double bestCos = 1.0;
    for (int i = span; i + span < static_cast<int>(line.size()); ++i) {
        const cv::Point a = line[i] - line[i - span];
        const cv::Point b = line[i + span] - line[i];
        const double la = std::hypot(a.x, a.y), lb = std::hypot(b.x, b.y);
        if (la < 1e-6 || lb < 1e-6) continue;
        const double cosv = (a.x * b.x + a.y * b.y) / (la * lb);
        if (cosv < bestCos) {
            bestCos = cosv;
            best = i;
        }
    }
    // cos(60°) = 0.5，比 HFK 的 0.5 门限保持一致。
    return bestCos <= 0.5 ? best : -1;
}

} // namespace gray
