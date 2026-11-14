#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cstdint>

namespace sudoku {

using Grid3 = std::array<std::uint8_t, 9>;
using Grid6 = std::array<std::uint8_t, 36>;

struct Config {
    int outputSize = 300;
    int cellMargin = 8;
    double minTagPerimeter = 20;
    int threshold = 150;
    bool adaptiveThreshold = true;
    double blankInkRatio = 0.018;
    double minTemplateScore = 0.42;
    double minScoreMargin = 0.035;
    int templateFont = cv::FONT_HERSHEY_SIMPLEX;
    double templateScale = 1.4;
    int templateThickness = 2;
    bool valid() const {
        return outputSize >= 90 && cellMargin >= 0 && cellMargin < outputSize / 6 && minTagPerimeter > 0 &&
            blankInkRatio >= 0 && blankInkRatio < 0.5 && minTemplateScore >= 0 && minTemplateScore <= 1 &&
            minScoreMargin >= 0 && minScoreMargin <= 1 && templateScale > 0 && templateThickness > 0;
    }
};

struct TileObservation {
    int tileIndex = -1;
    int tagTopLeft = -1;
    int tagBottomRight = -1;
    std::array<cv::Point2f, 4> corners{};
    cv::Mat rectified;
    Grid3 cells{};
    std::array<bool, 9> recognized{}; // false 表示低置信度/未知，不能当作空格发送。
    std::array<double, 9> cellConfidence{};
    double confidence = 0;
    bool localized = false;
    bool recognizedAll = false;
};

inline int tileIndexForTags(int a, int b) {
    if (a == 0 && b == 1) return 0;
    if (a == 2 && b == 3) return 1;
    if (a == 4 && b == 5) return 2;
    if (a == 6 && b == 7) return 3;
    return -1;
}

inline cv::Mat tileMask(const cv::Mat& gray, const Config& c) {
    cv::Mat binary;
    if (c.adaptiveThreshold) cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                                                     cv::THRESH_BINARY_INV, 31, 8);
    else cv::threshold(gray, binary, c.threshold, 255, cv::THRESH_BINARY_INV);
    return binary;
}

// Tags identify and orient the grid; warp its border, excluding external tags.
inline std::vector<TileObservation> locateTiles(const cv::Mat& bgr, const Config& c = {}) {
    std::vector<TileObservation> tiles;
    if (!c.valid() || bgr.empty() || bgr.type() != CV_8UC3) return tiles;
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bgr, dictionary, corners, ids);
    if (ids.size() < 2) return tiles;
    cv::Mat gray, ink;
    cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
    cv::threshold(gray,ink,0,255,cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
    std::vector<std::vector<cv::Point>> borders;
    cv::findContours(ink,borders,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    for (int index=0; index<4; ++index) {
        if (std::count(ids.begin(),ids.end(),2*index) != 1 ||
            std::count(ids.begin(),ids.end(),2*index+1) != 1) continue;
        auto a = std::find(ids.begin(),ids.end(),2*index)-ids.begin();
        auto b = std::find(ids.begin(),ids.end(),2*index+1)-ids.begin();
        if (cv::arcLength(corners[a],true) < c.minTagPerimeter ||
            cv::arcLength(corners[b],true) < c.minTagPerimeter) continue;
        auto center=[](const auto& points) {
            return (points[0]+points[1]+points[2]+points[3])*0.25f;
        };
        auto tl=center(corners[a]), br=center(corners[b]);
        double best=cv::norm(tl-br)*0.35;
        std::vector<cv::Point2f> grid;
        for (const auto& border:borders) {
            if (std::abs(cv::contourArea(border)) < 1600) continue;
            std::vector<cv::Point> q;
            cv::approxPolyDP(border,q,0.025*cv::arcLength(border,true),true);
            if (q.size()!=4 || !cv::isContourConvex(q)) continue;
            if (cv::contourArea(q,true)<0) std::reverse(q.begin(),q.end());
            for (unsigned j=0;j<4;++j) {
                double error=cv::norm(tl-cv::Point2f(q[j]))+cv::norm(br-cv::Point2f(q[(j+2)%4]));
                if (error >= best) continue;
                best=error;
                grid.clear();
                for (unsigned k=0;k<4;++k) grid.emplace_back(q[(j+k)%4]);
            }
        }
        if (grid.empty()) continue;
        TileObservation tile;
        tile.tileIndex=index;
        tile.tagTopLeft=2*index;
        tile.tagBottomRight=2*index+1;
        std::copy(grid.begin(),grid.end(),tile.corners.begin());
        const float end=static_cast<float>(c.outputSize-1);
        std::vector<cv::Point2f> dst={{0,0},{end,0},{end,end},{0,end}};
        cv::warpPerspective(bgr,tile.rectified,cv::getPerspectiveTransform(grid,dst),
                            cv::Size(c.outputSize,c.outputSize));
        tile.localized=true;
        tiles.push_back(std::move(tile));
    }
    return tiles;
}

inline bool locateTile(const cv::Mat& bgr, int tileIndex, TileObservation& out, const Config& c = {}) {
    out = {};
    for (auto& tile:locateTiles(bgr,c))
        if (tile.tileIndex==tileIndex) { out=std::move(tile); return true; }
    return false;
}

// 将格子转成固定大小的前景掩膜，并去掉周围空白，使数字大小差异较小。
inline cv::Mat normalizeGlyph(const cv::Mat& cell, const Config& c) {
    cv::Mat gray, binary;
    if (cell.channels() == 3) cv::cvtColor(cell, gray, cv::COLOR_BGR2GRAY);
    else gray = cell.clone();
    if (c.adaptiveThreshold) cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                                                    cv::THRESH_BINARY_INV, 31, 8);
    else cv::threshold(gray, binary, c.threshold, 255, cv::THRESH_BINARY_INV);
    std::vector<cv::Point> points;
    cv::findNonZero(binary, points);
    if (points.empty()) return cv::Mat::zeros(32, 24, CV_8UC1);
    const cv::Rect box = cv::boundingRect(points);
    cv::Mat glyph = binary(box).clone();
    const int targetW = 20, targetH = 28;
    const double scale = std::min(targetW / static_cast<double>(glyph.cols), targetH / static_cast<double>(glyph.rows));
    cv::Mat resized;
    cv::resize(glyph, resized, cv::Size(std::max(1, cvRound(glyph.cols * scale)), std::max(1, cvRound(glyph.rows * scale))), 0, 0, cv::INTER_AREA);
    cv::threshold(resized,resized,100,255,cv::THRESH_BINARY);
    cv::Mat canvas = cv::Mat::zeros(32, 24, CV_8UC1);
    resized.copyTo(canvas(cv::Rect((24-resized.cols)/2, (32-resized.rows)/2, resized.cols, resized.rows)));
    return canvas;
}

// 为 1 到 6 分别生成归一化字形模板。可以在配置中更改字体、字号和笔画。
inline std::array<std::vector<cv::Mat>, 7> makeDigitTemplates(const Config& c = {}) {
    std::array<std::vector<cv::Mat>, 7> templates;
    for (int digit = 0; digit <= 6; ++digit) templates[digit] = {};
    for (int digit = 1; digit <= 6; ++digit) {
        for (const double scale : {c.templateScale * 0.9, c.templateScale, c.templateScale * 1.1}) {
            for (const int thickness : {std::max(1, c.templateThickness - 1), c.templateThickness, c.templateThickness + 1}) {
                cv::Mat canvas(64, 48, CV_8UC1, cv::Scalar(255));
                const std::string text(1, static_cast<char>('0' + digit));
                int baseline = 0;
                const cv::Size size = cv::getTextSize(text, c.templateFont, scale, thickness, &baseline);
                cv::putText(canvas, text, cv::Point((canvas.cols-size.width)/2, (canvas.rows+size.height)/2),
                            c.templateFont, scale, cv::Scalar(0), thickness, cv::LINE_AA);
                templates[digit].push_back(normalizeGlyph(canvas, c));
            }
        }
    }
    return templates;
}

// 对一个格子做空白判断，再与 1..6 模板匹配；低分或候选差异太小时返回 0/低置信度。
inline std::uint8_t recognizeCell(const cv::Mat& bgrCell, const Config& c,
                                  const std::array<std::vector<cv::Mat>,7>& templates, double& confidence) {
    confidence = 0;
    const cv::Mat glyph = normalizeGlyph(bgrCell, c);
    const double inkRatio = cv::countNonZero(glyph) / static_cast<double>(glyph.total());
    if (inkRatio < c.blankInkRatio) { confidence = 1.0; return 0; }
    std::array<std::pair<double,int>,6> scores{};
    for (int digit = 1; digit <= 6; ++digit) {
        double best = -1;
        for (const auto& templ : templates[digit]) {
            cv::Mat diff;
            cv::absdiff(glyph, templ, diff);
            best = std::max(best, 1.0 - cv::countNonZero(diff) / static_cast<double>(glyph.total()));
        }
        scores[digit-1] = {best, digit};
    }
    std::sort(scores.begin(), scores.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    confidence = std::clamp(scores[0].first, 0.0, 1.0);
    if (scores[0].first < c.minTemplateScore || scores[0].first - scores[1].first < c.minScoreMargin) return 255;
    return static_cast<std::uint8_t>(scores[0].second);
}

inline cv::Mat cropCell(const cv::Mat& rectified, int row, int col, const Config& c) {
    const int size = rectified.rows / 3;
    const int m = std::min(c.cellMargin, size / 4);
    return rectified(cv::Rect(col * size + m, row * size + m, size - 2*m, size - 2*m)).clone();
}

// 用默认模板库识别单格；0 表示空格或低置信度，需查看 confidence 区分。
inline std::uint8_t recognizeCell(const cv::Mat& bgrCell, const Config& c = {}) {
    const auto templates = makeDigitTemplates(c);
    double confidence = 0;
    return recognizeCell(bgrCell, c, templates, confidence);
}

inline bool recognizeGrid(TileObservation& tile, const Config& c = {},
                          const std::function<std::uint8_t(const cv::Mat&, int, int)>& recognizer = {},
                          const std::array<std::vector<cv::Mat>,7>* cachedTemplates = nullptr) {
    tile.recognizedAll=false;
    tile.recognized.fill(false);
    if (!c.valid() || !tile.localized || tile.rectified.empty() ||
        tile.rectified.rows != tile.rectified.cols || tile.rectified.rows < 90) return false;
    int good = 0;
    const auto generated = cachedTemplates ? std::array<std::vector<cv::Mat>,7>{} : makeDigitTemplates(c);
    const auto& templates = cachedTemplates ? *cachedTemplates : generated;
    for (int r = 0; r < 3; ++r) for (int col = 0; col < 3; ++col) {
        const int index = r * 3 + col;
        const auto cell = cropCell(tile.rectified, r, col, c);
        double score = 0;
        const auto value = recognizer ? recognizer(cell, r, col) : recognizeCell(cell, c, templates, score);
        tile.cells[index] = value <= 6 ? value : 0;
        tile.recognized[index] = value <= 6;
        tile.cellConfidence[index] = recognizer ? (value <= 6 ? 1.0 : 0.0) : score;
        good += tile.recognized[index];
    }
    tile.confidence = good / 9.0;
    tile.recognizedAll = good == 9;
    return tile.recognizedAll;
}

inline bool mergeTiles(const std::array<TileObservation,4>& tiles, Grid6& full) {
    full.fill(0);
    for (int tile = 0; tile < 4; ++tile) {
        if (tiles[tile].tileIndex != tile || !tiles[tile].recognizedAll) return false;
        if (std::any_of(tiles[tile].cells.begin(),tiles[tile].cells.end(),[](auto value) { return value>6; })) return false;
        const int baseRow = tile >= 2 ? 3 : 0;
        const int baseCol = tile % 2 == 1 ? 3 : 0;
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) full[(baseRow+r)*6 + baseCol+c] = tiles[tile].cells[r*3+c];
    }
    return true;
}

inline bool validPlacement(const Grid6& g, int pos, int value) {
    const int r = pos / 6, c = pos % 6;
    for (int i = 0; i < 6; ++i) if (g[r*6+i] == value || g[i*6+c] == value) return false;
    const int br = (r / 2) * 2, bc = (c / 3) * 3;
    for (int y = br; y < br+2; ++y) for (int x = bc; x < bc+3; ++x) if (g[y*6+x] == value) return false;
    return true;
}

inline void solveSearch(Grid6& grid, Grid6& answer, unsigned& solutions, unsigned& nodes) {
    if (++nodes > 20000 || solutions >= 2) return;
    int best = -1, choices = 7;
    for (int p = 0; p < 36; ++p) if (grid[p] == 0) {
        int count = 0; for (int v = 1; v <= 6; ++v) if (validPlacement(grid, p, v)) ++count;
        if (count < choices) { choices = count; best = p; }
    }
    if (best < 0) { if (!solutions) answer=grid; ++solutions; return; }
    for (int v = 1; v <= 6 && solutions < 2 && nodes <= 20000; ++v)
        if (validPlacement(grid, best, v)) {
            grid[best] = static_cast<std::uint8_t>(v);
            solveSearch(grid,answer,solutions,nodes);
            grid[best] = 0;
        }
}

inline bool solve(Grid6& grid) {
    Grid6 work=grid, answer{};
    for (int i=0;i<36;++i) {
        auto value=work[i];
        if (value>6) return false;
        if (!value) continue;
        work[i]=0;
        if (!validPlacement(work,i,value)) return false;
        work[i]=value;
    }
    unsigned solutions=0, nodes=0;
    solveSearch(work,answer,solutions,nodes);
    if (solutions != 1 || nodes > 20000) return false;
    grid=answer;
    return true;
}

} // namespace sudoku
