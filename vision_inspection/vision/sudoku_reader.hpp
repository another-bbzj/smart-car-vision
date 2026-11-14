#pragma once
#include "sudoku.hpp"
#include <optional>

namespace sudoku {
using Cells = Grid3;
struct Tile { uint8_t index; Cells cells; };

// Confirm independently by tile ID. Repeat confirmed observations with fresh
// sequence numbers, so revisiting a tile repairs lost packets in this session.
class Reader {
    Config config_;
    std::array<std::vector<cv::Mat>,7> templates_;
    std::array<Cells,4> previous_{};
    std::array<unsigned,4> hits_{};
public:
    explicit Reader(const Config& config = {}) : config_(config) {
        if (!config.valid()) throw std::invalid_argument("Invalid Sudoku config");
        templates_=makeDigitTemplates(config_);
    }

    std::vector<Tile> readAll(const cv::Mat& bgr) {
        std::vector<Tile> confirmed;
        std::array<bool,4> seen{};
        for (auto& observation:locateTiles(bgr,config_)) {
            if (!recognizeGrid(observation,config_,{},&templates_)) continue;
            auto index=static_cast<uint8_t>(observation.tileIndex);
            seen[index]=true;
            hits_[index]=previous_[index]==observation.cells ? std::min(3U,hits_[index]+1U) : 1U;
            previous_[index]=observation.cells;
            if (hits_[index]>=2) confirmed.push_back({index,observation.cells});
        }
        for (unsigned i=0;i<4;++i) if (!seen[i]) hits_[i]=0;
        return confirmed;
    }

    std::optional<Tile> read(const cv::Mat& bgr) {
        auto tiles=readAll(bgr);
        if (tiles.empty()) return std::nullopt;
        return tiles.front();
    }
};
}
