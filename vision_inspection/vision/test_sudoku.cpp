#include "sudoku.hpp"
#include "vision_link.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); std::cout << "PASS: " << message << '\n'; }

cv::Mat makeTile(const sudoku::Grid3& cells) {
    cv::Mat img(300,300,CV_8UC3,cv::Scalar(255,255,255));
    for (int i=0;i<=3;++i) { int p=std::min(i*100,299); cv::line(img,{p,0},{p,299},cv::Scalar(0,0,0),2); cv::line(img,{0,p},{299,p},cv::Scalar(0,0,0),2); }
    for (int r=0;r<3;++r) for(int c=0;c<3;++c) if(cells[r*3+c]) {
        const std::string text(1, static_cast<char>('0'+cells[r*3+c]));
        int baseline = 0;
        const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.4, 2, &baseline);
        cv::putText(img, text, {c*100+(100-size.width)/2,r*100+(100+size.height)/2},
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0,0,0), 2, cv::LINE_AA);
    }
    return img;
}

int main() {
    try {
        sudoku::Grid6 grid = {1,2,3,4,5,6,4,5,6,1,2,3,2,3,4,5,6,1,5,6,1,2,3,4,3,4,5,6,1,2,6,1,2,3,4,5};
        auto solved = grid;
        check(sudoku::solve(solved), "valid 6x6 puzzle solver returns a solution");
        for (const auto x : solved) check(x >= 1 && x <= 6, "solution contains only digits 1..6");
        sudoku::Grid6 bad{}; bad[0]=1; bad[1]=1;
        check(!sudoku::solve(bad), "contradictory givens are rejected");
        auto fullBad=grid; fullBad[0]=fullBad[1];
        auto unchanged=fullBad;
        check(!sudoku::solve(fullBad) && fullBad==unchanged,"invalid full board rejected without mutation");
        sudoku::Grid6 empty{};
        check(!sudoku::solve(empty),"ambiguous board rejected");
        std::array<sudoku::TileObservation,4> tiles{};
        for (int t=0;t<4;++t) { tiles[t].tileIndex=t; tiles[t].recognizedAll=true; for(int i=0;i<9;++i) tiles[t].cells[i]=static_cast<std::uint8_t>((t*3+i)%6+1); }
        sudoku::Grid6 merged{};
        check(sudoku::mergeTiles(tiles, merged), "four 3x3 tiles merge into 6x6 positions");
        check(merged[0] == 1 && merged[3] == 4 && merged[18] == 1 && merged[21] == 4,
              "tile order maps upper/lower and left/right quadrants");
        const sudoku::Grid3 templateCells = {1,2,3,4,5,6,0,1,0};
        auto image = makeTile(templateCells);
        sudoku::TileObservation tile; tile.tileIndex=0; tile.rectified=image; tile.localized=true;
        const bool allRecognized = sudoku::recognizeGrid(tile);
        check(allRecognized, "default template matcher recognizes generated 1..6 digits and blanks");
        if (tile.cells != templateCells) { std::cerr << "expected:"; for (auto v : templateCells) std::cerr << ' ' << int(v); std::cerr << " actual:"; for (auto v : tile.cells) std::cerr << ' ' << int(v); std::cerr << '\n'; }
        check(tile.cells == templateCells, "template matcher returns the generated 3x3 values");
        sudoku::Config noisyConfig;
        noisyConfig.minScoreMargin = 0.01;
        sudoku::TileObservation noisy; noisy.tileIndex = 0; noisy.rectified = makeTile(templateCells); noisy.localized = true;
        for (int r=0;r<3;++r) for(int c=0;c<3;++c) if(templateCells[r*3+c]) {
            const std::string text(1, static_cast<char>('0'+templateCells[r*3+c]));
            cv::rectangle(noisy.rectified, cv::Rect(c*100+4,r*100+4,92,92), cv::Scalar(255,255,255), cv::FILLED);
            cv::putText(noisy.rectified, text, {c*100+34,r*100+72}, cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0,0,0), 3, cv::LINE_AA);
        }
        const bool noisyOk = sudoku::recognizeGrid(noisy, noisyConfig);
        if (!noisyOk) { std::cerr << "noisy:"; for (auto v : noisy.cells) std::cerr << ' ' << int(v); std::cerr << " scores:"; for (auto v : noisy.cellConfidence) std::cerr << ' ' << v; std::cerr << '\n'; }
        check(noisyOk && noisy.cells == templateCells, "template matcher tolerates small glyph scale and stroke changes");
        cv::Mat tagScene(500, 500, CV_8UC3, cv::Scalar(255,255,255));
        const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::Mat tag0, tag1;
        cv::aruco::drawMarker(dict, 0, 80, tag0, 1);
        cv::aruco::drawMarker(dict, 1, 80, tag1, 1);
        cv::cvtColor(tag0, tag0, cv::COLOR_GRAY2BGR); cv::cvtColor(tag1, tag1, cv::COLOR_GRAY2BGR);
        tag0.copyTo(tagScene(cv::Rect(40,40,80,80))); tag1.copyTo(tagScene(cv::Rect(380,380,80,80)));
        sudoku::TileObservation located;
        cv::Mat gridImage;
        cv::resize(image,gridImage,{240,240});
        gridImage.copyTo(tagScene(cv::Rect(130,130,240,240)));
        check(sudoku::locateTile(tagScene, 0, located) && located.tagTopLeft == 0 && located.tagBottomRight == 1 && !located.rectified.empty(),
              "AprilTag 36h11 pair locates and rectifies tile 0");
        check(sudoku::recognizeGrid(tile, {}, [](const cv::Mat&, int r, int c) { return static_cast<std::uint8_t>((r*3+c)%6+1); }),
              "recognizer callback fills all nine cells");
        check(tile.cells[8] == 3, "recognizer callback receives row and column");
        check(!sudoku::recognizeGrid(tile, {}, [](const cv::Mat&,int,int) { return std::uint8_t{7}; }),
              "out-of-range callback cannot become a recognized blank");
        check(!tile.recognizedAll,"invalid recognition clears previous ready state");
        const std::array<std::uint8_t,9> cells = {1,0,2,0,3,0,4,0,5};
        const auto frame = vision_link::encodeTile(1, 123, 0, 2, cells);
        check(frame.size() == 26 && frame[3] == vision_link::kTypeSudokuTile && frame[14] == 2 && frame[15] == 1,
              "TYPE=2 frame contains tile index and first cell");
        bool rejected=false; try { auto x=cells; x[0]=7; (void)vision_link::encodeTile(1,123,0,2,x); } catch (...) { rejected=true; }
        check(rejected, "TYPE=2 rejects cell values above 6");
        std::cout << "All Sudoku tests passed. AprilTag and real-font OCR still need fixture/field calibration.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
