#include "gray_track.hpp"
#include "inverse_lut.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

/* 灰度巡线与逆透视 LUT 的离线回归测试：不依赖摄像头和串口。 */

static void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}

int main() {
    try {
        // 白底 + 两条宽黑带，模拟左右边线。
        cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));
        cv::rectangle(image, cv::Rect(80, 0, 50, 480), cv::Scalar(0), cv::FILLED);
        cv::rectangle(image, cv::Rect(510, 0, 50, 480), cv::Scalar(0), cv::FILLED);

        const auto edges = gray::track(image);
        check(edges.leftFound && edges.rightFound, "gray track finds both starts");
        check(!edges.left.empty() && !edges.right.empty(), "gray crawler returns edge points");
        check(edges.left.size() <= 400 && edges.right.size() <= 400, "gray crawler respects max points");

        // 动态阈值二值化：黑带边缘被判为暗，白底保持亮。
        gray::Config cfg;
        const auto mask = gray::binarizeLocal(image, cfg);
        check(mask.at<uchar>(240, 129) == 255 && mask.at<uchar>(240, 300) == 0,
              "dynamic local threshold marks the black/white edge");

        // 直角折线：前 50 点向下，后 50 点向右，折点应被识别。
        std::vector<cv::Point> corner;
        for (int i = 0; i < 50; ++i) corner.emplace_back(100 + i, 100);
        for (int i = 1; i < 50; ++i) corner.emplace_back(149, 100 + i);
        check(gray::findCorner(corner) >= 0, "corner detector reports the bend");

        // 逆透视 LUT：尺寸与标定表一致，能映射有效点和整幅图。
        const auto& lut = bird::lut::instance();
        check(lut.valid && lut.width == 640 && lut.height == 480, "inverse LUT built from calibration");
        cv::Point2f mapped;
        check(lut.lookup(cv::Point(400, 320), mapped), "inverse LUT maps a valid source pixel");
        const std::vector<cv::Point> src{{400, 320}, {-1, 0}, {640, 0}};
        check(lut.mapPoints(src).size() == 1, "inverse LUT drops out-of-range source points");
        const auto bird = lut.warpGray(image);
        check(bird.cols == 640 && bird.rows == 480, "inverse LUT warps a full grayscale image");

        std::cout << "All gray track / inverse LUT tests passed.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
