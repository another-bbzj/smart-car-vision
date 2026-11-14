#include "obstacle_detector.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}

cv::Mat scene() {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(180, 50, 20));
    cv::rectangle(image, cv::Rect(80, 0, 480, 480), cv::Scalar(220, 220, 220), cv::FILLED);
    return image;
}

obstacle::Detection findColor(const obstacle::Observation& o, obstacle::Color color) {
    for (const auto& d : o.candidates) if (d.color == color) return d;
    return {};
}

int main() {
    try {
        obstacle::Config c;
        check(c.valid(), "default obstacle configuration is valid");
        auto image = scene();
        cv::circle(image, {150, 350}, 24, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::circle(image, {320, 300}, 24, cv::Scalar(0, 220, 220), cv::FILLED);
        cv::circle(image, {490, 250}, 24, cv::Scalar(0, 220, 0), cv::FILLED);
        auto o = obstacle::detect(image, c);
        check(o.candidates.size() == 3, "red yellow green circles are detected separately");
        check(findColor(o, obstacle::Color::Red).center.y > findColor(o, obstacle::Color::Green).center.y,
              "nearest target is lower in the image");
        check(findColor(o, obstacle::Color::Red).confidence > 0.5,
              "round red target receives useful confidence");

        auto blank = scene();
        check(obstacle::detect(blank, c).candidates.empty(), "white road and blue background produce no obstacle");
        auto line = scene(); cv::rectangle(line, {300, 200}, {520, 230}, cv::Scalar(0, 220, 220), cv::FILLED);
        check(obstacle::detect(line, c).candidates.empty(), "long non-round color blob is rejected");
        auto edge = scene(); cv::circle(edge, {15, 300}, 24, cv::Scalar(0, 0, 255), cv::FILLED);
        check(obstacle::detect(edge, c).candidates.empty(), "frame-edge target is rejected");
        auto small = scene(); cv::circle(small, {320, 300}, 2, cv::Scalar(0, 220, 0), cv::FILLED);
        check(obstacle::detect(small, c).candidates.empty(), "tiny color noise is rejected");
        bool rejected = false;
        try { auto bad = c; bad.morphKernel = 4; (void)obstacle::detect(image, bad); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid detector configuration is rejected");

        obstacle::Stabilizer stabilizer(c);
        auto one = stabilizer.update(obstacle::detect(scene(), c));
        check(!one.hasPrimary, "stabilizer does not invent a target");
        auto targetFrame = scene(); cv::circle(targetFrame, {320, 350}, 24, cv::Scalar(0, 220, 220), cv::FILLED);
        auto first = stabilizer.update(obstacle::detect(targetFrame, c));
        check(first.hasPrimary && !first.primary.stable, "first candidate is not stable yet");
        auto second = stabilizer.update(obstacle::detect(targetFrame, c));
        auto third = stabilizer.update(obstacle::detect(targetFrame, c));
        check(third.hasPrimary && third.primary.stable && third.primary.hits >= 3,
              "three associated frames stabilize one target");
        auto miss = stabilizer.update(obstacle::detect(scene(), c));
        check(!miss.hasPrimary, "short disappearance does not invent a visible target");
        auto recovered = stabilizer.update(obstacle::detect(targetFrame, c));
        check(recovered.hasPrimary && recovered.primary.stable,
              "short disappearance allows the stable track to recover");
        for (int i = 0; i < c.maxMissedFrames + 1; ++i) miss = stabilizer.update(obstacle::detect(scene(), c));
        check(!miss.hasPrimary, "long disappearance removes stale track");
        std::cout << "All obstacle tests passed. Color thresholds need venue calibration.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
