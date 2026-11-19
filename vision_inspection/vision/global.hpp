#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>


#define TCP_PORT 9000
#define TCP_BIND_IP "0.0.0.0"
#define JPEG_QUALITY 75
#define HTTP_PORT 8080

#include "TcpImshowServer.hpp"
#include "HttpMjpegServer.hpp"
#include <memory>

// Offline processing must not bind a port or start a network thread.
// 可选远程预览：默认离线处理不占端口，只有显式 enable 才启动网络服务。
// enable()     原有 TCP 查看器（9000，单客户端，需要配套客户端）
// enableHttp() 浏览器 MJPEG 查看器（8080，同一局域网任意浏览器）
class OptionalRemote {
public:
    void enable() { if (!server_) server_ = std::make_unique<TcpImshowServer>(TCP_PORT, JPEG_QUALITY, TCP_BIND_IP); }
    void enableHttp(uint16_t port = HTTP_PORT) {
        if (!http_) http_ = std::make_unique<HttpMjpegServer>(port);
    }
    void imshow(const std::string& name, const cv::Mat& image) {
        if (server_) server_->imshow(name, image);
        if (http_) http_->publish(name, image);
    }
private:
    std::unique_ptr<TcpImshowServer> server_;
    std::unique_ptr<HttpMjpegServer> http_;
};
inline OptionalRemote remote;


inline cv::Mat frame;
inline cv::Mat gray;
inline cv::Mat binary;
inline cv::Mat blurred;
inline cv::Mat edges;
inline cv::Mat opened;
inline cv::Mat closed;
inline cv::Mat eroded;
inline cv::Mat dilated;

inline std::vector<std::vector<cv::Point>> contours;
inline std::vector<cv::Point> leftLine;
inline std::vector<cv::Point> rightLine;

cv::Mat kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(3, 3));