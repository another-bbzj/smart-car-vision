#pragma once
// Browser-based MJPEG debug viewer for a headless Orange Pi.
//
// The vision process publishes named frames exactly like the existing TCP
// viewer; any browser on the same LAN opens http://<board-ip>:8080/ and sees
// the raw camera image and the processed images side by side. No client
// software is required on the Windows machine.
//
// Server side:
//   HttpMjpegServer http(8080);
//   http.publish("Camera", frame);
//
// HTTP endpoints:
//   GET /                     HTML page with all published streams
//   GET /stream?name=NAME     multipart/x-mixed-replace MJPEG stream
//   GET /snapshot?name=NAME   last JPEG of that name
//
// Frames are JPEG-encoded only while at least one client watches that name,
// and at most once per minIntervalMs, so an idle or closed page costs nothing.

#include <opencv2/opencv.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class HttpMjpegServer {
public:
    explicit HttpMjpegServer(uint16_t port = 8080, int jpegQuality = 75,
                             int minIntervalMs = 100,
                             const std::string& bindIp = "0.0.0.0")
        : port_(port), jpegQuality_(jpegQuality),
          interval_(std::chrono::milliseconds(minIntervalMs)),
          listeningFd_(-1) {
        if (jpegQuality_ < 1 || jpegQuality_ > 100) {
            throw std::invalid_argument("jpegQuality must be 1..100");
        }
        if (minIntervalMs < 0) {
            throw std::invalid_argument("minIntervalMs must be >= 0");
        }

        listeningFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listeningFd_ < 0) throw std::runtime_error("socket() failed");

        int reuse = 1;
        ::setsockopt(listeningFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        if (::inet_pton(AF_INET, bindIp.c_str(), &addr.sin_addr) != 1) {
            ::close(listeningFd_);
            listeningFd_ = -1;
            throw std::invalid_argument("Invalid HTTP bind IPv4 address: " + bindIp);
        }
        if (::bind(listeningFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
            ::listen(listeningFd_, 8) < 0) {
            const int err = errno;
            ::close(listeningFd_);
            listeningFd_ = -1;
            throw std::runtime_error("HTTP bind/listen failed (port in use?): " + std::to_string(err));
        }

        acceptThread_ = std::thread(&HttpMjpegServer::acceptLoop, this);
        std::cout << "[HTTP] Debug viewer on port " << port_
                  << ": open http://<board-ip>:" << port_ << "/ in a browser" << std::endl;
    }

    HttpMjpegServer(const HttpMjpegServer&) = delete;
    HttpMjpegServer& operator=(const HttpMjpegServer&) = delete;

    ~HttpMjpegServer() {
        stop_.store(true);
        if (listeningFd_ >= 0) {
            // Wakes select()/accept() in the listener thread.
            ::shutdown(listeningFd_, SHUT_RDWR);
        }
        if (acceptThread_.joinable()) acceptThread_.join();
        if (listeningFd_ >= 0) {
            ::close(listeningFd_);
            listeningFd_ = -1;
        }
        cv_.notify_all();
        std::vector<Worker> workers;
        {
            std::lock_guard<std::mutex> lock(workersMutex_);
            workers.swap(workers_);
        }
        for (auto& worker : workers) {
            if (worker.thread.joinable()) worker.thread.join();
        }
    }

    // Called once per frame from the vision loop. Encoding and copying are
    // skipped unless a browser is actually watching this name.
    void publish(const std::string& name, const cv::Mat& image) {
        if (image.empty() || stop_.load()) return;

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = channels_.find(name);
        if (it == channels_.end() || it->second.subscribers <= 0) return;
        if (it->second.generation != 0 && now - it->second.last < interval_) return;

        cv::Mat view = image;
        if (view.channels() == 4) {
            cv::cvtColor(view, view, cv::COLOR_BGRA2BGR);
        }
        if (view.depth() != CV_8U) {
            view.convertTo(view, CV_8U);
        }
        if (view.channels() != 1 && view.channels() != 3) return;

        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", view, jpeg,
                          {cv::IMWRITE_JPEG_QUALITY, jpegQuality_})) {
            return;
        }
        it->second.jpeg = std::move(jpeg);
        it->second.last = now;
        ++it->second.generation;
        cv_.notify_all();
    }

private:
    struct Worker {
        std::thread thread;
        std::shared_ptr<std::atomic<bool>> done;
    };

    struct Channel {
        int subscribers = 0;
        std::uint64_t generation = 0;
        std::chrono::steady_clock::time_point last{};
        std::vector<unsigned char> jpeg;
    };

    static bool sendAll(int fd, const void* data, std::size_t size) {
        const char* p = static_cast<const char*>(data);
        while (size != 0) {
            const ssize_t sent = ::send(fd, p, size, MSG_NOSIGNAL);
            if (sent <= 0) {
                if (sent < 0 && errno == EINTR) continue;
                return false;
            }
            p += sent;
            size -= static_cast<std::size_t>(sent);
        }
        return true;
    }

    static bool sendSimple(int fd, const char* status, const std::string& body) {
        const std::string response =
            std::string("HTTP/1.1 ") + status +
            "\r\nContent-Type: text/plain; charset=utf-8\r\n"
            "Content-Length: " + std::to_string(body.size()) +
            "\r\nConnection: close\r\n\r\n" + body;
        return sendAll(fd, response.data(), response.size());
    }

    static std::string queryValue(const std::string& query, const std::string& key) {
        std::size_t pos = 0;
        while (pos < query.size()) {
            const std::size_t amp = query.find('&', pos);
            const std::string item = query.substr(
                pos, amp == std::string::npos ? std::string::npos : amp - pos);
            const std::size_t eq = item.find('=');
            if (eq != std::string::npos && item.substr(0, eq) == key) {
                return item.substr(eq + 1);
            }
            if (amp == std::string::npos) break;
            pos = amp + 1;
        }
        return {};
    }

    static const std::string& indexPage() {
        static const std::string page = R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Vision Debug</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; padding: 16px; background: #111; color: #ddd;
         font: 14px/1.4 system-ui, "Microsoft YaHei", sans-serif; }
  h1 { font-size: 16px; font-weight: 600; margin: 0 0 8px; }
  .hint { color: #888; margin: 0 0 16px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(380px, 1fr)); gap: 14px; }
  figure { margin: 0; background: #1b1b1b; border: 1px solid #2a2a2a; border-radius: 8px; overflow: hidden; }
  figcaption { padding: 8px 10px; font-size: 13px; color: #bbb; border-bottom: 1px solid #2a2a2a; }
  figcaption b { color: #fff; font-weight: 600; }
  img { display: block; width: 100%; height: auto; background: #000; }
</style>
</head>
<body>
<h1>Vision Debug — 实时画面</h1>
<p class="hint">Camera 为摄像头原始画面，其余为处理结果，约 10 fps。关闭本页即断开，不影响车辆。</p>
<div class="grid">
  <figure><figcaption><b>Camera</b> 摄像头原始画面</figcaption><img src="/stream?name=Camera" alt="Camera"></figure>
  <figure><figcaption><b>bird</b> 鸟瞰 + 中线/预瞄</figcaption><img src="/stream?name=bird" alt="bird"></figure>
  <figure><figcaption><b>maze</b> 巡线（红左 / 蓝右）</figcaption><img src="/stream?name=maze" alt="maze"></figure>
  <figure><figcaption><b>mask</b> 白色道路掩膜</figcaption><img src="/stream?name=mask" alt="mask"></figure>
  <figure><figcaption><b>blue</b> 蓝底掩膜</figcaption><img src="/stream?name=blue" alt="blue"></figure>
  <figure><figcaption><b>obstacles</b> 三色路障</figcaption><img src="/stream?name=obstacles" alt="obstacles"></figure>
</div>
</body>
</html>
)HTML";
        return page;
    }

    void sendPage(int fd) {
        const std::string& page = indexPage();
        const std::string header =
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
            "Content-Length: " + std::to_string(page.size()) +
            "\r\nConnection: close\r\n\r\n";
        sendAll(fd, header.data(), header.size());
        sendAll(fd, page.data(), page.size());
    }

    void sendSnapshot(int fd, const std::string& name) {
        std::vector<unsigned char> jpeg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = channels_.find(name);
            if (it != channels_.end()) jpeg = it->second.jpeg;
        }
        if (jpeg.empty()) {
            sendSimple(fd, "404 Not Found", "No frame for '" + name + "' yet\n");
            return;
        }
        const std::string header =
            "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n"
            "Cache-Control: no-store\r\n"
            "Content-Length: " + std::to_string(jpeg.size()) +
            "\r\nConnection: close\r\n\r\n";
        sendAll(fd, header.data(), header.size());
        sendAll(fd, jpeg.data(), jpeg.size());
    }

    void subscribe(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++channels_[name].subscribers;
    }

    void unsubscribe(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = channels_.find(name);
        if (it != channels_.end() && it->second.subscribers > 0) {
            --it->second.subscribers;
        }
    }

    void stream(int fd, const std::string& name) {
        subscribe(name);
        static const char header[] =
            "HTTP/1.1 200 OK\r\n"
            "Connection: close\r\n"
            "Cache-Control: no-store\r\n"
            "Pragma: no-cache\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";

        bool ok = sendAll(fd, header, sizeof(header) - 1);
        std::uint64_t lastGeneration = 0;
        while (ok && !stop_.load()) {
            std::vector<unsigned char> jpeg;
            std::uint64_t generation = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                Channel& channel = channels_[name];
                cv_.wait_for(lock, std::chrono::milliseconds(500), [&] {
                    return stop_.load() || channel.generation != lastGeneration;
                });
                if (stop_.load()) break;
                if (channel.generation == lastGeneration) {
                    // No new frame for a while: check whether the browser left.
                    lock.unlock();
                    char probe;
                    const ssize_t got = ::recv(fd, &probe, 1, MSG_PEEK | MSG_DONTWAIT);
                    if (got == 0) break;
                    if (got < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) break;
                    continue;
                }
                jpeg = channel.jpeg;
                generation = channel.generation;
            }
            lastGeneration = generation;
            if (jpeg.empty()) continue;

            const std::string part =
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                std::to_string(jpeg.size()) + "\r\n\r\n";
            ok = sendAll(fd, part.data(), part.size()) &&
                 sendAll(fd, jpeg.data(), jpeg.size()) &&
                 sendAll(fd, "\r\n", 2);
        }
        unsubscribe(name);
    }

    void handleClient(int fd, std::shared_ptr<std::atomic<bool>> done) {
        struct DoneGuard {
            std::shared_ptr<std::atomic<bool>> flag;
            ~DoneGuard() { flag->store(true, std::memory_order_release); }
        } guard{std::move(done)};

        timeval recvTimeout{3, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
        timeval sendTimeout{2, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));

        std::string request;
        char buffer[1024];
        while (request.size() < 4096) {
            const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
            if (got <= 0) {
                ::close(fd);
                return;
            }
            request.append(buffer, static_cast<std::size_t>(got));
            if (request.find("\r\n\r\n") != std::string::npos) break;
        }

        const std::size_t lineEnd = request.find("\r\n");
        const std::string line = request.substr(
            0, lineEnd == std::string::npos ? request.size() : lineEnd);
        std::string method, target;
        {
            std::istringstream stream(line);
            stream >> method >> target;
        }
        if (method != "GET") {
            sendSimple(fd, "405 Method Not Allowed", "Only GET is supported\n");
            ::close(fd);
            return;
        }

        std::string path = target;
        std::string query;
        const std::size_t question = target.find('?');
        if (question != std::string::npos) {
            path = target.substr(0, question);
            query = target.substr(question + 1);
        }

        if (path == "/" || path == "/index.html") {
            sendPage(fd);
        } else if (path == "/stream") {
            const std::string name = queryValue(query, "name");
            if (name.empty()) {
                sendSimple(fd, "400 Bad Request", "Missing ?name=\n");
            } else {
                stream(fd, name);
            }
        } else if (path == "/snapshot") {
            const std::string name = queryValue(query, "name");
            if (name.empty()) {
                sendSimple(fd, "400 Bad Request", "Missing ?name=\n");
            } else {
                sendSnapshot(fd, name);
            }
        } else {
            sendSimple(fd, "404 Not Found", "Use /, /stream?name=NAME or /snapshot?name=NAME\n");
        }
        ::close(fd);
    }

    void reapFinishedWorkers() {
        std::lock_guard<std::mutex> lock(workersMutex_);
        auto it = workers_.begin();
        while (it != workers_.end()) {
            if (it->done->load(std::memory_order_acquire)) {
                if (it->thread.joinable()) it->thread.join();
                it = workers_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void acceptLoop() {
        while (!stop_.load()) {
            reapFinishedWorkers();
            fd_set set;
            FD_ZERO(&set);
            FD_SET(listeningFd_, &set);
            timeval tv{0, 250000};
            const int ready = ::select(listeningFd_ + 1, &set, nullptr, nullptr, &tv);
            if (stop_.load()) break;
            if (ready <= 0) continue;

            sockaddr_in peer{};
            socklen_t len = sizeof(peer);
            const int fd = ::accept(listeningFd_, reinterpret_cast<sockaddr*>(&peer), &len);
            if (fd < 0) continue;
            if (stop_.load()) {
                ::close(fd);
                break;
            }
            auto done = std::make_shared<std::atomic<bool>>(false);
            std::lock_guard<std::mutex> lock(workersMutex_);
            workers_.push_back(Worker{
                std::thread(&HttpMjpegServer::handleClient, this, fd, done),
                done
            });
        }
    }

    uint16_t port_;
    int jpegQuality_;
    std::chrono::milliseconds interval_;
    int listeningFd_;
    std::atomic<bool> stop_{false};

    std::thread acceptThread_;
    std::mutex workersMutex_;
    std::vector<Worker> workers_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::map<std::string, Channel> channels_;
};
