/**
 * HTTP 服务实现
 * 提供线程池、路由表、请求解析与响应组装；可选 Boost.Asio 异步或阻塞 socket 循环。
 */
#include "http_server.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <functional>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#ifdef USE_BOOST_ASIO
#include <boost/asio.hpp>
#endif

namespace drone_control {

namespace {

const char kContentTypeJson[] = "application/json";

std::string makeErrorJsonBody(const std::string& message) {
    std::string escaped;
    escaped.reserve(message.size() + 8);
    for (char c : message) {
        if (c == '"') escaped += "\\\"";
        else if (c == '\\') escaped += "\\\\";
        else if (c == '\n') escaped += "\\n";
        else if (c == '\r') escaped += "\\r";
        else if (c == '\t') escaped += "\\t";
        else escaped += c;
    }
    return "{\"error\": \"" + escaped + "\"}";
}

}  // namespace

/** 固定数量工作线程，从队列取任务执行，用于处理 HTTP 请求 */
class ThreadPool {
public:
    using Task = std::function<void()>;
    explicit ThreadPool(std::size_t num_threads) : stopping_(false) {
        if (num_threads == 0) num_threads = 4;
        for (std::size_t i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this]() {
                for (;;) {
                    Task task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        cv_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });
                        if (stopping_ && tasks_.empty()) return;
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    try { task(); } catch (const std::exception& e) { std::cerr << "ThreadPool: " << e.what() << std::endl; }
                }
            });
        }
    }
    ~ThreadPool() { shutdown(); }
    void enqueue(Task task) {
        { std::lock_guard<std::mutex> lock(mutex_); if (stopping_) return; tasks_.push(std::move(task)); }
        cv_.notify_one();
    }
    void shutdown() {
        { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
        cv_.notify_all();
        for (auto& w : workers_) if (w.joinable()) w.join();
        workers_.clear();
    }
private:
    std::vector<std::thread> workers_;
    std::queue<Task> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_;
};

#ifdef USE_BOOST_ASIO
namespace {
namespace asio = boost::asio;
using tcp = boost::asio::ip::tcp;
class AsioConnection : public std::enable_shared_from_this<AsioConnection> {
public:
    AsioConnection(tcp::socket socket, HttpServer& server, ThreadPool& pool, int timeout_ms)
        : socket_(std::move(socket)), server_(server), pool_(pool), timeout_ms_(timeout_ms) {}
    void start() { read_headers(); }
private:
    void read_headers() {
        auto self = shared_from_this();
        asio::async_read_until(socket_, buf_, "\r\n\r\n", [this, self](boost::system::error_code ec, std::size_t) {
            if (ec) return;
            std::string all(asio::buffers_begin(buf_.data()), asio::buffers_end(buf_.data()));
            buf_.consume(buf_.size());
            std::size_t pos = all.find("\r\n\r\n");
            std::string headers = (pos != std::string::npos) ? all.substr(0, pos) : all;
            std::string body_part = (pos != std::string::npos && pos + 4 <= all.size()) ? all.substr(pos + 4) : "";
            std::size_t content_length = 0;
            for (std::size_t i = 0; i + 15 < headers.size(); ) {
                if (headers.compare(i, 15, "Content-Length:") == 0) {
                    std::size_t j = i + 15;
                    while (j < headers.size() && headers[j] == ' ') ++j;
                    content_length = static_cast<std::size_t>(std::stoull(headers.substr(j)));
                    break;
                }
                std::size_t nl = headers.find('\n', i);
                i = (nl != std::string::npos) ? nl + 1 : headers.size();
            }
            if (content_length <= body_part.size()) {
                on_request_ready(headers + "\r\n\r\n" + body_part);
                return;
            }
            need_read_ = content_length - body_part.size();
            rest_.assign(body_part.begin(), body_part.end());
            rest_.resize(content_length);
            asio::async_read(socket_, asio::buffer(rest_.data() + body_part.size(), need_read_),
                [this, self, h = std::move(headers)](boost::system::error_code ec2, std::size_t) {
                    if (ec2) return;
                    on_request_ready(h + "\r\n\r\n" + std::string(rest_.begin(), rest_.end()));
                });
        });
    }
    void on_request_ready(std::string request) {
        auto self = shared_from_this();
        pool_.enqueue([this, self, req = std::move(request)]() {
            std::string resp = server_.processRequest(req);
            asio::post(socket_.get_executor(), [this, self, r = std::move(resp)]() {
                response_ = std::move(r);
                do_write();
            });
        });
    }
    void do_write() {
        auto self = shared_from_this();
        asio::async_write(socket_, asio::buffer(response_), [this, self](boost::system::error_code ec, std::size_t) {
            (void)ec;
            boost::system::error_code ign;
            socket_.shutdown(tcp::socket::shutdown_both, ign);
            socket_.close(ign);
        });
    }
    tcp::socket socket_;
    HttpServer& server_;
    ThreadPool& pool_;
    int timeout_ms_;
    asio::streambuf buf_;
    std::vector<char> rest_;
    std::size_t need_read_ = 0;
    std::string response_;
};
}
#endif

HttpServer::HttpServer(const std::string& host, int port)
    : host_(host), port_(port), static_root_(""), running_(false),
      thread_pool_size_(0), socket_timeout_ms_(5000) {
    unsigned int hc = std::thread::hardware_concurrency();
    thread_pool_size_ = (hc > 0) ? hc : 4;
}

HttpServer::~HttpServer() {
    stop();
}

void HttpServer::addRoute(const std::string& path, const std::string& method, RequestHandler handler) {
    std::string key = method + " " + path;
    routes_[key] = handler;
}

void HttpServer::addRouteWithBody(const std::string& path, const std::string& method, RequestHandlerWithBody handler) {
    std::string key = method + " " + path;
    routes_with_body_[key] = handler;
}

void HttpServer::setStaticRoot(const std::string& root_path) {
    static_root_ = root_path;
}

void HttpServer::setThreadPoolSize(std::size_t size) {
    if (running_) return;
    if (size > 0) thread_pool_size_ = size;
}

void HttpServer::setSocketTimeoutMs(int timeout_ms) {
    if (timeout_ms > 0) socket_timeout_ms_ = timeout_ms;
}

std::string HttpServer::processRequest(const std::string& request) {
    return handleRequest(request);
}

bool HttpServer::start() {
    if (running_) return false;
    running_ = true;
#ifdef USE_BOOST_ASIO
    thread_pool_ = std::make_unique<ThreadPool>(thread_pool_size_);
    boost::system::error_code ec;
    io_context_ = std::make_unique<boost::asio::io_context>();
    boost::asio::ip::tcp::endpoint ep = (host_ == "0.0.0.0")
        ? boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), static_cast<unsigned short>(port_))
        : boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(host_, ec), static_cast<unsigned short>(port_));
    acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(*io_context_, ep);
    acceptor_->listen(boost::asio::socket_base::max_listen_connections, ec);
    do_accept();
    server_thread_ = std::thread([this]() { io_context_->run(); });
    std::cout << "HTTP Server (Boost.Asio) listening on " << host_ << ":" << port_ << std::endl;
#else
    server_thread_ = std::thread(&HttpServer::serverLoop, this);
    std::cout << "HTTP Server listening on " << host_ << ":" << port_ << std::endl;
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

void HttpServer::stop() {
    if (!running_) return;
    running_ = false;
#ifdef USE_BOOST_ASIO
    if (io_context_) io_context_->stop();
    if (acceptor_) { boost::system::error_code ign; acceptor_->close(ign); }
#endif
    if (server_thread_.joinable()) server_thread_.join();
    if (thread_pool_) { thread_pool_->shutdown(); thread_pool_.reset(); }
#ifdef USE_BOOST_ASIO
    io_context_.reset();
    acceptor_.reset();
#endif
}

#ifdef USE_BOOST_ASIO
void HttpServer::do_accept() {
    if (!acceptor_ || !acceptor_->is_open()) return;
    acceptor_->async_accept([this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket) {
        if (!ec && thread_pool_)
            std::make_shared<AsioConnection>(std::move(socket), *this, *thread_pool_, socket_timeout_ms_)->start();
        if (running_) do_accept();
    });
}
#endif

void HttpServer::handleClient(int client_fd) {
    char buffer[4096] = {0};
    ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read <= 0) { close(client_fd); return; }
    std::string request(buffer, static_cast<std::size_t>(bytes_read));
    std::string response = handleRequest(request);
    send(client_fd, response.c_str(), response.length(), 0);
    close(client_fd);
}

void HttpServer::serverLoop() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { std::cerr << "Failed to create socket" << std::endl; return; }
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (host_ == "0.0.0.0") address.sin_addr.s_addr = INADDR_ANY;
    else inet_pton(AF_INET, host_.c_str(), &address.sin_addr);
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Fatal error: bind: Address already in use" << std::endl;
        close(server_fd);
        return;
    }
    if (listen(server_fd, 10) < 0) { close(server_fd); return; }
    while (running_) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) continue;
        handleClient(client_fd);
    }
    close(server_fd);
    std::cout << "HTTP Server stopped" << std::endl;
}

std::string HttpServer::handleRequest(const std::string& request) {
    std::string method = parseRequestMethod(request);
    std::string full_path = parseRequestPath(request);
    std::string body = parseRequestBody(request);
    
    // 为了路由匹配，去掉查询参数
    std::string path = full_path;
    size_t qpos = full_path.find('?');
    if (qpos != std::string::npos) {
        path = full_path.substr(0, qpos);
    }
    
    std::string key = method + " " + path;
    
    auto it_body = routes_with_body_.find(key);
    if (it_body != routes_with_body_.end()) {
        try {
            return createResponse(200, kContentTypeJson, it_body->second(full_path, method, body));
        } catch (const std::exception& e) {
            return createResponse(500, kContentTypeJson, makeErrorJsonBody(e.what()));
        }
    }
    auto it = routes_.find(key);
    if (it != routes_.end()) {
        try {
            return createResponse(200, kContentTypeJson, it->second(full_path, method));
        } catch (const std::exception& e) {
            return createResponse(500, kContentTypeJson, makeErrorJsonBody(e.what()));
        }
    }
    if (!static_root_.empty() && method == "GET") {
        std::string static_response = serveStaticFile(path);
        if (!static_response.empty()) return static_response;
    }
    std::string not_found_body = "{\"error\": \"Not Found\", \"path\": \"" + path + "\"}";
    return createResponse(404, kContentTypeJson, not_found_body);
}

std::string HttpServer::parseRequestMethod(const std::string& request) {
    size_t space_pos = request.find(' ');
    if (space_pos != std::string::npos) {
        return request.substr(0, space_pos);
    }
    return "GET";
}

std::string HttpServer::parseRequestPath(const std::string& request) {
    size_t first_space = request.find(' ');
    if (first_space != std::string::npos) {
        size_t second_space = request.find(' ', first_space + 1);
        if (second_space != std::string::npos) {
            std::string full_path = request.substr(first_space + 1, second_space - first_space - 1);
            // 保留查询字符串，让处理函数自己解析
            return full_path;
        }
    }
    return "/";
}

std::string HttpServer::parseRequestBody(const std::string& request) {
    // 查找请求体开始位置（\r\n\r\n之后）
    size_t header_end = request.find("\r\n\r\n");
    if (header_end != std::string::npos) {
        return request.substr(header_end + 4);
    }
    return "";
}

std::string HttpServer::createResponse(int status_code, const std::string& content_type, const std::string& body) {
    std::ostringstream response;
    
    std::string status_text;
    switch (status_code) {
        case 200: status_text = "OK"; break;
        case 404: status_text = "Not Found"; break;
        case 500: status_text = "Internal Server Error"; break;
        default: status_text = "Unknown"; break;
    }
    
    response << "HTTP/1.1 " << status_code << " " << status_text << "\r\n";
    response << "Content-Type: " << content_type << "\r\n";
    response << "Content-Length: " << body.length() << "\r\n";
    response << "Access-Control-Allow-Origin: *\r\n";
    response << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
    response << "Access-Control-Allow-Headers: Content-Type, Authorization\r\n";
    response << "Connection: close\r\n";
    response << "\r\n";
    response << body;
    
    return response.str();
}

std::string HttpServer::serveStaticFile(const std::string& path) {
    // 防止路径遍历攻击
    if (path.find("..") != std::string::npos) {
        return "";
    }
    
    // 构建文件路径
    std::string file_path = static_root_;
    if (path == "/" || path.empty()) {
        file_path += "/index.html";
    } else if (path == "/index.html") {
        file_path += "/index.html";
    } else {
        // 如果路径以/web/开头，去掉/web前缀
        if (path.substr(0, 5) == "/web/") {
            file_path += path.substr(4); // 去掉/web，保留/
        } else {
            file_path += path;
        }
    }
    
    // 检查文件是否存在
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        return "";
    }
    
    // 读取文件内容
    std::ostringstream content;
    content << file.rdbuf();
    file.close();
    
    // 获取MIME类型
    std::string mime_type = getMimeType(file_path);
    
    return createResponse(200, mime_type, content.str());
}

std::string HttpServer::getMimeType(const std::string& file_path) {
    size_t dot_pos = file_path.find_last_of('.');
    if (dot_pos == std::string::npos) {
        return "application/octet-stream";
    }
    
    std::string extension = file_path.substr(dot_pos + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    
    if (extension == "html" || extension == "htm") {
        return "text/html; charset=utf-8";
    } else if (extension == "css") {
        return "text/css";
    } else if (extension == "js") {
        return "application/javascript";
    } else if (extension == "json") {
        return kContentTypeJson;
    } else if (extension == "png") {
        return "image/png";
    } else if (extension == "jpg" || extension == "jpeg") {
        return "image/jpeg";
    } else if (extension == "gif") {
        return "image/gif";
    } else if (extension == "svg") {
        return "image/svg+xml";
    } else if (extension == "ico") {
        return "image/x-icon";
    } else {
        return "application/octet-stream";
    }
}

} // namespace drone_control