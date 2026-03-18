#pragma once

/**
 * HTTP 服务
 * 提供 REST 接口：按路径与方法注册回调，支持无 body / 带 body 两种；可选静态文件；可用 Boost.Asio 或阻塞 socket 实现。
 */
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <map>
#include <memory>
#include <cstddef>

#ifdef USE_BOOST_ASIO
#include <boost/asio.hpp>
#endif

namespace drone_control {

class ThreadPool;

/** 按路径+方法分发请求，支持静态目录与线程池 */
class HttpServer {
public:
    /** 无 body 的回调：path, method -> 完整 HTTP 响应字符串 */
    using RequestHandler = std::function<std::string(const std::string& path, const std::string& method)>;
    /** 带 body 的回调：path, method, body -> 完整 HTTP 响应字符串 */
    using RequestHandlerWithBody = std::function<std::string(const std::string& path, const std::string& method, const std::string& body)>;

    HttpServer(const std::string& host = "0.0.0.0", int port = 8080);
    ~HttpServer();

    void addRoute(const std::string& path, const std::string& method, RequestHandler handler);
    void addRouteWithBody(const std::string& path, const std::string& method, RequestHandlerWithBody handler);
    void setStaticRoot(const std::string& root_path);
    void setThreadPoolSize(std::size_t size);
    void setSocketTimeoutMs(int timeout_ms);
    bool start();
    void stop();
    bool isRunning() const { return running_; }

    /** 供内部或测试用：传入原始请求字符串，返回完整响应字符串 */
    std::string processRequest(const std::string& request);

private:
    std::string host_;
    int port_;
    std::string static_root_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    std::map<std::string, RequestHandler> routes_;
    std::map<std::string, RequestHandlerWithBody> routes_with_body_;

    std::size_t thread_pool_size_;
    int socket_timeout_ms_;
    std::unique_ptr<ThreadPool> thread_pool_;

#ifdef USE_BOOST_ASIO
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    void do_accept();
#endif

    void serverLoop();
    void handleClient(int client_fd);
    std::string handleRequest(const std::string& request);
    std::string parseRequestPath(const std::string& request);
    std::string parseRequestMethod(const std::string& request);
    std::string parseRequestBody(const std::string& request);
    std::string createResponse(int status_code, const std::string& content_type, const std::string& body);
    std::string serveStaticFile(const std::string& path);
    std::string getMimeType(const std::string& file_path);
};

} // namespace drone_control