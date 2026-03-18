/**
 * mavlink_manager_concurrent_runner.hpp - MAVLink 周期任务并发运行器
 *
 * 将“连接 / 遥测 / 心跳 / 认证 / 重连”作为 5 个周期任务，由同一事件循环（io_context）按固定间隔触发，
 * 实际执行投递到线程池，实现事件驱动式并发。详见 docs/CONCURRENT_MAVLINK_DESIGN.md。
 */
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#ifdef USE_BOOST_ASIO
#include <boost/asio.hpp>
#endif

namespace drone_control {

/**
 * 基于 Asio 定时器 + 线程池的周期任务运行器。
 * 每个任务按指定间隔在 io_context 上被调度，回调在线程池中执行，避免阻塞事件循环。
 */
class MAVLinkManagerConcurrentRunner {
public:
    using Task = std::function<void()>;
    using Interval = std::chrono::steady_clock::duration;

    MAVLinkManagerConcurrentRunner() = default;

#ifdef USE_BOOST_ASIO
    /**
     * @param io_context 事件循环，可与 HttpServer 共用
     * @param pool_size  执行阻塞任务的线程池大小，默认 4
     */
    explicit MAVLinkManagerConcurrentRunner(
        boost::asio::io_context& io_context,
        std::size_t pool_size = 4u
    );
#endif

    ~MAVLinkManagerConcurrentRunner();

    /** 添加周期任务：每 interval 在线程池中执行 task */
    void addPeriodicTask(Interval interval, Task task);

    /** 启动所有已添加的周期任务 */
    void start();

    /** 停止所有定时器并等待线程池收尾 */
    void stop();

    bool running() const { return running_; }

private:
    MAVLinkManagerConcurrentRunner(const MAVLinkManagerConcurrentRunner&) = delete;
    MAVLinkManagerConcurrentRunner& operator=(const MAVLinkManagerConcurrentRunner&) = delete;

#ifdef USE_BOOST_ASIO
    struct TimerTask {
        std::unique_ptr<boost::asio::steady_timer> timer;
        Interval interval;
        Task task;
    };
    void scheduleNext(TimerTask& tt);

    boost::asio::io_context* io_context_{ nullptr };
    std::unique_ptr<boost::asio::thread_pool> pool_;
    std::vector<TimerTask> tasks_;
#endif

    std::atomic<bool> running_{ false };
};

} // namespace drone_control
