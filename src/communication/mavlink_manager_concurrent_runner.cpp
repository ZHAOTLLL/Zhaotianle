/**
 * MAVLink 并发运行器
 * 基于 Boost.Asio 的定时任务与线程池，周期性驱动 MAVLink 连接、遥测、心跳等。
 */
#include "communication/mavlink_manager_concurrent_runner.hpp"
#include <iostream>

namespace drone_control {

#ifdef USE_BOOST_ASIO

MAVLinkManagerConcurrentRunner::MAVLinkManagerConcurrentRunner(
    boost::asio::io_context& io_context,
    std::size_t pool_size
)
    : io_context_(&io_context)
    , pool_(std::make_unique<boost::asio::thread_pool>(pool_size > 0 ? pool_size : 4))
{
}

MAVLinkManagerConcurrentRunner::~MAVLinkManagerConcurrentRunner() {
    stop();
}

void MAVLinkManagerConcurrentRunner::addPeriodicTask(Interval interval, Task task) {
    if (!io_context_ || !pool_) return;
    auto timer = std::make_unique<boost::asio::steady_timer>(*io_context_);
    tasks_.push_back({ std::move(timer), interval, std::move(task) });
}

void MAVLinkManagerConcurrentRunner::scheduleNext(TimerTask& tt) {
    if (!running_.load() || !tt.timer || !io_context_) return;
    tt.timer->expires_after(tt.interval);
    tt.timer->async_wait([this, &tt](boost::system::error_code ec) {
        if (ec || !running_.load()) return;
        Task task = tt.task;
        boost::asio::post(*pool_, [task]() {
            try { task(); } catch (const std::exception& e) {
                std::cerr << "MAVLinkManagerConcurrentRunner task: " << e.what() << std::endl;
            }
        });
        scheduleNext(tt);
    });
}

void MAVLinkManagerConcurrentRunner::start() {
    if (running_.exchange(true)) return;
    for (auto& tt : tasks_)
        scheduleNext(tt);
    std::cout << "MAVLinkManagerConcurrentRunner started, " << tasks_.size() << " periodic task(s)." << std::endl;
}

void MAVLinkManagerConcurrentRunner::stop() {
    if (!running_.exchange(false)) return;
    for (auto& tt : tasks_) {
        if (tt.timer) {
            boost::system::error_code ign;
            tt.timer->cancel(ign);
        }
    }
    if (pool_)
        pool_->join();
    std::cout << "MAVLinkManagerConcurrentRunner stopped." << std::endl;
}

#else

MAVLinkManagerConcurrentRunner::~MAVLinkManagerConcurrentRunner() = default;

void MAVLinkManagerConcurrentRunner::addPeriodicTask(Interval, Task) {}
void MAVLinkManagerConcurrentRunner::start() {}
void MAVLinkManagerConcurrentRunner::stop() {}

#endif

} // namespace drone_control
