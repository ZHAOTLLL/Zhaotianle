/**
 * 错误恢复管理
 * 错误分类、重试与服务降级策略，保障访问控制服务可用性。
 */
#include "error/error_recovery_manager.hpp"
#include <algorithm>
#include <thread>

namespace drone_control {

ErrorRecoveryManager::ErrorRecoveryManager(const Config& config)
    : config_(config), running_(false), degraded_mode_(false) {
}

ErrorRecoveryManager::~ErrorRecoveryManager() {
    stop();
}

void ErrorRecoveryManager::handleError(const std::string& error_code, ErrorSeverity severity,
                                       const std::string& context, const std::string& description) {
    ErrorEvent error(error_code, severity, context, description);
    
    // 更新统计信息
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        error_statistics_[error_code]++;
    }
    
    // 添加到历史记录
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        error_history_.push_back(error);
        
        // 限制历史记录大小
        if (error_history_.size() > config_.max_error_queue_size) {
            error_history_.erase(error_history_.begin());
        }
    }
    
    // 立即处理严重错误
    if (severity == ErrorSeverity::CRITICAL) {
        notifyCallbacks(error);
        
        // 对于严重错误，尝试立即恢复
        ErrorEvent mutable_error = error;
        executeRecovery(mutable_error);
        return;
    }
    
    // 检查是否应激活服务降级
    if (shouldActivateDegradation(error)) {
        degraded_mode_.store(true);
    }
    
    // 将非严重错误添加到处理队列
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (error_queue_.size() < config_.max_error_queue_size) {
            error_queue_.push(error);
        }
    }
    
    // 通知回调
    notifyCallbacks(error);
}

void ErrorRecoveryManager::registerRecoveryStrategy(const std::string& error_code,
                                                     RecoveryStrategy recovery_func) {
    recovery_strategies_[error_code] = recovery_func;
}

void ErrorRecoveryManager::registerNotificationCallback(ErrorNotificationCallback callback) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    notification_callbacks_.push_back(callback);
}

void ErrorRecoveryManager::setServiceDegradationMode(bool enabled) {
    degraded_mode_.store(enabled);
}

bool ErrorRecoveryManager::isInDegradedMode() const {
    return degraded_mode_.load();
}

std::map<std::string, int> ErrorRecoveryManager::getErrorStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return error_statistics_;
}

std::vector<ErrorEvent> ErrorRecoveryManager::getRecentErrors(size_t max_count) const {
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    std::vector<ErrorEvent> recent_errors;
    size_t start_index = error_history_.size() > max_count ? 
                        error_history_.size() - max_count : 0;
    
    for (size_t i = start_index; i < error_history_.size(); ++i) {
        recent_errors.push_back(error_history_[i]);
    }
    
    return recent_errors;
}

void ErrorRecoveryManager::clearResolvedErrors() {
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    auto it = std::remove_if(error_history_.begin(), error_history_.end(),
                            [](const ErrorEvent& error) { return error.resolved; });
    error_history_.erase(it, error_history_.end());
}

void ErrorRecoveryManager::start() {
    if (running_.load()) {
        return;
    }
    
    running_.store(true);
    processing_thread_ = std::make_unique<std::thread>(&ErrorRecoveryManager::processErrors, this);
}

void ErrorRecoveryManager::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    
    if (processing_thread_ && processing_thread_->joinable()) {
        processing_thread_->join();
    }
}

bool ErrorRecoveryManager::isRunning() const {
    return running_.load();
}

void ErrorRecoveryManager::processErrors() {
    while (running_.load()) {
        ErrorEvent error("", ErrorSeverity::LOW, "");
        bool has_error = false;
        
        // 从队列获取下一个错误
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!error_queue_.empty()) {
                error = error_queue_.front();
                error_queue_.pop();
                has_error = true;
            }
        }
        
        if (has_error) {
            // 尝试从错误中恢复
            executeRecovery(error);
        } else {
            // 没有错误要处理，短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 定期清理旧错误
        static auto last_cleanup = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - last_cleanup > std::chrono::minutes(5)) {
            cleanupOldErrors();
            last_cleanup = now;
        }
    }
}

bool ErrorRecoveryManager::executeRecovery(ErrorEvent& error) {
    auto strategy_it = recovery_strategies_.find(error.error_code);
    
    if (strategy_it == recovery_strategies_.end()) {
        // 基于严重程度的默认恢复
        switch (error.severity) {
            case ErrorSeverity::LOW:
                error.resolved = true;
                return true;
                
            case ErrorSeverity::MEDIUM:
                if (error.retry_count < config_.max_retry_attempts) {
                    error.retry_count++;
                    auto delay = calculateRetryDelay(error.retry_count);
                    std::this_thread::sleep_for(delay);
                    
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        error_queue_.push(error);
                    }
                    return false;
                } else {
                    error.resolved = false;
                    return false;
                }
                
            case ErrorSeverity::HIGH:
            case ErrorSeverity::CRITICAL:
                error.resolved = false;
                return false;
        }
        
        return false;
    }
    
    // Execute the registered recovery strategy
    try {
        bool recovery_success = strategy_it->second(error);
        
        if (recovery_success) {
            error.resolved = true;
            
            if (degraded_mode_.load() && error.severity >= ErrorSeverity::MEDIUM) {
                degraded_mode_.store(false);
            }
            
            return true;
        } else {
            if (error.severity != ErrorSeverity::CRITICAL && error.retry_count < config_.max_retry_attempts) {
                error.retry_count++;
                auto delay = calculateRetryDelay(error.retry_count);
                std::this_thread::sleep_for(delay);
                
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    error_queue_.push(error);
                }
                return false;
            } else {
                error.resolved = false;
                return false;
            }
        }
    } catch (const std::exception& e) {
        error.resolved = false;
        return false;
    }
}

void ErrorRecoveryManager::notifyCallbacks(const ErrorEvent& error) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    
    for (const auto& callback : notification_callbacks_) {
        try {
            callback(error);
        } catch (const std::exception& e) {
            // Exception in callback, silently continue
        }
    }
}

std::chrono::milliseconds ErrorRecoveryManager::calculateRetryDelay(int retry_count) const {
    // Exponential backoff: base_delay * (multiplier ^ (retry_count - 1))
    auto delay = config_.retry_delay;
    for (int i = 1; i < retry_count; ++i) {
        delay = std::chrono::milliseconds(delay.count() * config_.retry_backoff_multiplier.count());
    }
    
    // Cap the maximum delay at 30 seconds
    const auto max_delay = std::chrono::milliseconds(30000);
    return std::min(delay, max_delay);
}

void ErrorRecoveryManager::cleanupOldErrors() {
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto cutoff_time = now - config_.error_retention_time;
    
    auto it = std::remove_if(error_history_.begin(), error_history_.end(),
                            [cutoff_time](const ErrorEvent& error) {
                                return error.timestamp < cutoff_time && error.resolved;
                            });
    
    error_history_.erase(it, error_history_.end());
}

bool ErrorRecoveryManager::shouldActivateDegradation(const ErrorEvent& error) const {
    // Activate degradation for high severity errors or repeated medium severity errors
    if (error.severity >= ErrorSeverity::HIGH) {
        return true;
    }
    
    if (error.severity == ErrorSeverity::MEDIUM) {
        // Check if we've seen this error type multiple times recently
        std::lock_guard<std::mutex> lock(stats_mutex_);
        auto it = error_statistics_.find(error.error_code);
        if (it != error_statistics_.end() && it->second >= 3) {
            return true;
        }
    }
    
    return false;
}

} // namespace drone_control