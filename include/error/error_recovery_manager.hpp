/**
 * error_recovery_manager.hpp - 错误恢复与服务降级
 */
#pragma once

#include <string>
#include <map>
#include <queue>
#include <functional>
#include <memory>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

namespace drone_control {

/** 错误严重程度 */
enum class ErrorSeverity {
    LOW,      // 可自动恢复 - Can be automatically recovered
    MEDIUM,   // 需要重试或降级服务 - Requires retry or service degradation
    HIGH,     // 需要人工干预 - Requires manual intervention
    CRITICAL  // 系统安全相关，立即停止服务 - System safety related, stop service immediately
};

/**
 * @brief Error event structure containing all error information
 */
struct ErrorEvent {
    std::string error_code;
    ErrorSeverity severity;
    std::string context;
    std::string description;
    std::chrono::steady_clock::time_point timestamp;
    int retry_count;
    bool resolved;
    
    ErrorEvent(const std::string& code, ErrorSeverity sev, const std::string& ctx, 
               const std::string& desc = "")
        : error_code(code), severity(sev), context(ctx), description(desc),
          timestamp(std::chrono::steady_clock::now()), retry_count(0), resolved(false) {}
};

/**
 * @brief Recovery strategy function type
 */
using RecoveryStrategy = std::function<bool(const ErrorEvent&)>;

/**
 * @brief Error notification callback type
 */
using ErrorNotificationCallback = std::function<void(const ErrorEvent&)>;

/**
 * @brief Comprehensive error recovery manager with hierarchical error handling
 */
class ErrorRecoveryManager {
public:
    /**
     * @brief Configuration for error handling behavior
     */
    struct Config {
        int max_retry_attempts;
        std::chrono::milliseconds retry_delay;
        std::chrono::milliseconds retry_backoff_multiplier;
        bool enable_automatic_recovery;
        bool enable_service_degradation;
        size_t max_error_queue_size;
        std::chrono::minutes error_retention_time;
        
        Config() 
            : max_retry_attempts(3)
            , retry_delay(std::chrono::milliseconds(1000))
            , retry_backoff_multiplier(std::chrono::milliseconds(2))
            , enable_automatic_recovery(true)
            , enable_service_degradation(true)
            , max_error_queue_size(1000)
            , error_retention_time(std::chrono::minutes(60))
        {}
    };

    explicit ErrorRecoveryManager(const Config& config = Config());
    ~ErrorRecoveryManager();

    /**
     * @brief Handle an error with specified severity and context
     * @param error_code Unique identifier for the error type
     * @param severity Error severity level
     * @param context Additional context information
     * @param description Optional detailed description
     */
    void handleError(const std::string& error_code, ErrorSeverity severity,
                     const std::string& context, const std::string& description = "");

    /**
     * @brief Register a recovery strategy for a specific error code
     * @param error_code Error code to handle
     * @param recovery_func Function to execute for recovery
     */
    void registerRecoveryStrategy(const std::string& error_code,
                                  RecoveryStrategy recovery_func);

    /**
     * @brief Register a notification callback for error events
     * @param callback Function to call when errors occur
     */
    void registerNotificationCallback(ErrorNotificationCallback callback);

    /**
     * @brief Enable or disable service degradation mode
     * @param enabled Whether to enable degradation mode
     */
    void setServiceDegradationMode(bool enabled);

    /**
     * @brief Check if the system is in degraded mode
     * @return True if system is degraded
     */
    bool isInDegradedMode() const;

    /**
     * @brief Get error statistics
     * @return Map of error codes to occurrence counts
     */
    std::map<std::string, int> getErrorStatistics() const;

    /**
     * @brief Get recent error events
     * @param max_count Maximum number of events to return
     * @return Vector of recent error events
     */
    std::vector<ErrorEvent> getRecentErrors(size_t max_count = 100) const;

    /**
     * @brief Clear resolved errors from history
     */
    void clearResolvedErrors();

    /**
     * @brief Start the error processing thread
     */
    void start();

    /**
     * @brief Stop the error processing thread
     */
    void stop();

    /**
     * @brief Check if the manager is running
     */
    bool isRunning() const;

private:
    Config config_;
    std::map<std::string, RecoveryStrategy> recovery_strategies_;
    std::vector<ErrorNotificationCallback> notification_callbacks_;
    std::queue<ErrorEvent> error_queue_;
    std::vector<ErrorEvent> error_history_;
    std::map<std::string, int> error_statistics_;
    
    mutable std::mutex queue_mutex_;
    mutable std::mutex history_mutex_;
    mutable std::mutex stats_mutex_;
    mutable std::mutex callbacks_mutex_;
    
    std::atomic<bool> running_;
    std::atomic<bool> degraded_mode_;
    std::unique_ptr<std::thread> processing_thread_;
    
    /**
     * @brief Main error processing loop
     */
    void processErrors();
    
    /**
     * @brief Execute recovery strategy for an error
     * @param error Error event to recover from
     * @return True if recovery was successful
     */
    bool executeRecovery(ErrorEvent& error);
    
    /**
     * @brief Notify all registered callbacks about an error
     * @param error Error event to notify about
     */
    void notifyCallbacks(const ErrorEvent& error);
    
    /**
     * @brief Calculate retry delay with exponential backoff
     * @param retry_count Current retry attempt number
     * @return Delay duration
     */
    std::chrono::milliseconds calculateRetryDelay(int retry_count) const;
    
    /**
     * @brief Clean up old error events from history
     */
    void cleanupOldErrors();
    
    /**
     * @brief Determine if service degradation should be activated
     * @param error Error event that occurred
     * @return True if degradation should be activated
     */
    bool shouldActivateDegradation(const ErrorEvent& error) const;
};

/**
 * @brief Predefined error codes for common error types
 */
namespace ErrorCodes {
    // Communication errors
    constexpr const char* MAVLINK_CONNECTION_FAILED = "MAVLINK_CONNECTION_FAILED";
    constexpr const char* MAVLINK_TIMEOUT = "MAVLINK_TIMEOUT";
    constexpr const char* MAVLINK_PROTOCOL_ERROR = "MAVLINK_PROTOCOL_ERROR";
    
    // Authentication errors
    constexpr const char* CERTIFICATE_VALIDATION_FAILED = "CERTIFICATE_VALIDATION_FAILED";
    constexpr const char* TOKEN_EXPIRED = "TOKEN_EXPIRED";
    constexpr const char* AUTHENTICATION_TIMEOUT = "AUTHENTICATION_TIMEOUT";
    
    // Access control errors
    constexpr const char* POLICY_EVALUATION_FAILED = "POLICY_EVALUATION_FAILED";
    constexpr const char* SIGNATURE_VERIFICATION_FAILED = "SIGNATURE_VERIFICATION_FAILED";
    constexpr const char* PERMISSION_DENIED = "PERMISSION_DENIED";
    
    // Flight control errors
    constexpr const char* FLIGHT_COMMAND_FAILED = "FLIGHT_COMMAND_FAILED";
    constexpr const char* MISSION_UPLOAD_FAILED = "MISSION_UPLOAD_FAILED";
    constexpr const char* TRAJECTORY_DEVIATION = "TRAJECTORY_DEVIATION";
    constexpr const char* EMERGENCY_LANDING_REQUIRED = "EMERGENCY_LANDING_REQUIRED";
    
    // System errors
    constexpr const char* CONFIGURATION_ERROR = "CONFIGURATION_ERROR";
    constexpr const char* RESOURCE_EXHAUSTION = "RESOURCE_EXHAUSTION";
    constexpr const char* DATABASE_CONNECTION_FAILED = "DATABASE_CONNECTION_FAILED";
    
    // Security errors
    constexpr const char* SECURITY_VIOLATION = "SECURITY_VIOLATION";
    constexpr const char* UNAUTHORIZED_ACCESS_ATTEMPT = "UNAUTHORIZED_ACCESS_ATTEMPT";
    constexpr const char* INTRUSION_DETECTED = "INTRUSION_DETECTED";
}

} // namespace drone_control