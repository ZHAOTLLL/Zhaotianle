/**
 * security_violation_handler.hpp - 安全违规处理与审计
 */
#pragma once

#include "error/error_recovery_manager.hpp"
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>

namespace drone_control {

/** 安全违规严重程度 */
enum class SecurityViolationSeverity {
    LOW,        // 轻微违规 - Minor violation (e.g., minor protocol deviation)
    MEDIUM,     // 中等违规 - Medium violation (e.g., unauthorized area access attempt)
    HIGH,       // 严重违规 - Serious violation (e.g., certificate forgery attempt)
    CRITICAL    // 极严重违规 - Critical violation (e.g., active attack detected)
};

/** 安全违规类型 */
namespace SecurityViolationType {
    constexpr const char* UNAUTHORIZED_ACCESS = "UNAUTHORIZED_ACCESS";
    constexpr const char* CERTIFICATE_FORGERY = "CERTIFICATE_FORGERY";
    constexpr const char* SIGNATURE_TAMPERING = "SIGNATURE_TAMPERING";
    constexpr const char* GEOFENCE_VIOLATION = "GEOFENCE_VIOLATION";
    constexpr const char* COMMAND_INJECTION = "COMMAND_INJECTION";
    constexpr const char* REPLAY_ATTACK = "REPLAY_ATTACK";
    constexpr const char* PRIVILEGE_ESCALATION = "PRIVILEGE_ESCALATION";
    constexpr const char* DATA_EXFILTRATION = "DATA_EXFILTRATION";
    constexpr const char* MALICIOUS_PAYLOAD = "MALICIOUS_PAYLOAD";
    constexpr const char* INTRUSION_ATTEMPT = "INTRUSION_ATTEMPT";
}

/** 安全违规事件 */
struct SecurityViolationEvent {
    std::string violation_id;
    std::string violation_type;
    SecurityViolationSeverity severity;
    int drone_id;
    std::string source_ip;
    std::string description;
    std::map<std::string, std::string> evidence;
    std::chrono::steady_clock::time_point timestamp;
    bool resolved;
    std::string resolution_action;
    
    SecurityViolationEvent(const std::string& type, SecurityViolationSeverity sev, 
                          int drone, const std::string& desc)
        : violation_type(type), severity(sev), drone_id(drone), description(desc),
          timestamp(std::chrono::steady_clock::now()), resolved(false) {
        // Generate unique violation ID
        violation_id = type + "_" + std::to_string(drone) + "_" + 
                      std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                          timestamp.time_since_epoch()).count());
    }
};

/** 紧急响应动作类型 */
enum class EmergencyAction {
    LOG_ONLY,           // 仅记录 - Only log the event
    WARN_OPERATOR,      // 警告操作员 - Warn the operator
    RESTRICT_ACCESS,    // 限制访问 - Restrict access permissions
    ABORT_FLIGHT,       // 中止飞行 - Abort current flight
    EMERGENCY_LANDING,  // 紧急降落 - Force emergency landing
    DISCONNECT_DRONE,   // 断开连接 - Disconnect the drone
    SYSTEM_LOCKDOWN     // 系统锁定 - Lock down the entire system
};

/** 安全审计记录 */
struct SecurityAuditEntry {
    std::string audit_id;
    std::chrono::steady_clock::time_point timestamp;
    std::string event_type;
    int drone_id;
    std::string user_id;
    std::string action;
    std::string result;
    std::map<std::string, std::string> details;
    
    SecurityAuditEntry(const std::string& type, int drone, const std::string& user,
                      const std::string& act, const std::string& res)
        : event_type(type), drone_id(drone), user_id(user), action(act), result(res),
          timestamp(std::chrono::steady_clock::now()) {
        audit_id = "AUDIT_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                      timestamp.time_since_epoch()).count());
    }
};

/**
 * @brief Security violation response callback type
 */
using SecurityResponseCallback = std::function<void(const SecurityViolationEvent&, EmergencyAction)>;

/**
 * @brief Flight abort callback type
 */
using FlightAbortCallback = std::function<bool(int drone_id, const std::string& reason)>;

/**
 * @brief Emergency landing callback type
 */
using EmergencyLandingCallback = std::function<bool(int drone_id, const std::string& reason)>;

class SecurityViolationHandler {
public:
    /**
     * @brief Configuration for security violation handling
     */
    struct Config {
        bool enable_automatic_response;
        bool enable_emergency_actions;
        std::chrono::minutes audit_retention_time;
        size_t max_audit_entries;
        bool enable_real_time_alerts;
        std::string audit_log_file;
        
        Config()
            : enable_automatic_response(true)
            , enable_emergency_actions(true)
            , audit_retention_time(std::chrono::minutes(1440)) // 24 hours
            , max_audit_entries(10000)
            , enable_real_time_alerts(true)
            , audit_log_file("security_audit.log")
        {}
    };

    explicit SecurityViolationHandler(const Config& config = Config());
    ~SecurityViolationHandler();

    /**
     * @brief Report a security violation
     * @param violation_type Type of security violation
     * @param severity Severity level of the violation
     * @param drone_id ID of the drone involved
     * @param description Detailed description of the violation
     * @param evidence Additional evidence data
     */
    void reportViolation(const std::string& violation_type, 
                        SecurityViolationSeverity severity,
                        int drone_id,
                        const std::string& description,
                        const std::map<std::string, std::string>& evidence = {});

    /**
     * @brief Register a security response callback
     * @param callback Function to call when security violations occur
     */
    void registerResponseCallback(SecurityResponseCallback callback);

    /**
     * @brief Register flight abort callback
     * @param callback Function to call for flight abort operations
     */
    void registerFlightAbortCallback(FlightAbortCallback callback);

    /**
     * @brief Register emergency landing callback
     * @param callback Function to call for emergency landing operations
     */
    void registerEmergencyLandingCallback(EmergencyLandingCallback callback);

    /**
     * @brief Manually trigger emergency flight abort
     * @param drone_id ID of the drone to abort
     * @param reason Reason for the abort
     * @return True if abort was successful
     */
    bool triggerEmergencyAbort(int drone_id, const std::string& reason);

    /**
     * @brief Manually trigger emergency landing
     * @param drone_id ID of the drone to land
     * @param reason Reason for the emergency landing
     * @return True if landing was initiated successfully
     */
    bool triggerEmergencyLanding(int drone_id, const std::string& reason);

    /**
     * @brief Add security audit entry
     * @param event_type Type of event being audited
     * @param drone_id ID of the drone involved
     * @param user_id ID of the user involved
     * @param action Action that was performed
     * @param result Result of the action
     * @param details Additional details
     */
    void addAuditEntry(const std::string& event_type, int drone_id, 
                      const std::string& user_id, const std::string& action,
                      const std::string& result, 
                      const std::map<std::string, std::string>& details = {});

    /**
     * @brief Get security violation statistics
     * @return Map of violation types to occurrence counts
     */
    std::map<std::string, int> getViolationStatistics() const;

    /**
     * @brief Get recent security violations
     * @param max_count Maximum number of violations to return
     * @return Vector of recent violations
     */
    std::vector<SecurityViolationEvent> getRecentViolations(size_t max_count = 100) const;

    /**
     * @brief Get security audit log
     * @param start_time Start time for audit query
     * @param end_time End time for audit query
     * @return Vector of audit entries in the time range
     */
    std::vector<SecurityAuditEntry> getAuditLog(
        const std::chrono::steady_clock::time_point& start_time,
        const std::chrono::steady_clock::time_point& end_time) const;

    /**
     * @brief Generate security report
     * @param start_time Start time for the report
     * @param end_time End time for the report
     * @return Formatted security report string
     */
    std::string generateSecurityReport(
        const std::chrono::steady_clock::time_point& start_time,
        const std::chrono::steady_clock::time_point& end_time) const;

    /**
     * @brief Clear resolved violations from history
     */
    void clearResolvedViolations();

    /**
     * @brief Set system lockdown mode
     * @param enabled Whether to enable lockdown mode
     */
    void setSystemLockdown(bool enabled);

    /**
     * @brief Check if system is in lockdown mode
     * @return True if system is locked down
     */
    bool isSystemLocked() const;

    /**
     * @brief Initialize integration with error recovery manager
     * @param error_manager Pointer to the error recovery manager
     */
    void integrateWithErrorManager(std::shared_ptr<ErrorRecoveryManager> error_manager);

private:
    Config config_;
    std::vector<SecurityViolationEvent> violation_history_;
    std::vector<SecurityAuditEntry> audit_log_;
    std::map<std::string, int> violation_statistics_;
    std::vector<SecurityResponseCallback> response_callbacks_;
    FlightAbortCallback flight_abort_callback_;
    EmergencyLandingCallback emergency_landing_callback_;
    std::shared_ptr<ErrorRecoveryManager> error_manager_;
    
    mutable std::mutex violation_mutex_;
    mutable std::mutex audit_mutex_;
    mutable std::mutex stats_mutex_;
    mutable std::mutex callbacks_mutex_;
    std::atomic<bool> system_locked_;
    
    /**
     * @brief Determine appropriate emergency action for a violation
     * @param violation The security violation event
     * @return Recommended emergency action
     */
    EmergencyAction determineEmergencyAction(const SecurityViolationEvent& violation) const;
    
    /**
     * @brief Execute emergency response action
     * @param violation The security violation event
     * @param action The emergency action to execute
     */
    void executeEmergencyAction(const SecurityViolationEvent& violation, EmergencyAction action);
    
    /**
     * @brief Notify all registered response callbacks
     * @param violation The security violation event
     * @param action The emergency action being taken
     */
    void notifyResponseCallbacks(const SecurityViolationEvent& violation, EmergencyAction action);
    
    /**
     * @brief Write audit entry to log file
     * @param entry The audit entry to write
     */
    void writeAuditToFile(const SecurityAuditEntry& entry);
    
    /**
     * @brief Clean up old audit entries
     */
    void cleanupOldAuditEntries();
    
    /**
     * @brief Format timestamp for logging
     * @param timestamp The timestamp to format
     * @return Formatted timestamp string
     */
    std::string formatTimestamp(const std::chrono::steady_clock::time_point& timestamp) const;
};

} // namespace drone_control