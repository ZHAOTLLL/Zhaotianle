/**
 * 安全违规处理与审计
 * 记录违规事件、严重程度与响应动作，支持审计日志输出。
 */
#include "error/security_violation_handler.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>

namespace drone_control {

SecurityViolationHandler::SecurityViolationHandler(const Config& config)
    : config_(config), system_locked_(false) {
}

SecurityViolationHandler::~SecurityViolationHandler() {
}

void SecurityViolationHandler::reportViolation(const std::string& violation_type, 
                                              SecurityViolationSeverity severity,
                                              int drone_id,
                                              const std::string& description,
                                              const std::map<std::string, std::string>& evidence) {
    SecurityViolationEvent violation(violation_type, severity, drone_id, description);
    violation.evidence = evidence;
    
    // Update statistics
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        violation_statistics_[violation_type]++;
    }
    
    // Add to violation history
    {
        std::lock_guard<std::mutex> lock(violation_mutex_);
        violation_history_.push_back(violation);
        
        // Limit history size
        if (violation_history_.size() > config_.max_audit_entries) {
            violation_history_.erase(violation_history_.begin());
        }
    }
    
    // Add audit entry
    addAuditEntry("SECURITY_VIOLATION", drone_id, "SYSTEM", 
                 "VIOLATION_DETECTED", "REPORTED", {
                     {"violation_type", violation_type},
                     {"severity", std::to_string(static_cast<int>(severity))},
                     {"description", description}
                 });
    
    // Determine and execute emergency action
    EmergencyAction action = determineEmergencyAction(violation);
    executeEmergencyAction(violation, action);
    
    // Notify callbacks
    notifyResponseCallbacks(violation, action);
    
    // Report to error recovery manager if integrated
    if (error_manager_) {
        ErrorSeverity error_severity;
        switch (severity) {
            case SecurityViolationSeverity::LOW:
                error_severity = ErrorSeverity::LOW;
                break;
            case SecurityViolationSeverity::MEDIUM:
                error_severity = ErrorSeverity::MEDIUM;
                break;
            case SecurityViolationSeverity::HIGH:
                error_severity = ErrorSeverity::HIGH;
                break;
            case SecurityViolationSeverity::CRITICAL:
                error_severity = ErrorSeverity::CRITICAL;
                break;
        }
        
        error_manager_->handleError(ErrorCodes::SECURITY_VIOLATION, error_severity,
                                   "Security violation: " + violation_type + " on drone " + std::to_string(drone_id),
                                   description);
    }
}

void SecurityViolationHandler::registerResponseCallback(SecurityResponseCallback callback) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    response_callbacks_.push_back(callback);
}

void SecurityViolationHandler::registerFlightAbortCallback(FlightAbortCallback callback) {
    flight_abort_callback_ = callback;
}

void SecurityViolationHandler::registerEmergencyLandingCallback(EmergencyLandingCallback callback) {
    emergency_landing_callback_ = callback;
}

bool SecurityViolationHandler::triggerEmergencyAbort(int drone_id, const std::string& reason) {
    addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "FLIGHT_ABORT", "INITIATED", {
        {"reason", reason}
    });
    
    if (flight_abort_callback_) {
        bool success = flight_abort_callback_(drone_id, reason);
        
        addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "FLIGHT_ABORT", 
                     success ? "SUCCESS" : "FAILED", {
                         {"reason", reason}
                     });
        
        return success;
    } else {
        addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "FLIGHT_ABORT", "NO_CALLBACK", {
            {"reason", reason}
        });
        return false;
    }
}

bool SecurityViolationHandler::triggerEmergencyLanding(int drone_id, const std::string& reason) {
    addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "EMERGENCY_LANDING", "INITIATED", {
        {"reason", reason}
    });
    
    if (emergency_landing_callback_) {
        bool success = emergency_landing_callback_(drone_id, reason);
        
        addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "EMERGENCY_LANDING", 
                     success ? "SUCCESS" : "FAILED", {
                         {"reason", reason}
                     });
        
        return success;
    } else {
        addAuditEntry("EMERGENCY_ACTION", drone_id, "SYSTEM", "EMERGENCY_LANDING", "NO_CALLBACK", {
            {"reason", reason}
        });
        return false;
    }
}

void SecurityViolationHandler::addAuditEntry(const std::string& event_type, int drone_id, 
                                           const std::string& user_id, const std::string& action,
                                           const std::string& result, 
                                           const std::map<std::string, std::string>& details) {
    SecurityAuditEntry entry(event_type, drone_id, user_id, action, result);
    entry.details = details;
    
    {
        std::lock_guard<std::mutex> lock(audit_mutex_);
        audit_log_.push_back(entry);
        
        // Limit audit log size
        if (audit_log_.size() > config_.max_audit_entries) {
            audit_log_.erase(audit_log_.begin());
        }
    }
    
    // Write to file if configured
    if (!config_.audit_log_file.empty()) {
        writeAuditToFile(entry);
    }
    
    // Periodic cleanup
    static auto last_cleanup = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (now - last_cleanup > std::chrono::minutes(10)) {
        cleanupOldAuditEntries();
        last_cleanup = now;
    }
}

std::map<std::string, int> SecurityViolationHandler::getViolationStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return violation_statistics_;
}

std::vector<SecurityViolationEvent> SecurityViolationHandler::getRecentViolations(size_t max_count) const {
    std::lock_guard<std::mutex> lock(violation_mutex_);
    
    std::vector<SecurityViolationEvent> recent_violations;
    size_t start_index = violation_history_.size() > max_count ? 
                        violation_history_.size() - max_count : 0;
    
    for (size_t i = start_index; i < violation_history_.size(); ++i) {
        recent_violations.push_back(violation_history_[i]);
    }
    
    return recent_violations;
}

std::vector<SecurityAuditEntry> SecurityViolationHandler::getAuditLog(
    const std::chrono::steady_clock::time_point& start_time,
    const std::chrono::steady_clock::time_point& end_time) const {
    
    std::lock_guard<std::mutex> lock(audit_mutex_);
    
    std::vector<SecurityAuditEntry> filtered_log;
    
    for (const auto& entry : audit_log_) {
        if (entry.timestamp >= start_time && entry.timestamp <= end_time) {
            filtered_log.push_back(entry);
        }
    }
    
    return filtered_log;
}

std::string SecurityViolationHandler::generateSecurityReport(
    const std::chrono::steady_clock::time_point& start_time,
    const std::chrono::steady_clock::time_point& end_time) const {
    
    std::ostringstream report;
    
    report << "=== SECURITY REPORT ===\n";
    report << "Report Period: " << formatTimestamp(start_time) 
           << " to " << formatTimestamp(end_time) << "\n\n";
    
    // Violation statistics
    auto stats = getViolationStatistics();
    report << "VIOLATION STATISTICS:\n";
    int total_violations = 0;
    for (const auto& [type, count] : stats) {
        report << "  " << type << ": " << count << "\n";
        total_violations += count;
    }
    report << "  TOTAL: " << total_violations << "\n\n";
    
    // Recent violations
    auto violations = getRecentViolations(20);
    report << "RECENT VIOLATIONS (last 20):\n";
    for (const auto& violation : violations) {
        if (violation.timestamp >= start_time && violation.timestamp <= end_time) {
            report << "  [" << formatTimestamp(violation.timestamp) << "] "
                   << violation.violation_type << " (Drone " << violation.drone_id 
                   << ", Severity " << static_cast<int>(violation.severity) << "): "
                   << violation.description << "\n";
        }
    }
    report << "\n";
    
    // Audit summary
    auto audit_entries = getAuditLog(start_time, end_time);
    report << "AUDIT SUMMARY:\n";
    report << "  Total audit entries: " << audit_entries.size() << "\n";
    
    std::map<std::string, int> action_counts;
    for (const auto& entry : audit_entries) {
        action_counts[entry.action]++;
    }
    
    report << "  Actions performed:\n";
    for (const auto& [action, count] : action_counts) {
        report << "    " << action << ": " << count << "\n";
    }
    
    report << "\n=== END REPORT ===\n";
    
    return report.str();
}

void SecurityViolationHandler::clearResolvedViolations() {
    std::lock_guard<std::mutex> lock(violation_mutex_);
    
    auto it = std::remove_if(violation_history_.begin(), violation_history_.end(),
                            [](const SecurityViolationEvent& violation) { 
                                return violation.resolved; 
                            });
    
    violation_history_.erase(it, violation_history_.end());
}

void SecurityViolationHandler::setSystemLockdown(bool enabled) {
    system_locked_.store(enabled);
    
    if (enabled) {
        addAuditEntry("SYSTEM_CONTROL", -1, "SYSTEM", "LOCKDOWN_ACTIVATED", "SUCCESS");
    } else {
        addAuditEntry("SYSTEM_CONTROL", -1, "SYSTEM", "LOCKDOWN_DEACTIVATED", "SUCCESS");
    }
}

bool SecurityViolationHandler::isSystemLocked() const {
    return system_locked_.load();
}

void SecurityViolationHandler::integrateWithErrorManager(std::shared_ptr<ErrorRecoveryManager> error_manager) {
    error_manager_ = error_manager;
    
    // Register security violation recovery strategies with the error manager
    if (error_manager_) {
        error_manager_->registerRecoveryStrategy(ErrorCodes::SECURITY_VIOLATION,
            [this](const ErrorEvent& error) -> bool {
                return true;
            });
        
        error_manager_->registerRecoveryStrategy(ErrorCodes::UNAUTHORIZED_ACCESS_ATTEMPT,
            [this](const ErrorEvent& error) -> bool {
                addAuditEntry("ERROR_RECOVERY", -1, "SYSTEM", "UNAUTHORIZED_ACCESS_RECOVERY", "HANDLED", {
                    {"error_context", error.context}
                });
                return true;
            });
        
        error_manager_->registerRecoveryStrategy(ErrorCodes::INTRUSION_DETECTED,
            [this](const ErrorEvent& error) -> bool {
                setSystemLockdown(true);
                addAuditEntry("ERROR_RECOVERY", -1, "SYSTEM", "INTRUSION_RESPONSE", "LOCKDOWN_ACTIVATED", {
                    {"error_context", error.context}
                });
                return true;
            });
    }
}

EmergencyAction SecurityViolationHandler::determineEmergencyAction(const SecurityViolationEvent& violation) const {
    // If system is locked, only allow logging
    if (system_locked_.load()) {
        return EmergencyAction::LOG_ONLY;
    }
    
    // Determine action based on violation type and severity
    switch (violation.severity) {
        case SecurityViolationSeverity::LOW:
            return EmergencyAction::LOG_ONLY;
            
        case SecurityViolationSeverity::MEDIUM:
            if (violation.violation_type == SecurityViolationType::UNAUTHORIZED_ACCESS ||
                violation.violation_type == SecurityViolationType::GEOFENCE_VIOLATION) {
                return EmergencyAction::RESTRICT_ACCESS;
            }
            return EmergencyAction::WARN_OPERATOR;
            
        case SecurityViolationSeverity::HIGH:
            if (violation.violation_type == SecurityViolationType::CERTIFICATE_FORGERY ||
                violation.violation_type == SecurityViolationType::SIGNATURE_TAMPERING ||
                violation.violation_type == SecurityViolationType::PRIVILEGE_ESCALATION) {
                return EmergencyAction::ABORT_FLIGHT;
            }
            return EmergencyAction::RESTRICT_ACCESS;
            
        case SecurityViolationSeverity::CRITICAL:
            if (violation.violation_type == SecurityViolationType::INTRUSION_ATTEMPT ||
                violation.violation_type == SecurityViolationType::MALICIOUS_PAYLOAD ||
                violation.violation_type == SecurityViolationType::COMMAND_INJECTION) {
                return EmergencyAction::SYSTEM_LOCKDOWN;
            }
            return EmergencyAction::EMERGENCY_LANDING;
    }
    
    return EmergencyAction::LOG_ONLY;
}

void SecurityViolationHandler::executeEmergencyAction(const SecurityViolationEvent& violation, EmergencyAction action) {
    if (!config_.enable_emergency_actions && action != EmergencyAction::LOG_ONLY) {
        return;
    }
    
    switch (action) {
        case EmergencyAction::LOG_ONLY:
            break;
            
        case EmergencyAction::WARN_OPERATOR:
            addAuditEntry("EMERGENCY_ACTION", violation.drone_id, "SYSTEM", "WARN_OPERATOR", "EXECUTED");
            break;
            
        case EmergencyAction::RESTRICT_ACCESS:
            addAuditEntry("EMERGENCY_ACTION", violation.drone_id, "SYSTEM", "RESTRICT_ACCESS", "EXECUTED");
            break;
            
        case EmergencyAction::ABORT_FLIGHT:
            triggerEmergencyAbort(violation.drone_id, "Security violation: " + violation.violation_type);
            break;
            
        case EmergencyAction::EMERGENCY_LANDING:
            triggerEmergencyLanding(violation.drone_id, "Security violation: " + violation.violation_type);
            break;
            
        case EmergencyAction::DISCONNECT_DRONE:
            addAuditEntry("EMERGENCY_ACTION", violation.drone_id, "SYSTEM", "DISCONNECT_DRONE", "EXECUTED");
            break;
            
        case EmergencyAction::SYSTEM_LOCKDOWN:
            setSystemLockdown(true);
            break;
    }
}

void SecurityViolationHandler::notifyResponseCallbacks(const SecurityViolationEvent& violation, EmergencyAction action) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    
    for (const auto& callback : response_callbacks_) {
        try {
            callback(violation, action);
        } catch (const std::exception& e) {
            // Exception in callback, silently continue
        }
    }
}

void SecurityViolationHandler::writeAuditToFile(const SecurityAuditEntry& entry) {
    try {
        std::ofstream audit_file(config_.audit_log_file, std::ios::app);
        if (audit_file.is_open()) {
            audit_file << formatTimestamp(entry.timestamp) << "|"
                      << entry.audit_id << "|"
                      << entry.event_type << "|"
                      << entry.drone_id << "|"
                      << entry.user_id << "|"
                      << entry.action << "|"
                      << entry.result;
            
            // Add details
            for (const auto& [key, value] : entry.details) {
                audit_file << "|" << key << "=" << value;
            }
            
            audit_file << std::endl;
        }
    } catch (const std::exception& e) {
        // Failed to write audit entry, silently continue
    }
}

void SecurityViolationHandler::cleanupOldAuditEntries() {
    std::lock_guard<std::mutex> lock(audit_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto cutoff_time = now - config_.audit_retention_time;
    
    auto it = std::remove_if(audit_log_.begin(), audit_log_.end(),
                            [cutoff_time](const SecurityAuditEntry& entry) {
                                return entry.timestamp < cutoff_time;
                            });
    
    audit_log_.erase(it, audit_log_.end());
}

std::string SecurityViolationHandler::formatTimestamp(const std::chrono::steady_clock::time_point& timestamp) const {
    auto time_t = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now() + 
        (timestamp - std::chrono::steady_clock::now()));
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

} // namespace drone_control