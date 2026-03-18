/**
 * 访问决策生成
 * 根据策略评估结果生成访问决策结构（授权、级别、原因、飞行计划等）。
 */
#include "access_control/access_decision_generator.hpp"
#include <chrono>
#include <sstream>
#include <algorithm>

namespace drone_control {

AccessDecisionGenerator::AccessDecisionGenerator() = default;

AccessDecisionGenerator::~AccessDecisionGenerator() = default;

AccessDecision AccessDecisionGenerator::generateDecision(const AccessRequest& request,
                                                        bool policy_granted,
                                                        const std::string& policy_reason,
                                                        const std::map<std::string, std::string>& attributes,
                                                        AccessLevel level) {
    AccessDecision decision;
    decision.granted = policy_granted;
    decision.level = level;
    decision.decision_time = std::chrono::system_clock::now();
    
    // 生成详细理由
    decision.reason = generateDetailedReason(policy_granted, attributes, policy_reason);
    
    if (policy_granted) {
        // 生成约束和限制
        decision.restricted_capabilities = generateBehaviorRestrictions(attributes, request.target_location);
        decision.required_actions = generateRequiredActions(attributes, request.operation_type);
        
        // 计算有效期
        decision.validity_duration = calculateValidityDuration(attributes, request.operation_type);
        
        // 记录应用的策略
        decision.applied_policies.push_back("policy_" + std::to_string(static_cast<int>(level)));
    } else {
        // 拒绝访问时提供改进建议
        decision.required_actions = generateImprovementSuggestions(attributes, policy_reason);
    }
    
    return decision;
}

std::vector<std::string> AccessDecisionGenerator::generateBehaviorRestrictions(
    const std::map<std::string, std::string>& attributes,
    const std::string& target_location) {
    
    std::vector<std::string> restrictions;
    
    // 基于目标位置的敏感级别设置限制
    auto sensitivity_it = attributes.find("location_sensitivity");
    if (sensitivity_it != attributes.end()) {
        if (sensitivity_it->second == "high") {
            restrictions.push_back("no_photography");
            restrictions.push_back("no_audio_recording");
            restrictions.push_back("maintain_minimum_altitude_100m");
            restrictions.push_back("no_loitering");
        } else if (sensitivity_it->second == "medium") {
            restrictions.push_back("limited_photography");
            restrictions.push_back("maintain_minimum_altitude_50m");
        }
    }
    
    // 基于任务类型设置限制
    auto mission_it = attributes.find("mission_type");
    if (mission_it != attributes.end()) {
        if (mission_it->second == "delivery") {
            restrictions.push_back("no_route_deviation");
            restrictions.push_back("direct_flight_only");
        } else if (mission_it->second == "surveillance") {
            restrictions.push_back("log_all_surveillance_activities");
            restrictions.push_back("privacy_compliance_required");
        }
    }
    
    // 基于载荷类型设置限制
    auto payload_it = attributes.find("payload_type");
    if (payload_it != attributes.end()) {
        if (payload_it->second == "hazardous") {
            restrictions.push_back("no_flight_over_populated_areas");
            restrictions.push_back("emergency_landing_plan_required");
        } else if (payload_it->second == "camera") {
            restrictions.push_back("privacy_zones_avoidance");
        }
    }
    
    // 基于天气条件设置限制
    auto weather_it = attributes.find("weather_condition");
    if (weather_it != attributes.end()) {
        if (weather_it->second == "windy") {
            restrictions.push_back("reduced_speed_limit");
            restrictions.push_back("lower_altitude_limit");
        } else if (weather_it->second == "rain") {
            restrictions.push_back("waterproof_equipment_required");
            restrictions.push_back("reduced_flight_duration");
        }
    }
    
    return restrictions;
}

std::vector<std::string> AccessDecisionGenerator::generateRequiredActions(
    const std::map<std::string, std::string>& attributes,
    const std::string& operation_type) {
    
    std::vector<std::string> actions;
    
    // 基于组织类型的必需操作
    auto org_it = attributes.find("organization");
    if (org_it != attributes.end()) {
        if (org_it->second == "government" || org_it->second == "police") {
            actions.push_back("maintain_official_flight_log");
            actions.push_back("enable_priority_communication_channel");
        } else if (org_it->second == "commercial") {
            actions.push_back("verify_commercial_insurance");
            actions.push_back("comply_with_commercial_regulations");
        }
    }
    
    // 基于操作类型的必需操作
    if (operation_type == "emergency") {
        actions.push_back("continuous_position_reporting");
        actions.push_back("maintain_emergency_communication");
    } else if (operation_type == "delivery") {
        actions.push_back("confirm_delivery_completion");
        actions.push_back("update_delivery_status");
    } else if (operation_type == "inspection") {
        actions.push_back("record_inspection_data");
        actions.push_back("submit_inspection_report");
    }
    
    // 基于电池状态的必需操作
    auto battery_it = attributes.find("battery_level");
    if (battery_it != attributes.end()) {
        double battery_level = std::stod(battery_it->second);
        if (battery_level < 0.3) {
            actions.push_back("return_to_base_immediately");
        } else if (battery_level < 0.5) {
            actions.push_back("plan_return_route");
        }
    }
    
    // 时间相关的必需操作
    auto current_hour = std::chrono::duration_cast<std::chrono::hours>(
        std::chrono::system_clock::now().time_since_epoch()).count() % 24;
    
    if (current_hour < 6 || current_hour > 22) { // 夜间飞行
        actions.push_back("enable_navigation_lights");
        actions.push_back("reduce_flight_speed");
    }
    
    return actions;
}

std::string AccessDecisionGenerator::generateDetailedReason(bool granted,
                                                           const std::map<std::string, std::string>& attributes,
                                                           const std::string& policy_reason) {
    std::stringstream reason;
    
    if (granted) {
        reason << "Access granted based on policy evaluation. ";
        
        // 添加基于属性的详细说明
        auto org_it = attributes.find("organization");
        if (org_it != attributes.end()) {
            reason << "Organization: " << org_it->second << ". ";
        }
        
        auto trust_it = attributes.find("trust_level");
        if (trust_it != attributes.end()) {
            reason << "Trust level: " << trust_it->second << ". ";
        }
        
        auto mission_it = attributes.find("mission_type");
        if (mission_it != attributes.end()) {
            reason << "Mission type: " << mission_it->second << ". ";
        }
        
        reason << policy_reason;
    } else {
        reason << "Access denied: " << policy_reason << ". ";
        
        // 提供改进建议
        auto org_it = attributes.find("organization");
        if (org_it == attributes.end() || org_it->second.empty()) {
            reason << "Suggestion: Provide valid organization credentials. ";
        }
        
        auto cert_it = attributes.find("certificate_id");
        if (cert_it == attributes.end() || cert_it->second.empty()) {
            reason << "Suggestion: Provide valid authentication certificate. ";
        }
        
        auto weather_it = attributes.find("weather_condition");
        if (weather_it != attributes.end() && weather_it->second == "severe") {
            reason << "Suggestion: Wait for weather conditions to improve. ";
        }
    }
    
    return reason.str();
}

Duration AccessDecisionGenerator::calculateValidityDuration(
    const std::map<std::string, std::string>& attributes,
    const std::string& operation_type) {
    
    Duration base_duration = std::chrono::minutes(30); // 默认30分钟
    
    // 基于信任级别调整有效期
    auto trust_it = attributes.find("trust_level");
    if (trust_it != attributes.end()) {
        if (trust_it->second == "high") {
            base_duration = std::chrono::minutes(120); // 高信任2小时
        } else if (trust_it->second == "medium") {
            base_duration = std::chrono::minutes(60);  // 中等信任1小时
        }
    }
    
    // 基于操作类型调整有效期
    if (operation_type == "emergency") {
        base_duration = std::chrono::minutes(15); // 紧急任务短期有效
    } else if (operation_type == "delivery") {
        base_duration = std::chrono::minutes(45); // 配送任务适中
    } else if (operation_type == "inspection") {
        base_duration = std::chrono::minutes(90); // 巡检任务较长
    }
    
    // 基于电池状态调整有效期
    auto battery_it = attributes.find("battery_level");
    if (battery_it != attributes.end()) {
        double battery_level = std::stod(battery_it->second);
        if (battery_level < 0.5) {
            // 电量不足时缩短有效期
            base_duration = std::chrono::minutes(
                static_cast<int>(base_duration.count() * battery_level));
        }
    }
    
    return base_duration;
}

std::vector<std::string> AccessDecisionGenerator::generateImprovementSuggestions(
    const std::map<std::string, std::string>& attributes,
    const std::string& policy_reason) {
    
    std::vector<std::string> suggestions;
    
    // 基于拒绝原因提供具体建议
    if (policy_reason.find("authentication") != std::string::npos) {
        suggestions.push_back("provide_valid_authentication_certificate");
        suggestions.push_back("verify_certificate_not_expired");
    }
    
    if (policy_reason.find("weather") != std::string::npos) {
        suggestions.push_back("wait_for_weather_improvement");
        suggestions.push_back("reschedule_flight_operation");
    }
    
    if (policy_reason.find("time") != std::string::npos) {
        suggestions.push_back("reschedule_to_permitted_hours");
        suggestions.push_back("apply_for_special_time_permission");
    }
    
    if (policy_reason.find("location") != std::string::npos) {
        suggestions.push_back("choose_alternative_location");
        suggestions.push_back("apply_for_special_location_permission");
    }
    
    // 基于属性缺失提供建议
    if (attributes.find("organization") == attributes.end()) {
        suggestions.push_back("provide_organization_credentials");
    }
    
    if (attributes.find("mission_type") == attributes.end()) {
        suggestions.push_back("specify_mission_type");
    }
    
    return suggestions;
}

} // namespace drone_control