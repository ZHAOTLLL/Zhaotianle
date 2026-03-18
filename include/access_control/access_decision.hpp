/**
 * 访问决策
 * 一次访问控制评估的输出：是否放行、访问级别、原因、飞行计划、性能统计等。
 */
#pragma once

#include "common/types.hpp"
#include "flight_control/flight_plan.hpp"
#include <string>
#include <vector>
#include <optional>
#include <chrono>

namespace drone_control {

/** 访问决策结果：授权与否、级别、原因、约束与可选飞行计划 */
struct AccessDecision {
    bool granted;                                    // 是否授权
    AccessLevel level;                              // 访问级别
    std::string reason;                             // 决策原因
    std::string geofence_signature;                 // 地理围栏签名
    std::vector<std::string> restricted_capabilities; // 受限能力
    std::vector<std::string> required_actions;       // 必需操作
    std::optional<FlightPlan> flight_plan;          // 飞行计划
    Duration validity_duration;                     // 有效期
    
    // 决策时间戳
    Timestamp decision_time;
    
    // 决策ID（用于追踪）
    std::string decision_id;
    
    // 策略信息
    std::vector<std::string> applied_policies;      // 应用的策略
    std::string policy_version;                     // 策略版本
    
    // 性能信息
    struct PerformanceMetrics {
        double total_evaluation_time_ms;            // 总评估时间（毫秒）
        double authentication_time_ms;              // 身份认证时间（毫秒）
        double region_access_time_ms;              // 区域访问时间（毫秒）
        double behavior_control_time_ms;            // 行为控制时间（毫秒）
        
        // 细粒度时间统计（按模块）
        // 第一级：身份认证阶段
        double dss_signature_verify_time_ms;        // DSS签名验证时间（毫秒）
        
        // 第二级：区域访问控制阶段
        double attribute_aggregation_time_ms;       // 属性聚合时间（毫秒）
        double policy_matching_time_ms;             // 策略匹配时间（毫秒）
        
        // 第三级：行为控制阶段
        double flight_plan_generation_time_ms;      // 飞行计划生成时间（毫秒）
        double geofence_sanitizing_time_ms;         // 地理围栏签名清洗时间（毫秒）
        double geofence_encryption_time_ms;         // 地理围栏加密时间（毫秒）
        double geofence_token_generation_time_ms;   // 地理围栏令牌生成总时间（清洗+加密，毫秒）
        
        PerformanceMetrics() 
            : total_evaluation_time_ms(0.0)
            , authentication_time_ms(0.0)
            , region_access_time_ms(0.0)
            , behavior_control_time_ms(0.0)
            , dss_signature_verify_time_ms(0.0)
            , attribute_aggregation_time_ms(0.0)
            , policy_matching_time_ms(0.0)
            , flight_plan_generation_time_ms(0.0)
            , geofence_sanitizing_time_ms(0.0)
            , geofence_encryption_time_ms(0.0)
            , geofence_token_generation_time_ms(0.0) {}
    } performance_metrics;
    
    AccessDecision() 
        : granted(false)
        , level(AccessLevel::AUTHENTICATION)
        , validity_duration(std::chrono::minutes(30))
        , decision_time(std::chrono::system_clock::now())
        , policy_version("1.0") {}
        
    AccessDecision(bool grant, AccessLevel lvl, const std::string& rsn)
        : granted(grant)
        , level(lvl)
        , reason(rsn)
        , validity_duration(std::chrono::minutes(30))
        , decision_time(std::chrono::system_clock::now())
        , policy_version("1.0") {}
};

} // namespace drone_control