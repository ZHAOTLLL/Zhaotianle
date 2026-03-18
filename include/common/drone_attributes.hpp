/**
 * @file drone_attributes.hpp
 * @brief 定义无人机属性结构体，包含身份、环境、任务和动态属性信息，以及扩展的无人机状态信息。
 */
#pragma once

#include "types.hpp"
#include <string>
#include <vector>
#include <map>
#include <chrono>

namespace drone_control {

/**
 * 无人机属性结构体
 * 包含身份、环境、任务和动态属性信息
 */
struct DroneAttributes {
    // 身份属性
    std::string organization;        // 归属组织
    std::string role;               // 角色
    std::string certificate_id;     // 证书ID
    std::string trust_level;        // 信任级别
    
    // 环境属性
    Timestamp current_time;         // 当前时间
    Position position;              // 当前位置
    std::string weather_condition;  // 天气条件
    
    // 任务属性
    std::string mission_type;       // 任务类型
    std::string payload_type;       // 载荷类型
    std::vector<std::string> required_capabilities;  // 所需能力
    
    // 动态属性
    std::string flight_mode;        // 飞行模式
    double battery_level;           // 电池电量 (0.0-1.0)
    std::string emergency_status;   // 紧急状态
    
    DroneAttributes() 
        : current_time(std::chrono::system_clock::now())
        , battery_level(1.0)
        , emergency_status("normal") {}
};

/**
 * 扩展的无人机状态信息
 */
struct ExtendedDroneState {
    DroneId drone_id;
    Position position;
    FlightStatus flight_status;
    
    // 扩展信息
    std::string owner_organization;     // 归属单位
    std::string drone_model;           // 无人机型号
    std::string current_mission;       // 当前任务
    std::vector<std::string> capabilities;  // 能力列表
    std::chrono::system_clock::time_point last_update;  // 最后更新时间（用于对外展示）
    bool is_privileged;                // 是否为特权无人机
    
    // 认证状态信息
    bool is_authenticated;             // 是否已认证
    std::string authentication_type;   // 认证类型 (x509_certificate, jwt_token)
    std::string trust_level;           // 信任级别 (high, medium, low)
    std::string certificate_id;        // 证书ID
    std::chrono::system_clock::time_point auth_timestamp;  // 认证时间戳（用于对外展示）
    std::map<std::string, std::string> auth_attributes;    // 认证属性
    
    // 签名状态信息
    bool has_valid_signature;          // 是否有有效签名
    std::string signature_id;          // 签名ID
    std::string signature_region;      // 签名区域
    std::chrono::system_clock::time_point signature_expiry;  // 签名过期时间（用于对外展示）
    std::string signature_operation;   // 签名操作类型
    
    // 实时状态
    double battery_percentage;         // 电池百分比
    double ground_speed;              // 地面速度 (m/s)
    bool is_armed;                    // 是否解锁
    
    // 速度信息
    double velocity_north;            // 北向速度 (m/s)
    double velocity_east;             // 东向速度 (m/s)
    double velocity_down;             // 下向速度 (m/s)
    double speed;                     // 总速度 (m/s)
    double heading_deg;               // 航向角 (度)
    
    ExtendedDroneState() 
        : drone_id(0)
        , flight_status(FlightStatus::UNKNOWN)
        , last_update(std::chrono::system_clock::now())
        , is_privileged(false)
        , is_authenticated(false)
        , authentication_type("none")
        , trust_level("low")
        , has_valid_signature(false)
        , battery_percentage(100.0)
        , ground_speed(0.0)
        , is_armed(false)
        , velocity_north(0.0)
        , velocity_east(0.0)
        , velocity_down(0.0)
        , speed(0.0)
        , heading_deg(0.0) {}
};

} // namespace drone_control