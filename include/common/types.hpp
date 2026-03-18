#pragma once

/**
 * 公共类型定义
 * 定义全项目共用的基础类型、结构体与枚举（无人机 ID、位置、航点、飞行状态、访问级别等）。
 */
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <optional>

namespace drone_control {

/** 无人机 ID 类型 */
using DroneId = int;
/** 时间戳类型 */
using Timestamp = std::chrono::system_clock::time_point;
/** 时长类型（分钟） */
using Duration = std::chrono::minutes;

/** 地理坐标与高度 */
struct Position {
    double latitude;
    double longitude;
    double altitude;
    
    Position() : latitude(0.0), longitude(0.0), altitude(0.0) {}
    Position(double lat, double lon, double alt) 
        : latitude(lat), longitude(lon), altitude(alt) {}
};

// 航点信息
struct Waypoint {
    Position position;
    double speed_mps;
    std::chrono::seconds hold_time;
    std::string action;
    
    Waypoint() : speed_mps(5.0), hold_time(0) {}
};

// 飞行状态枚举
enum class FlightStatus {
    UNKNOWN,
    LANDED,
    TAKING_OFF,
    IN_AIR,
    LANDING,
    EMERGENCY
};

// 访问级别枚举
enum class AccessLevel {
    AUTHENTICATION,  // 身份认证级
    REGION,         // 区域访问级  
    BEHAVIOR        // 行为控制级
};

// 控制模式枚举
enum class ControlMode {
    OFFBOARD_DIRECT,  // 直接offboard控制
    MISSION_PLAN,     // 发送.plan文件
    GUIDED_WAYPOINTS  // 引导式航点
};

} // namespace drone_control