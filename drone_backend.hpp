#ifndef DRONE_BACKEND_HPP
#define DRONE_BACKEND_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <algorithm>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>

#include <zmq.hpp>  // ZeroMQ C++绑定

#include <openssl/pem.h>
#include <openssl/err.h>
#include <openssl/rsa.h>
#include <openssl/sha.h>

// 飞行模式转换为字符串
std::string flight_mode_to_string(mavsdk::Telemetry::FlightMode mode) {
        switch(mode) {
        case Telemetry::FlightMode::Unknown:          return "Unknown";
        case Telemetry::FlightMode::Ready:           return "Ready";
        case Telemetry::FlightMode::Takeoff:         return "Takeoff";
        case Telemetry::FlightMode::Hold:            return "Hold";
        case Telemetry::FlightMode::Mission:         return "Mission";
        case Telemetry::FlightMode::ReturnToLaunch:  return "ReturnToLaunch";
        case Telemetry::FlightMode::Land:            return "Land";
        case Telemetry::FlightMode::FollowMe:        return "FollowMe";
        case Telemetry::FlightMode::Offboard:        return "Offboard";
        case Telemetry::FlightMode::Altctl:          return "Altctl";
        case Telemetry::FlightMode::Posctl:          return "Posctl";
        case Telemetry::FlightMode::Manual:          return "Manual";
        default: return "Unknown";
    }
}

// 连接结果转换为字符串
std::string connection_result_to_string(mavsdk::ConnectionResult result) {
    switch(result) {
        case ConnectionResult::Success: return "Success";
        case ConnectionResult::Timeout: return "Timeout";
        case ConnectionResult::SocketError: return "SocketError";
        case ConnectionResult::BindError: return "BindError";
        case ConnectionResult::ConnectionError: return "ConnectionError";
        case ConnectionResult::NotImplemented: return "NotImplemented";
        default: return "Unknown";
    }
}

// 无人机状态结构体（从原代码复制）
struct DroneState {
    int system_id = -1;
    std::string flight_mode = "Unknown";
    struct Position {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
    } position;
    double absolute_altitude = 0.0;
    double relative_altitude = 0.0;
};

// 航点请求结构体（代替WaypointRequest msg）
struct WaypointRequest {
    std::string request_id;
    std::string drone_id;
    std::string operation;  // e.g., "read", "write", "delete"
    std::string data;       // 请求数据
    std::string signature;  // 签名
    std::map<std::string, std::string> attributes;  // 属性
};

// 访问响应结构体（代替AccessResponse msg）
struct AccessResponse {
    std::string request_id;
    std::string drone_id;
    bool access_granted;
    std::string reason;
    std::string timestamp;  // 可以用std::time生成
};

// 访问控制策略基类（从原代码复制）
class AccessControlStrategy {
public:
    virtual ~AccessControlStrategy() = default;
    virtual bool checkAccess(const std::string& drone_id, 
                             const std::string& resource, 
                             const std::map<std::string, std::string>& attributes,
                             const DroneState& drone_state) = 0;
};

// RBACStrategy（从原代码复制，略微调整日志为std::cout）
class RBACStrategy : public AccessControlStrategy {
private:
    std::map<std::string, std::vector<std::string>> role_permissions;
    std::map<std::string, std::string> drone_roles;

public:
    RBACStrategy() {
        // ... (保持原样)
    }
    bool checkAccess(const std::string& drone_id, 
                     const std::string& resource, 
                     const std::map<std::string, std::string>& attributes,
                     const DroneState& drone_state) override {
        // ... (保持原样，但RCLCPP_INFO改为std::cout << "INFO: " << ...)
        if (resource == "waypoint:write" && drone_state.flight_mode != "Mission") {
            std::cout << "INFO: 拒绝访问：仅允许在Mission模式下执行写操作，当前模式：" << drone_state.flight_mode << std::endl;
            return false;
        }
        // 其余逻辑不变
    }
};

// ABACStrategy（类似调整）
class ABACStrategy : public AccessControlStrategy {
public:
    bool checkAccess(const std::string& drone_id, 
                     const std::string& resource, 
                     const std::map<std::string, std::string>& attributes,
                     const DroneState& drone_state) override {
        // ... (保持原样，日志用std::cout)
    }
};

// RuleBasedStrategy（类似调整）
class RuleBasedStrategy : public AccessControlStrategy {
private:
    std::vector<std::function<bool(const std::string&, const std::string&, 
                                   const std::map<std::string, std::string>&, 
                                   const DroneState&)>> rules;

public:
    RuleBasedStrategy() {
        // ... (保持原样)
    }
    bool checkAccess(const std::string& drone_id, 
                     const std::string& resource, 
                     const std::map<std::string, std::string>& attributes,
                     const DroneState& drone_state) override {
        // ... (保持原样)
    }
};

#endif  // DRONE_BACKEND_HPP