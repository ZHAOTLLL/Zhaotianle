/**
 * 边缘设备管理器
 * 负责边缘设备的注册、状态管理和与核心系统的通信
 */
#pragma once

#include "common/types.hpp"
#include "common/drone_attributes.hpp"
#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <chrono>
#include <functional>

// 包含 gRPC 头文件
#include <grpcpp/grpcpp.h>

namespace drone_control {

// 边缘设备信息
struct EdgeDeviceInfo {
    std::string device_id;
    std::string device_name;
    std::string location;
    bool online;
    std::chrono::system_clock::time_point last_heartbeat;
    std::string core_service_address;
};

// 边缘设备事件回调类型
typedef std::function<void(const std::string&, bool)> EdgeDeviceStatusCallback;
typedef std::function<void(DroneId, const ExtendedDroneState&)> DroneStateCallback;

/** 边缘设备管理器 */
class EdgeDeviceManager {
public:
    EdgeDeviceManager();
    ~EdgeDeviceManager();

    // 初始化
    bool initialize(const std::string& device_id, const std::string& device_name, const std::string& location, const std::string& core_service_address);

    // 核心服务连接管理
    bool connectToCoreService();
    void disconnectFromCoreService();
    bool isConnectedToCore() const;

    // 无人机状态管理
    void registerDrone(DroneId drone_id);
    void unregisterDrone(DroneId drone_id);
    void updateDroneState(DroneId drone_id, const ExtendedDroneState& state);
    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id) const;

    // 回调注册
    void registerEdgeDeviceStatusCallback(EdgeDeviceStatusCallback callback);
    void registerDroneStateCallback(DroneStateCallback callback);

    // 状态查询
    std::vector<DroneId> getRegisteredDrones() const;
    EdgeDeviceInfo getDeviceInfo() const;

    // 核心服务通信
    bool syncDroneState(DroneId drone_id, const ExtendedDroneState& state);
    bool evaluateAccess(const std::string& geofence_token, DroneId drone_id);

private:
    // 边缘设备信息
    EdgeDeviceInfo device_info_;

    // 核心服务连接
    std::shared_ptr<grpc::Channel> core_channel_;
    bool connected_to_core_;

    // 无人机状态管理
    mutable std::mutex drones_mutex_;
    std::map<DroneId, ExtendedDroneState> drones_;

    // 回调
    std::vector<EdgeDeviceStatusCallback> edge_device_status_callbacks_;
    std::vector<DroneStateCallback> drone_state_callbacks_;

    // 内部方法
    void heartbeatToCore();
    void processCoreResponses();
};

} // namespace drone_control