/**
 * 边缘设备管理器实现
 */
#include "edge/edge_device_manager.hpp"
#include "drone_control.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace drone_control {

EdgeDeviceManager::EdgeDeviceManager() : connected_to_core_(false) {
}

EdgeDeviceManager::~EdgeDeviceManager() {
    disconnectFromCoreService();
}

bool EdgeDeviceManager::initialize(const std::string& device_id, const std::string& device_name, const std::string& location, const std::string& core_service_address) {
    device_info_.device_id = device_id;
    device_info_.device_name = device_name;
    device_info_.location = location;
    device_info_.online = true;
    device_info_.last_heartbeat = std::chrono::system_clock::now();
    device_info_.core_service_address = core_service_address;

    std::cout << "[边缘设备] 初始化成功: " << device_name << " (" << device_id << ")" << std::endl;
    std::cout << "[边缘设备] 核心服务地址: " << core_service_address << std::endl;

    return connectToCoreService();
}

bool EdgeDeviceManager::connectToCoreService() {
    try {
        // 暂时简化实现，避免 gRPC 编译错误
        std::cout << "[边缘设备] 模拟连接到核心服务: " << device_info_.core_service_address << std::endl;
        connected_to_core_ = true;
        
        // 启动心跳线程
        std::thread([this]() {
            while (connected_to_core_) {
                heartbeatToCore();
                std::this_thread::sleep_for(std::chrono::seconds(30));
            }
        }).detach();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 连接核心服务异常: " << e.what() << std::endl;
        return false;
    }
    return false;
}

void EdgeDeviceManager::disconnectFromCoreService() {
    if (connected_to_core_) {
        connected_to_core_ = false;
        core_channel_.reset();
        std::cout << "[边缘设备] 断开与核心服务的连接" << std::endl;
    }
}

bool EdgeDeviceManager::isConnectedToCore() const {
    return connected_to_core_;
}

void EdgeDeviceManager::registerDrone(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    if (drones_.find(drone_id) == drones_.end()) {
        drones_[drone_id] = ExtendedDroneState();
        std::cout << "[边缘设备] 注册无人机: " << drone_id << std::endl;
    }
}

void EdgeDeviceManager::unregisterDrone(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    auto it = drones_.find(drone_id);
    if (it != drones_.end()) {
        drones_.erase(it);
        std::cout << "[边缘设备] 注销无人机: " << drone_id << std::endl;
    }
}

void EdgeDeviceManager::updateDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    drones_[drone_id] = state;

    // 通知回调
    for (auto& callback : drone_state_callbacks_) {
        callback(drone_id, state);
    }

    // 同步到核心服务
    syncDroneState(drone_id, state);
}

std::optional<ExtendedDroneState> EdgeDeviceManager::getDroneState(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    auto it = drones_.find(drone_id);
    if (it != drones_.end()) {
        return it->second;
    }
    return std::nullopt;
}

void EdgeDeviceManager::registerEdgeDeviceStatusCallback(EdgeDeviceStatusCallback callback) {
    edge_device_status_callbacks_.push_back(callback);
}

void EdgeDeviceManager::registerDroneStateCallback(DroneStateCallback callback) {
    drone_state_callbacks_.push_back(callback);
}

std::vector<DroneId> EdgeDeviceManager::getRegisteredDrones() const {
    std::lock_guard<std::mutex> lock(drones_mutex_);
    std::vector<DroneId> drones;
    for (const auto& [drone_id, _] : drones_) {
        drones.push_back(drone_id);
    }
    return drones;
}

EdgeDeviceInfo EdgeDeviceManager::getDeviceInfo() const {
    return device_info_;
}

bool EdgeDeviceManager::syncDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    if (!connected_to_core_) {
        return false;
    }

    try {
        auto stub = drone_control::DroneControlService::NewStub(core_channel_);
        grpc::ClientContext context;
        drone_control::SyncDroneStateRequest request;
        drone_control::SyncDroneStateResponse response;

        request.set_device_id(device_info_.device_id);
        request.set_drone_id(drone_id);
        
        // 设置无人机状态
        auto* drone_state = request.mutable_drone_state();
        drone_state->set_drone_id(state.drone_id);
        drone_state->set_latitude(state.position.latitude);
        drone_state->set_longitude(state.position.longitude);
        drone_state->set_altitude(state.position.altitude);
        drone_state->set_speed(state.speed);
        drone_state->set_ground_speed(state.ground_speed);
        drone_state->set_battery_percentage(state.battery_percentage);
        drone_state->set_is_armed(state.is_armed);
        drone_state->set_flight_status(static_cast<int>(state.flight_status));

        auto status = stub->SyncDroneState(&context, request, &response);
        if (status.ok() && response.success()) {
            return true;
        } else {
            std::cerr << "[边缘设备] 同步无人机状态失败: " << status.error_message() << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 同步无人机状态异常: " << e.what() << std::endl;
        return false;
    }
}

bool EdgeDeviceManager::evaluateAccess(const std::string& geofence_token, DroneId drone_id) {
    if (!connected_to_core_) {
        return false;
    }

    try {
        auto stub = drone_control::DroneControlService::NewStub(core_channel_);
        grpc::ClientContext context;
        drone_control::EvaluateEdgeAccessRequest request;
        drone_control::EvaluateEdgeAccessResponse response;

        request.set_device_id(device_info_.device_id);
        request.set_geofence_token(geofence_token);
        request.set_drone_id(drone_id);

        auto status = stub->EvaluateEdgeAccess(&context, request, &response);
        if (status.ok()) {
            return response.access_granted();
        } else {
            std::cerr << "[边缘设备] 评估访问权限失败: " << status.error_message() << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 评估访问权限异常: " << e.what() << std::endl;
        return false;
    }
}

void EdgeDeviceManager::heartbeatToCore() {
    if (!connected_to_core_) {
        return;
    }

    try {
        auto stub = drone_control::DroneControlService::NewStub(core_channel_);
        grpc::ClientContext context;
        drone_control::HeartbeatRequest request;
        drone_control::HeartbeatResponse response;

        request.set_device_id(device_info_.device_id);

        auto status = stub->Heartbeat(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "[边缘设备] 心跳失败: " << status.error_message() << std::endl;
            connected_to_core_ = false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 心跳异常: " << e.what() << std::endl;
        connected_to_core_ = false;
    }
}

void EdgeDeviceManager::processCoreResponses() {
    // 处理来自核心服务的响应
    // 这里可以实现异步处理逻辑
}

} // namespace drone_control