/**
 * 边缘设备管理器实现
 */
#include "edge/edge_device_manager.hpp"
#include "drone_control.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <iostream>
#include <thread>
#include <chrono>
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

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
        core_channel_ = grpc::CreateChannel(device_info_.core_service_address, grpc::InsecureChannelCredentials());
        if (!core_channel_) {
            std::cerr << "[边缘设备] 创建核心服务通道失败: " << device_info_.core_service_address << std::endl;
            connected_to_core_ = false;
            return false;
        }

        const bool ready = core_channel_->WaitForConnected(
            std::chrono::system_clock::now() + std::chrono::seconds(2));
        if (ready) {
            std::cout << "[边缘设备][连接] 到核心服务连接已建立: " << device_info_.core_service_address << std::endl;
        } else {
            std::cout << "[边缘设备][连接] 到核心服务通道已创建（等待首个RPC建立连接）: "
                      << device_info_.core_service_address << std::endl;
        }
        connected_to_core_ = true;

        // 启动心跳线程
        heartbeat_running_.store(true);
        std::thread([this]() {
            while (heartbeat_running_.load()) {
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
        heartbeat_running_.store(false);
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
    if (!connected_to_core_ || !core_channel_) {
        return false;
    }

    try {
        auto stub = drone_control::DroneControlService::NewStub(core_channel_);
        grpc::ClientContext context;
        drone_control::SyncDroneStateRequest request;
        drone_control::SyncDroneStateResponse response;

        request.set_device_id(device_info_.device_id);
        request.set_drone_id(static_cast<int32_t>(drone_id));
        std::cout << "[边缘设备][DEBUG] 开始同步状态: device=" << device_info_.device_id
                  << ", drone=" << drone_id
                  << ", lat=" << state.position.latitude
                  << ", lon=" << state.position.longitude
                  << ", alt=" << state.position.altitude << std::endl;

#ifdef HAVE_NLOHMANN_JSON
        json state_json;
        state_json["position"]["latitude"] = state.position.latitude;
        state_json["position"]["longitude"] = state.position.longitude;
        state_json["position"]["altitude"] = state.position.altitude;
        state_json["velocity"]["north"] = state.velocity_north;
        state_json["velocity"]["east"] = state.velocity_east;
        state_json["velocity"]["down"] = state.velocity_down;
        state_json["battery_percentage"] = state.battery_percentage;
        state_json["is_armed"] = state.is_armed;
        state_json["flight_status"] = static_cast<int>(state.flight_status);
        request.set_state_json(state_json.dump());
#else
        request.set_state_json("{}");
#endif

        auto status = stub->SyncDroneState(&context, request, &response);
        if (!status.ok() || !response.success()) {
            std::cerr << "[边缘设备] 同步无人机状态失败: "
                      << (status.ok() ? response.message() : status.error_message()) << std::endl;
            return false;
        }
        std::cout << "[边缘设备][DEBUG] 状态同步完成: drone=" << drone_id
                  << ", message=" << response.message() << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 同步无人机状态异常: " << e.what() << std::endl;
        return false;
    }
}

bool EdgeDeviceManager::evaluateAccess(const std::string& geofence_token, DroneId drone_id) {
    if (!connected_to_core_ || !core_channel_) {
        return false;
    }

    try {
        auto stub = drone_control::DroneControlService::NewStub(core_channel_);
        grpc::ClientContext context;
        drone_control::EvaluateEdgeAccessRequest request;
        drone_control::EvaluateEdgeAccessResponse response;

        request.set_device_id(device_info_.device_id);
        request.set_edge_device_id(device_info_.device_id);
        request.set_drone_id(static_cast<int32_t>(drone_id));
        request.set_geofence_token(geofence_token);
        std::cout << "[边缘设备][DEBUG] 发起访问评估: device=" << device_info_.device_id
                  << ", drone=" << drone_id
                  << ", token_len=" << geofence_token.size() << std::endl;
#ifdef HAVE_NLOHMANN_JSON
        json request_json;
        request_json["drone_id"] = static_cast<int32_t>(drone_id);
        request_json["context"]["edge_device_id"] = device_info_.device_id;
        request_json["context"]["geofence_token"] = geofence_token;
        request.set_request_json(request_json.dump());
        std::cout << "[边缘设备][DEBUG] 评估请求JSON长度: " << request.request_json().size() << std::endl;
#endif

        auto status = stub->EvaluateEdgeAccess(&context, request, &response);
        if (!status.ok()) {
            std::cerr << "[边缘设备] 评估访问权限失败: " << status.error_message() << std::endl;
            return false;
        }

        if (response.access_granted() && !response.token().empty()) {
            std::lock_guard<std::mutex> lock(token_mutex_);
            cached_access_token_ = response.token();
            cached_flight_plan_json_ = response.flight_plan_json();
            std::cout << "[边缘设备][DEBUG] 缓存评估结果: token_len=" << cached_access_token_.size()
                      << ", flight_plan_len=" << cached_flight_plan_json_.size()
                      << ", validity=" << response.validity_duration() << std::endl;
        } else if (!response.access_granted()) {
            std::lock_guard<std::mutex> lock(token_mutex_);
            cached_access_token_.clear();
            cached_flight_plan_json_.clear();
        }

        if (!response.access_granted()) {
            std::cerr << "[边缘设备] 核心服务拒绝访问: " << response.reason() << std::endl;
        }
        std::cout << "[边缘设备][DEBUG] 访问评估响应: granted=" << (response.access_granted() ? "true" : "false")
                  << ", reason=" << response.reason()
                  << ", decision=" << response.decision() << std::endl;

        return response.access_granted();
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 评估访问权限异常: " << e.what() << std::endl;
        return false;
    }
}

std::string EdgeDeviceManager::getCachedAccessToken() const {
    std::lock_guard<std::mutex> lock(token_mutex_);
    return cached_access_token_;
}

std::string EdgeDeviceManager::getCachedFlightPlanJson() const {
    std::lock_guard<std::mutex> lock(token_mutex_);
    return cached_flight_plan_json_;
}

void EdgeDeviceManager::heartbeatToCore() {
    if (!connected_to_core_ || !core_channel_) {
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
            heartbeat_running_.store(false);
        }
    } catch (const std::exception& e) {
        std::cerr << "[边缘设备] 心跳异常: " << e.what() << std::endl;
        connected_to_core_ = false;
        heartbeat_running_.store(false);
    }
}

void EdgeDeviceManager::processCoreResponses() {
    // 处理来自核心服务的响应
    // 这里可以实现异步处理逻辑
}

} // namespace drone_control