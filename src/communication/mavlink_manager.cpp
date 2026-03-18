/**
 * MAVLink 连接管理实现
 * 连接 PX4、心跳、遥测、指令下发及与访问控制相关的认证与任务交互。
 */
#include "communication/mavlink_manager.hpp"
#include "state/drone_state_manager.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/access_request.hpp"
#include "access_control/access_decision.hpp"
#include "flight_control/flight_plan.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <set>
#include <thread>
#include <atomic>
#include <cmath>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

#ifdef HAVE_MAVSDK
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/mavlink_types.h>
#include <mavlink/common/mavlink.h>
#endif


namespace drone_control {

MAVLinkManager::MAVLinkManager()
    : running_(false)
    , next_drone_id_(1) {
#ifdef HAVE_MAVSDK
    mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::GroundStation);
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(config);
#endif
}
MAVLinkManager::~MAVLinkManager() {
    stop();
}

bool MAVLinkManager::start() {
    if (running_.load()) {
        return true;
    }
    
    running_.store(true);
    
    connection_thread_ = std::thread(&MAVLinkManager::connectionWorker, this);
    telemetry_thread_ = std::thread(&MAVLinkManager::telemetryWorker, this);
    heartbeat_thread_ = std::thread(&MAVLinkManager::heartbeatChecker, this);
    authentication_thread_ = std::thread(&MAVLinkManager::authenticationWorker, this);
    reconnection_thread_ = std::thread(&MAVLinkManager::reconnectionWorker, this);
    return true;
}

void MAVLinkManager::startConcurrentMode() {
    if (running_.load()) return;
    running_.store(true);
}

void MAVLinkManager::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    
    // 等待工作线程结束
    if (connection_thread_.joinable()) {
        connection_thread_.join();
    }
    if (telemetry_thread_.joinable()) {
        telemetry_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    if (authentication_thread_.joinable()) {
        authentication_thread_.join();
    }
    if (reconnection_thread_.joinable()) {
        reconnection_thread_.join();
    }
    disconnectAll();
}

void MAVLinkManager::sendMissionConfirmation(uint32_t drone_id) {
#ifdef HAVE_MAVSDK
    std::unique_lock<std::mutex> lock(connections_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end() || !it->second->mavlink_passthrough) {
        return;
    }
    
    mavlink_message_t message;
    mavlink_command_long_t cmd{};
    
    cmd.target_system = 1;
    cmd.target_component = MAV_COMP_ID_AUTOPILOT1;
    cmd.command = MAV_CMD_USER_1;
    cmd.param1 = 1.0f;
    
    mavlink_msg_command_long_encode(255, MAV_COMP_ID_AUTOPILOT1, &message, &cmd);
    
    auto result = it->second->mavlink_passthrough->send_message(message);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "[MAVLink] 已向无人机 " << drone_id << " 发送任务确认" << std::endl;
    } else {
        std::cout << "[MAVLink] 任务确认发送失败，无人机 " << drone_id << ", result=" << static_cast<int>(result) << std::endl;
    }
#else
    (void)drone_id;
#endif
}

void MAVLinkManager::requestCertificateUpload(uint32_t drone_id) {
#ifdef HAVE_MAVSDK
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end() || !it->second->mavlink_passthrough) {
        return;
    }
    
    mavlink_message_t message;
    mavlink_command_long_t cmd{};
    
    cmd.target_system = 1;
    cmd.target_component = MAV_COMP_ID_AUTOPILOT1;
    cmd.command = MAV_CMD_USER_2;
    cmd.param1 = 1.0f;
    
    mavlink_msg_command_long_encode(255, MAV_COMP_ID_AUTOPILOT1, &message, &cmd);
    
    auto result = it->second->mavlink_passthrough->send_message(message);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "[MAVLink] 已请求无人机 " << drone_id << " 上传证书" << std::endl;
        // 等待PX4端准备证书文件（增加等待时间，确保文件已准备好）
        std::cout << "[MAVLink] 等待PX4端准备证书文件..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 增加到5秒，确保PX4端有足够时间准备文件
        
        // 在独立线程中执行FTP下载，支持重试机制
        std::thread ftp_thread([this, drone_id]() {
            downloadCertificateViaFTPWithRetry(drone_id);
        });
        ftp_thread.detach();
    } else {
        std::cout << "[MAVLink] 证书上传请求失败，无人机 " << drone_id << ", result=" << static_cast<int>(result) << std::endl;
    }
#else
    (void)drone_id;
#endif
}

void MAVLinkManager::performAccessControlEvaluation(uint32_t drone_id) {
    std::cout << "开始访问控制评估..." << std::endl;
    
    if (!access_control_engine_) {
        std::cout << "[错误] 访问控制引擎未设置" << std::endl;
        sendAccessControlResult(drone_id, false, "Access control engine not available");
        return;
    }
    
    // 获取缓存的任务目的信息
    auto mission_it = mission_purpose_cache_.find(drone_id);
    if (mission_it == mission_purpose_cache_.end()) {
        std::cout << "[错误] 未找到无人机 " << drone_id << " 的任务目的信息" << std::endl;
        sendAccessControlResult(drone_id, false, "Mission purpose not found");
        return;
    }
    
    const auto& mission_info = mission_it->second;
    
    // 获取无人机状态
    auto conn_it = connections_.find(drone_id);
    if (conn_it == connections_.end() || !conn_it->second) {
        std::cout << "[错误] 无人机 " << drone_id << " 连接不存在" << std::endl;
        sendAccessControlResult(drone_id, false, "Drone connection not found");
        return;
    }
    
    auto& conn = conn_it->second;
    
    // 使用缓存的任务目的信息构造访问请求
    AccessRequest request = constructAccessRequest(drone_id, mission_info.mission_name, 
                                                 mission_info.target_location, mission_info.operation_type,
                                                 mission_info.certificate_hash, mission_info.mission_description);
    
    // 添加证书内容到访问请求中（如果存在）
    if (!mission_info.certificate_content.empty()) {
        request.context["credential"] = mission_info.certificate_content;
        std::cout << "已将证书内容添加到访问请求中，大小: " << mission_info.certificate_content.size() << " 字节" << std::endl;
    } else {
        std::cout << "[警告] 任务目的缓存中没有证书内容" << std::endl;
    }
    
    // 执行访问控制评估
    std::cout << "执行访问控制评估..." << std::endl;
    std::cout << "   无人机ID: " << request.drone_id << std::endl;
    std::cout << "   任务名称: " << mission_info.mission_name << std::endl;
    std::cout << "   目标位置: " << mission_info.target_location << std::endl;
    std::cout << "   操作类型: " << mission_info.operation_type << std::endl;
    std::cout << "   组织: " << request.attributes.organization << std::endl;
    std::cout << "   信任级别: " << request.attributes.trust_level << std::endl;
    
    AccessDecision decision = access_control_engine_->evaluateAccess(request);
    
    std::cout << "访问控制评估结果:" << std::endl;
    std::cout << "   授权结果: " << (decision.granted ? "通过" : "拒绝") << std::endl;
    std::cout << "   原因: " << decision.reason << std::endl;
    std::cout << "   评估时间: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
    
    // 检查是否生成了地理围栏解锁凭证和飞行计划
    if (decision.granted) {
        std::cout << "生成的访问控制数据:" << std::endl;
        if (!decision.geofence_signature.empty()) {
            std::cout << "   地理围栏解锁凭证（D.cred）: " << decision.geofence_signature << std::endl;
        } else {
            std::cout << "   [错误] 地理围栏解锁凭证未生成" << std::endl;
        }
        if (decision.flight_plan.has_value()) {
            std::cout << "   飞行计划（D.plan）: " << decision.flight_plan->plan_id 
                      << " (" << decision.flight_plan->waypoints.size() << " 个航点)" << std::endl;
        } else {
            std::cout << "   [错误] 飞行计划未生成" << std::endl;
        }
    }
    
    // 在地面端维护全局无人机认证状态：当访问控制判定通过时，立刻标记为已认证并写入认证相关信息
    try {
        auto conn_it2 = connections_.find(drone_id);
        if (conn_it2 != connections_.end() && conn_it2->second) {
            auto& dconn = conn_it2->second;
            if (decision.granted) {
                dconn->is_authenticated = true;
                dconn->status = ConnectionStatus::AUTHENTICATED;
                dconn->state.is_authenticated = true;
                // 使用已知来源填充认证字段
                dconn->state.authentication_type = dconn->credential_type.empty() ? "access_control" : dconn->credential_type;
                // 信任级别优先使用请求中的属性
                if (!request.attributes.trust_level.empty()) {
                    dconn->state.trust_level = request.attributes.trust_level;
                } else {
                    auto it_attr = dconn->extracted_attributes.find("trust_level");
                    if (it_attr != dconn->extracted_attributes.end()) {
                        dconn->state.trust_level = it_attr->second;
                    }
                }
                auto cert_it = dconn->extracted_attributes.find("certificate_id");
                if (cert_it != dconn->extracted_attributes.end()) {
                    dconn->state.certificate_id = cert_it->second;
                }
                dconn->state.auth_timestamp = std::chrono::system_clock::now();
                // 将评估上下文属性一并放入，便于前端调试查看
                dconn->state.auth_attributes = dconn->extracted_attributes;
                
                // 立即同步到状态管理器，便于前端展示
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, dconn->state);
                }
                
                // 输出到标准输出作为“后端调试信息”
                std::cout << "[Access] 无人机 " << drone_id << " 认证通过，trust_level="
                          << dconn->state.trust_level << ", auth_type=" << dconn->state.authentication_type
                          << ", cert_id=" << dconn->state.certificate_id << std::endl;
            } else {
                // 若拒绝，显式置为未认证，便于前端清晰展示
                dconn->is_authenticated = false;
                dconn->state.is_authenticated = false;
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, dconn->state);
                }
                std::cout << "[Access] 无人机 " << drone_id << " 认证被拒绝，原因: " << decision.reason << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[警告] 更新地面端认证状态时异常: " << e.what() << std::endl;
    }

    // 自动发送访问控制结果给无人机
    std::cout << "自动发送访问控制结果给无人机 " << drone_id << std::endl;
    sendAccessControlResult(drone_id, decision.granted, decision.reason);
    
    // 如果授权通过，发送地理围栏解锁凭证和飞行计划
    if (decision.granted) {
        std::cout << "准备发送访问控制数据给无人机 " << drone_id << "..." << std::endl;
        
        if (!decision.geofence_signature.empty()) {
            std::cout << "[发送] 地理围栏解锁凭证（D.cred）: " << decision.geofence_signature << std::endl;
            sendGeofenceSignature(drone_id, decision.geofence_signature, decision.validity_duration);
        } else {
            std::cout << "[发送] 地理围栏解锁凭证为空，跳过发送" << std::endl;
        }
        
        if (decision.flight_plan.has_value()) {
            std::cout << "[发送] 飞行计划（D.plan）: " << decision.flight_plan->plan_id 
                      << " (" << decision.flight_plan->waypoints.size() << " 个航点)" << std::endl;
            sendFlightPlan(drone_id, decision.flight_plan.value());
        } else {
            std::cout << "[发送] 飞行计划为空，跳过发送" << std::endl;
        }
    }
    
    std::cout << "访问控制评估完成，保持缓存以防止重复接收消息" << std::endl;
}

void MAVLinkManager::sendAccessControlResult(uint32_t drone_id, bool access_granted, const std::string& reason) {
#ifdef HAVE_MAVSDK
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        std::cout << "[错误] 无人机 " << drone_id << " 未连接，无法发送访问控制结果" << std::endl;
        return;
    }
    
    auto& conn = it->second;
    if (!conn->mavlink_passthrough) {
        std::cout << "[错误] 无人机 " << drone_id << " MAVLink通道未建立" << std::endl;
        return;
    }
    
    // 发送VEHICLE_CMD_USER_3作为访问控制结果命令
    mavlink_message_t message;
    mavlink_command_long_t cmd;
    
    cmd.target_system = 1;  // PX4 SITL的系统ID通常是1
    cmd.target_component = MAV_COMP_ID_AUTOPILOT1;
    cmd.command = MAV_CMD_USER_3;  // 访问控制结果命令
    cmd.confirmation = 0;
    cmd.param1 = access_granted ? 1.0f : 0.0f;  // 访问结果
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    
    mavlink_msg_command_long_encode(255, MAV_COMP_ID_AUTOPILOT1, &message, &cmd);
    
    // 通过MAVLink通道发送
    auto result = conn->mavlink_passthrough->send_message(message);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "访问控制结果已发送给无人机 " << drone_id 
                  << ": " << (access_granted ? "ACCESS_GRANTED" : "ACCESS_DENIED") << std::endl;
        std::cout << "   原因: " << reason << std::endl;
    } else {
        std::cout << "[错误] 发送访问控制结果失败: " << static_cast<int>(result) << std::endl;
    }
#else
    std::cout << "[警告] MAVSDK 不可用，无法发送访问控制结果" << std::endl;
#endif
}

void MAVLinkManager::sendGeofenceSignature(uint32_t drone_id, const std::string& signature, const std::chrono::seconds& validity_duration) {
#ifdef HAVE_MAVSDK
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end() || !it->second->mavlink_passthrough) {
        std::cout << "[错误] 无人机 " << drone_id << " 未连接或MAVLink通道未建立，无法发送地理围栏解锁凭证" << std::endl;
        return;
    }
    
    // 使用自定义MAVLink消息 GEOFENCE_SIGNATURE (ID 401)
    // 注意：由于MAVLink消息定义可能未完全生成，这里使用command_long作为临时方案
    // 实际应该使用自定义消息 GEOFENCE_SIGNATURE
    mavlink_message_t message;
    mavlink_command_long_t cmd;
    
    cmd.target_system = 1;
    cmd.target_component = MAV_COMP_ID_AUTOPILOT1;
    cmd.command = 31013;  // 使用自定义命令作为地理围栏解锁凭证命令（临时方案，待定义MAV_CMD_USER_4）
    cmd.confirmation = 0;
    cmd.param1 = 1.0f;  // 表示这是地理围栏解锁凭证
    cmd.param2 = static_cast<float>(validity_duration.count());  // 有效期（秒）
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    
    mavlink_msg_command_long_encode(255, MAV_COMP_ID_AUTOPILOT1, &message, &cmd);
    
    auto result = it->second->mavlink_passthrough->send_message(message);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "地理围栏解锁凭证命令已发送给无人机 " << drone_id 
                  << "，有效期: " << validity_duration.count() << " 秒" << std::endl;
        std::cout << "   签名长度: " << signature.length() << " 字节" << std::endl;
        
        // 注意：由于MAVLink command_long的param字段有限，实际签名数据需要通过FTP或其他方式传输
        // 这里先发送命令通知PX4端准备接收，然后通过FTP发送签名文件
        // TODO: 实现通过FTP发送签名文件的逻辑
    } else {
        std::cout << "[错误] 发送地理围栏解锁凭证失败: " << static_cast<int>(result) << std::endl;
    }
#else
    (void)drone_id;
    (void)signature;
    (void)validity_duration;
    std::cout << "[警告] MAVSDK 不可用，无法发送地理围栏凭证" << std::endl;
#endif
}

void MAVLinkManager::sendFlightPlan(uint32_t drone_id, const FlightPlan& flight_plan) {
#ifdef HAVE_MAVSDK
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end() || !it->second->mission) {
        std::cout << "[错误] 无人机 " << drone_id << " 未连接或Mission插件未初始化，无法发送飞行计划" << std::endl;
        return;
    }
    
    // 将FlightPlan转换为MAVSDK Mission格式
    std::vector<mavsdk::Mission::MissionItem> mission_items;
    
    for (const auto& waypoint : flight_plan.waypoints) {
        mavsdk::Mission::MissionItem item;
        item.latitude_deg = waypoint.position.latitude;
        item.longitude_deg = waypoint.position.longitude;
        item.relative_altitude_m = waypoint.position.altitude;
        item.speed_m_s = waypoint.speed_mps;
        item.is_fly_through = true;
        item.gimbal_pitch_deg = 0.0f;
        item.gimbal_yaw_deg = 0.0f;
        item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
        
        // MAVSDK MissionItem没有command字段，使用MAVLink命令通过其他方式发送
        // 这里先使用基本的航点格式
        mission_items.push_back(item);
    }
    
    if (mission_items.empty()) {
        std::cout << "[警告] 飞行计划为空，无法发送" << std::endl;
        return;
    }
    
    // 构建MissionPlan
    mavsdk::Mission::MissionPlan mission_plan;
    mission_plan.mission_items = mission_items;
    
    // 上传任务到无人机
    std::cout << "上传飞行计划到无人机 " << drone_id << "，包含 " << mission_items.size() << " 个航点" << std::endl;
    
    auto upload_result = it->second->mission->upload_mission(mission_plan);
    if (upload_result == mavsdk::Mission::Result::Success) {
        std::cout << "飞行计划上传成功" << std::endl;
        
        // 启动任务
        auto start_result = it->second->mission->start_mission();
        if (start_result == mavsdk::Mission::Result::Success) {
            std::cout << "飞行计划已启动" << std::endl;
        } else {
            std::cout << "[警告] 飞行计划上传成功但启动失败: " << static_cast<int>(start_result) << std::endl;
        }
    } else {
        std::cout << "[错误] 飞行计划上传失败: " << static_cast<int>(upload_result) << std::endl;
    }
#else
    (void)drone_id;
    (void)flight_plan;
    std::cout << "[警告] MAVSDK 不可用，无法发送飞行计划" << std::endl;
#endif
}

bool MAVLinkManager::isRunning() const {
    return running_.load();
}

bool MAVLinkManager::connectToDrone(const std::string& connection_url) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    // 检查是否已经连接到此URL
    auto it = url_to_drone_id_.find(connection_url);
    if (it != url_to_drone_id_.end()) {
        std::cout << "Already connected to " << connection_url << std::endl;
        return true;
    }
    
    // 创建新的连接
    auto conn = std::make_unique<DroneConnection>();
    conn->drone_id = generateDroneId();
    conn->connection_url = connection_url;
    conn->status = ConnectionStatus::CONNECTING;
    
    DroneId drone_id = conn->drone_id;
    
    // 存储连接信息
    connections_[drone_id] = std::move(conn);
    url_to_drone_id_[connection_url] = drone_id;
    
    std::cout << "Initiating connection to drone " << drone_id 
              << " at " << connection_url << std::endl;
    
    // 通知连接状态变化
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::CONNECTING);
    }
    
    return true;
}

bool MAVLinkManager::startDiscovery(const std::string& listen_url) {
#ifdef HAVE_MAVSDK
    if (!mavsdk_) return false;
    if (discovery_mode_) {
        std::cout << "Discovery already active on " << discovery_listen_url_ << std::endl;
        return true;
    }
    auto result = mavsdk_->add_any_connection(listen_url);
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Failed to start discovery listener: " << listen_url << std::endl;
        return false;
    }
    discovery_listen_url_ = listen_url;
    discovery_mode_ = true;
    std::cout << "LAN discovery started, listening on " << listen_url << std::endl;
    return true;
#else
    (void)listen_url;
    return false;
#endif
}

void MAVLinkManager::disconnectFromDrone(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return;
    }
    
    for (auto url_it = url_to_drone_id_.begin(); url_it != url_to_drone_id_.end(); ++url_it) {
        if (url_it->second == drone_id) { url_to_drone_id_.erase(url_it); break; }
    }
    for (auto it = system_key_to_drone_id_.begin(); it != system_key_to_drone_id_.end(); ++it) {
        if (it->second == drone_id) { system_key_to_drone_id_.erase(it); break; }
    }
    // 更新状态并通知
    it->second->status = ConnectionStatus::DISCONNECTED;
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::DISCONNECTED);
    }
    
    // 移除连接
    connections_.erase(it);
    
    std::cout << "Disconnected from drone " << drone_id << std::endl;
}

void MAVLinkManager::disconnectAll() {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto& [drone_id, conn] : connections_) {
        for (auto& callback : connection_callbacks_) callback(drone_id, ConnectionStatus::DISCONNECTED);
    }
    connections_.clear();
    url_to_drone_id_.clear();
    system_key_to_drone_id_.clear();
    std::cout << "Disconnected from all drones" << std::endl;
}

std::vector<DroneId> MAVLinkManager::getConnectedDrones() const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    std::vector<DroneId> connected_drones;
    for (const auto& [drone_id, conn] : connections_) {
        if (conn->status == ConnectionStatus::CONNECTED || conn->status == ConnectionStatus::AUTHENTICATED) {
            connected_drones.push_back(drone_id);
        }
    }
    
    return connected_drones;
}

MAVLinkManager::ConnectionStatus MAVLinkManager::getConnectionStatus(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return ConnectionStatus::DISCONNECTED;
    }
    
    return it->second->status;
}

bool MAVLinkManager::isDroneConnected(DroneId drone_id) const {
    auto status = getConnectionStatus(drone_id);
    return status == ConnectionStatus::CONNECTED || status == ConnectionStatus::AUTHENTICATED;
}

void MAVLinkManager::registerConnectionCallback(ConnectionCallback callback) {
    connection_callbacks_.push_back(callback);
}

void MAVLinkManager::registerTelemetryCallback(TelemetryCallback callback) {
    telemetry_callbacks_.push_back(callback);
}

void MAVLinkManager::registerAuthenticationCallback(AuthenticationCallback callback) {
    authentication_callbacks_.push_back(callback);
}

bool MAVLinkManager::sendCommand(DroneId drone_id, const std::string& command, 
                                const std::map<std::string, std::string>& parameters) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        std::cout << "[错误] [MAVLinkManager] 无人机 " << drone_id << " 连接不存在" << std::endl;
        return false;
    }
    
    // 检查连接状态 - 支持CONNECTED和AUTHENTICATED状态
    if (it->second->status != ConnectionStatus::CONNECTED && 
        it->second->status != ConnectionStatus::AUTHENTICATED) {
        std::cout << "[错误] [MAVLinkManager] 无人机 " << drone_id << " 状态不正确: " 
                  << static_cast<int>(it->second->status) << std::endl;
        return false;
    }
    
    std::cout << "[MAVLinkManager] 无人机 " << drone_id << " 连接状态正常，准备发送命令: " << command << std::endl;
    
#ifdef HAVE_MAVSDK
    // 使用MAVSDK发送命令
    if (it->second->action) {
        try {
            if (command == "arm") {
                auto result = it->second->action->arm();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully armed drone " << drone_id << std::endl;
                    return true;
                } else {
                    std::cout << "Failed to arm drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "disarm") {
                auto result = it->second->action->disarm();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully disarmed drone " << drone_id << std::endl;
                    return true;
                } else {
                    std::cout << "Failed to disarm drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "takeoff") {
                float altitude = 10.0f; // 默认高度
                auto alt_param = parameters.find("altitude");
                if (alt_param != parameters.end()) {
                    try {
                        altitude = std::stof(alt_param->second);
                    } catch (const std::exception& e) {
                        std::cout << "Invalid altitude parameter: " << e.what() << std::endl;
                    }
                }
                
                auto result = it->second->action->set_takeoff_altitude(altitude);
                if (result == mavsdk::Action::Result::Success) {
                    result = it->second->action->takeoff();
                    if (result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully initiated takeoff for drone " << drone_id << " to " << altitude << "m" << std::endl;
                        return true;
                    } else {
                        std::cout << "Failed to takeoff drone " << drone_id << ": " << result << std::endl;
                        return false;
                    }
                } else {
                    std::cout << "Failed to set takeoff altitude for drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "land") {
                auto result = it->second->action->land();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated landing for drone " << drone_id << " (keeping armed)" << std::endl;
                    return true;
                } else {
                    std::cout << "Failed to land drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "emergency_land") {
                auto result = it->second->action->land();
                if (result == mavsdk::Action::Result::Success) {
                    // 紧急降落：先降落，然后解除武装
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    auto disarm_result = it->second->action->disarm();
                    if (disarm_result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully initiated emergency landing and disarmed drone " << drone_id << std::endl;
                    } else {
                        std::cout << "Emergency landing initiated but failed to disarm drone " << drone_id << std::endl;
                    }
                    return true;
                } else {
                    std::cout << "Failed to emergency land drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "return_to_launch") {
                auto result = it->second->action->return_to_launch();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated return to launch for drone " << drone_id << std::endl;
                    return true;
                } else {
                    std::cout << "Failed to return to launch drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "hold") {
                auto result = it->second->action->hold();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated hold for drone " << drone_id << std::endl;
                    return true;
                } else {
                    std::cout << "Failed to hold drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
            }
            else if (command == "move_absolute") {
                std::cout << "[MAVLinkManager] 执行绝对位置移动命令..." << std::endl;
                
                // 解析位置参数
                double latitude = 0.0, longitude = 0.0, altitude = 0.0, yaw = 0.0;
                
                auto lat_it = parameters.find("latitude");
                auto lon_it = parameters.find("longitude");
                auto alt_it = parameters.find("altitude");
                auto yaw_it = parameters.find("yaw");
                
                if (lat_it != parameters.end()) {
                    try {
                        latitude = std::stod(lat_it->second);
                    } catch (const std::exception& e) {
                        std::cout << "[错误] [MAVLinkManager] 无效的纬度参数: " << e.what() << std::endl;
                        return false;
                    }
                }
                
                if (lon_it != parameters.end()) {
                    try {
                        longitude = std::stod(lon_it->second);
                    } catch (const std::exception& e) {
                        std::cout << "[错误] [MAVLinkManager] 无效的经度参数: " << e.what() << std::endl;
                        return false;
                    }
                }
                
                if (alt_it != parameters.end()) {
                    try {
                        altitude = std::stod(alt_it->second);
                    } catch (const std::exception& e) {
                        std::cout << "[错误] [MAVLinkManager] 无效的高度参数: " << e.what() << std::endl;
                        return false;
                    }
                }
                
                if (yaw_it != parameters.end()) {
                    try {
                        yaw = std::stod(yaw_it->second);
                    } catch (const std::exception& e) {
                        std::cout << "[警告] [MAVLinkManager] 无效的航向参数，使用默认值0: " << e.what() << std::endl;
                        yaw = 0.0;
                    }
                }
                
                std::cout << "   目标位置: 纬度=" << latitude << ", 经度=" << longitude 
                          << ", 高度=" << altitude << "米, 航向=" << yaw << "度" << std::endl;
                
                // 添加坐标验证
                if (latitude < -90 || latitude > 90) {
                    std::cout << "[错误] [MAVLinkManager] 纬度超出有效范围: " << latitude << std::endl;
                    return false;
                }
                if (longitude < -180 || longitude > 180) {
                    std::cout << "[错误] [MAVLinkManager] 经度超出有效范围: " << longitude << std::endl;
                    return false;
                }
                if (altitude < 0 || altitude > 1000) {
                    std::cout << "[错误] [MAVLinkManager] 高度超出合理范围: " << altitude << std::endl;
                    return false;
                }
                
                // 使用MAVSDK Action模式实现绝对坐标飞行
                try {
                    // 先停止当前任务（如果有）
                    it->second->mission->pause_mission();
                    
                    // 确保飞行模式正确
                    auto flight_mode = it->second->telemetry->flight_mode();
                    std::cout << "   🛩️ 当前飞行模式: " << flight_mode << std::endl;
                    
                    // 检查当前飞行模式
                    bool use_mission_mode = false;
                    if (flight_mode == mavsdk::Telemetry::FlightMode::Hold) {
                        std::cout << "   [警告] 当前为Hold模式，goto_location不会工作，直接使用Mission模式" << std::endl;
                        use_mission_mode = true;
                    } else if (flight_mode == mavsdk::Telemetry::FlightMode::Offboard) {
                        std::cout << "   [警告] 当前为Offboard模式，goto_location可能不工作，建议先退出Offboard模式" << std::endl;
                        // 先停止 Offboard 模式
                        try {
                            auto* offboard_ptr = static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                            offboard_ptr->stop();
                            std::cout << "   🔧 已停止Offboard模式，准备使用goto_location" << std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        } catch (const std::exception& e) {
                            std::cout << "   [警告] 停止Offboard模式失败: " << e.what() << std::endl;
                        }
                    } else {
                        std::cout << "   ℹ️ 当前飞行模式: " << flight_mode << "，goto_location将处理模式切换" << std::endl;
                    }
                    
                    // 修复：根据 MAVSDK 文档，goto_location 的第三个参数是绝对高度
                    // 这里直接使用目标高度，让无人机飞到指定高度
                    
                    std::cout << "   📊 目标位置: 纬度=" << latitude << ", 经度=" << longitude << ", 高度=" << altitude << "米" << std::endl;
                    std::cout << "   📊 使用绝对高度进行飞行" << std::endl;
                    
                    // 添加坐标验证和调试信息
                    std::cout << "   🔍 坐标验证: 纬度范围[" << latitude << "] 经度范围[" << longitude << "] 高度范围[" << altitude << "]" << std::endl;
                    
                    // 检查坐标是否在合理范围内（合肥地区）
                    if (latitude < 31.0 || latitude > 32.0) {
                        std::cout << "   [警告] 纬度超出合肥地区范围，可能坐标错误" << std::endl;
                    }
                    if (longitude < 117.0 || longitude > 118.0) {
                        std::cout << "   [警告] 经度超出合肥地区范围，可能坐标错误" << std::endl;
                    }
                    
                    // 检查是否强制使用任务模式
                    auto mission_mode_it = parameters.find("use_mission_mode");
                    if (mission_mode_it != parameters.end() && mission_mode_it->second == "true") {
                        std::cout << "   🔄 检测到Mission模式请求，使用Mission模式确保高度保持..." << std::endl;
                        use_mission_mode = true; // 使用Mission模式确保高度保持
                    }
                    
                    // 根据飞行模式选择执行方式
                    if (use_mission_mode) {
                        std::cout << "   🔄 使用 Mission 模式执行位置控制..." << std::endl;
                    } else {
                        // 尝试使用 goto_location 方法（使用相对高度）
                        // 注意：MAVSDK 的 goto_location 期望度为单位的角度，高度为相对高度
                        std::cout << "   🚁 发送 goto_location 命令: lat=" << latitude << "°, lon=" << longitude << "°, alt=" << altitude << "m, yaw=" << yaw << "°" << std::endl;
                        std::cout << "   📊 使用相对高度进行飞行（相对于起飞点）" << std::endl;
                        auto result = it->second->action->goto_location(latitude, longitude, altitude, yaw);
                        if (result == mavsdk::Action::Result::Success) {
                            std::cout << "[MAVLinkManager] 无人机 " << drone_id << " 开始飞往目标位置" << std::endl;
                            std::cout << "   目标: 纬度=" << latitude << "°, 经度=" << longitude << "°, 高度=" << altitude << "m" << std::endl;
                            return true;
                        } else {
                            std::cout << "[错误] [MAVLinkManager] goto_location 失败: " << result << std::endl;
                            std::cout << "   🔍 失败原因分析: 可能是坐标格式、飞行模式或权限问题" << std::endl;
                            
                            // 检查是否明确要求使用Mission模式
                            auto mission_mode_it = parameters.find("use_mission_mode");
                            if (mission_mode_it != parameters.end() && mission_mode_it->second == "true") {
                                std::cout << "   🔄 明确要求使用 Mission 模式，尝试Mission模式..." << std::endl;
                                use_mission_mode = true;
                            } else {
                                std::cout << "   [错误] goto_location失败且未要求Mission模式，任务执行失败" << std::endl;
                                return false;
                            }
                        }
                    }
                    
                    // 使用 Mission 模式
                    if (use_mission_mode) {
                        try {
                            // 创建航点任务 - 优化高度保持
                            std::vector<mavsdk::Mission::MissionItem> mission_items;
                            
                            // 第一个航点：确保高度
                            mavsdk::Mission::MissionItem altitude_item;
                            altitude_item.latitude_deg = latitude;
                            altitude_item.longitude_deg = longitude;
                            altitude_item.relative_altitude_m = altitude; // 使用相对高度（相对于起飞点）
                            altitude_item.speed_m_s = 10.0f; // 降低速度确保稳定
                            altitude_item.is_fly_through = false; // 必须到达该点
                            altitude_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                            altitude_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                            altitude_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                            altitude_item.loiter_time_s = 2.0f; // 在该点悬停2秒确保高度稳定
                            altitude_item.camera_photo_interval_s = 0.0f;
                            
                            mission_items.push_back(altitude_item);
                            
                            // 第二个航点：目标位置
                            mavsdk::Mission::MissionItem target_item;
                            target_item.latitude_deg = latitude;
                            target_item.longitude_deg = longitude;
                            target_item.relative_altitude_m = altitude; // 保持相同高度
                            target_item.speed_m_s = 5.0f; // 更慢的速度确保精度
                            target_item.is_fly_through = false; // 必须到达该点
                            target_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                            target_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                            target_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                            target_item.loiter_time_s = 5.0f; // 在目标点悬停5秒
                            target_item.camera_photo_interval_s = 0.0f;
                            
                            mission_items.push_back(target_item);
                            
                            std::cout << "   🚀 设置飞行速度: 10 m/s (高度保持) + 5 m/s (精确定位)" << std::endl;
                            std::cout << "   创建2个航点确保高度保持和精确定位" << std::endl;
                            
                            // 创建MissionPlan
                            mavsdk::Mission::MissionPlan mission_plan;
                            mission_plan.mission_items = mission_items;
                            
                            // 上传并执行任务
                            auto upload_result = it->second->mission->upload_mission(mission_plan);
                            if (upload_result != mavsdk::Mission::Result::Success) {
                                std::cout << "[错误] [MAVLinkManager] 上传任务失败: " << upload_result << std::endl;
                                return false;
                            }
                            
                            std::cout << "[MAVLinkManager] 任务上传成功，等待Mission ID设置..." << std::endl;
                            
                            // 等待Mission ID设置和GPS位置锁定（最多等待30秒）
                            bool mission_ready = false;
                            for (int retry = 0; retry < 300; retry++) {
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                
                                // 检查Mission是否已准备好 - 使用download_mission验证
                                try {
                                    // 检查Mission是否已下载（验证Mission ID是否已设置）
                                    auto download_result = it->second->mission->download_mission();
                                    if (download_result.first == mavsdk::Mission::Result::Success && 
                                        download_result.second.mission_items.size() > 0) {
                                        std::cout << "   📥 Mission下载验证: 航点数量=" << download_result.second.mission_items.size() << std::endl;
                                        
                                        // 额外等待时间让Navigator处理Mission并更新mission_result
                                        if (retry > 150) { // 等待15秒后开始检查Mission状态
                                            std::cout << "   🔍 等待Navigator处理Mission并更新mission_result... (" << (retry-150) << "/150)" << std::endl;
                                            
                                            // 再等待一些时间让Navigator完成Mission处理
                                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                        }
                                        
                                        // 等待GPS位置锁定完成
                                        if (retry > 100) { // 等待10秒后开始检查GPS状态
                                            std::cout << "   🛰️ 等待GPS位置锁定完成... (" << (retry-100) << "/200)" << std::endl;
                                            
                                            // 再等待一些时间让GPS位置锁定
                                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                        }
                                        
                                        mission_ready = true;
                                        std::cout << "   Mission下载验证通过，Mission ID已设置" << std::endl;
                                        break;
                                    }
                                    
                                    // 等待更长时间让PX4完成Mission ID设置
                                    if (retry > 50) { // 等待5秒后开始检查Mission状态
                                        std::cout << "   🔍 等待PX4完成Mission ID设置... (" << (retry-50) << "/250)" << std::endl;
                                    }
                                } catch (const std::exception& e) {
                                    std::cout << "   🔍 检查Mission信息时异常: " << e.what() << std::endl;
                                }
                                
                                if (retry % 50 == 0) {
                                    std::cout << "   ⏳ 等待Mission ID设置和GPS位置锁定... (" << (retry/50 + 1) << "/6)" << std::endl;
                                }
                            }
                            
                            if (!mission_ready) {
                                std::cout << "[警告] [MAVLinkManager] Mission ID设置超时，但继续尝试启动任务..." << std::endl;
                            }
                            
                            // 尝试启动任务
                            auto start_result = it->second->mission->start_mission();
                            if (start_result != mavsdk::Mission::Result::Success) {
                                std::cout << "[错误] [MAVLinkManager] 启动任务失败: " << start_result << std::endl;
                                
                                // 如果启动失败，尝试清除当前任务并重新上传
                                std::cout << "🔄 [MAVLinkManager] 尝试清除任务并重新上传..." << std::endl;
                                
                                // 清除当前任务
                                it->second->mission->clear_mission();
                                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                
                                // 重新上传任务
                                auto retry_upload = it->second->mission->upload_mission(mission_plan);
                                if (retry_upload == mavsdk::Mission::Result::Success) {
                                    std::cout << "[MAVLinkManager] 重新上传任务成功，再次尝试启动..." << std::endl;
                                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                    
                                    auto retry_start = it->second->mission->start_mission();
                                    if (retry_start != mavsdk::Mission::Result::Success) {
                                        std::cout << "[错误] [MAVLinkManager] 重试启动任务仍然失败: " << retry_start << std::endl;
                                        return false;
                                    }
                                } else {
                                    std::cout << "[错误] [MAVLinkManager] 重新上传任务失败: " << retry_upload << std::endl;
                                    return false;
                                }
                            }
                            
                            std::cout << "[MAVLinkManager] 无人机 " << drone_id << " 开始执行任务飞往目标位置" << std::endl;
                            return true;
                        } catch (const std::exception& e) {
                            std::cout << "[错误] [MAVLinkManager] Mission 模式也失败: " << e.what() << std::endl;
                            
                            // Mission模式失败，使用高度保持的备选方案
                            std::cout << "🔄 [MAVLinkManager] Mission模式失败，使用高度保持的备选方案..." << std::endl;
                            
                            try {
                                // 方案1：先设置高度，再设置位置
                                std::cout << "   步骤1: 确保高度保持在 " << altitude << " 米..." << std::endl;
                                
                                // 使用goto_location，但先确保高度
                                auto goto_result = it->second->action->goto_location(latitude, longitude, altitude, 0.0f);
                                if (goto_result == mavsdk::Action::Result::Success) {
                                    std::cout << "[MAVLinkManager] 使用高度保持的goto_location成功" << std::endl;
                                    std::cout << "   [警告] 注意：请监控高度变化，如发现下降请立即干预" << std::endl;
                                    return true;
                                } else {
                                    std::cout << "[错误] [MAVLinkManager] 高度保持方案也失败: " << goto_result << std::endl;
                                    std::cout << "   💡 建议：检查GPS信号和系统健康状态" << std::endl;
                                    return false;
                                }
                            } catch (const std::exception& goto_e) {
                                std::cout << "[错误] [MAVLinkManager] 高度保持方案异常: " << goto_e.what() << std::endl;
                                return false;
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cout << "[错误] [MAVLinkManager] 绝对位置移动异常: " << e.what() << std::endl;
                    return false;
                }
            }
            else if (command == "set_mode") {
                auto mode_param = parameters.find("mode");
                if (mode_param == parameters.end()) {
                    std::cout << "Missing 'mode' parameter for set_flight_mode command" << std::endl;
                    return false;
                }
                
                // 将飞行模式字符串转换为MAVSDK的枚举
                std::string mode = mode_param->second;
                mavsdk::Action::Result result = mavsdk::Action::Result::Unknown;
                
                if (mode == "MANUAL") {
                    // 手动模式 - 通过设置offboard模式来实现
                    std::cout << "Setting flight mode to MANUAL for drone " << drone_id << std::endl;
                    result = mavsdk::Action::Result::Success; // 简化处理
                } else if (mode == "AUTO") {
                    // 自动模式
                    std::cout << "Setting flight mode to AUTO for drone " << drone_id << std::endl;
                    result = mavsdk::Action::Result::Success; // 简化处理
                } else if (mode == "RTL") {
                    // 返航模式
                    result = it->second->action->return_to_launch();
                    if (result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully set flight mode to RTL for drone " << drone_id << std::endl;
                    } else {
                        std::cout << "Failed to set flight mode to RTL for drone " << drone_id << ": " << result << std::endl;
                    }
                } else if (mode == "HOLD") {
                    // 悬停模式
                    result = it->second->action->hold();
                    if (result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully set flight mode to HOLD for drone " << drone_id << std::endl;
                    } else {
                        std::cout << "Failed to set flight mode to HOLD for drone " << drone_id << ": " << result << std::endl;
                    }
                } else if (mode == "GUIDED") {
                    // GUIDED模式 - 通过offboard模式实现
                    std::cout << "Setting flight mode to GUIDED (via offboard) for drone " << drone_id << std::endl;
                    // GUIDED模式在MAVSDK中通常通过offboard模式实现
                    // 这里我们直接返回成功，因为goto_location会自动处理
                    result = mavsdk::Action::Result::Success;
                } else if (mode == "OFFBOARD") {
                    // Offboard模式 - 用于外部控制
                    std::cout << "Setting flight mode to OFFBOARD for drone " << drone_id << std::endl;
                    // 注意：MAVSDK 中没有直接的 set_mode API
                    // Offboard 模式需要通过 offboard 插件来启动
                    // 这里我们返回成功，实际模式切换会在 offboard 控制中处理
                    result = mavsdk::Action::Result::Success;
                } else {
                    std::cout << "Unknown flight mode: " << mode << std::endl;
                    return false;
                }
                
                return (result == mavsdk::Action::Result::Success);
            }
            else if (command == "offboard_control") {
                auto cmd_param = parameters.find("command");
                if (cmd_param == parameters.end()) {
                    std::cout << "Missing 'command' parameter for offboard_control" << std::endl;
                    return false;
                }
                
                std::string offboard_cmd = cmd_param->second;
                std::cout << "Executing offboard control command: " << offboard_cmd << " for drone " << drone_id << std::endl;
                
                if (offboard_cmd == "start_offboard") {
                    // 启动 offboard 模式
                    try {
                        // 设置位置和速度控制
                        mavsdk::Offboard::PositionNedYaw position_ned_yaw{};
                        mavsdk::Offboard::VelocityNedYaw velocity_ned_yaw{};
                        
                        // 获取当前位置作为起始点，保持当前位置
                        auto position = it->second->telemetry->position();
                        // 在 Offboard 模式下，我们需要持续发送当前位置来保持位置
                        // 这里设置为当前位置，让无人机保持在当前位置
                        position_ned_yaw.north_m = 0.0f;  // 相对于当前位置
                        position_ned_yaw.east_m = 0.0f;   // 相对于当前位置
                        position_ned_yaw.down_m = -position.relative_altitude_m; // 保持当前高度
                        position_ned_yaw.yaw_deg = 0.0f;
                        
                        // 设置速度控制
                        velocity_ned_yaw.north_m_s = 0.0f;
                        velocity_ned_yaw.east_m_s = 0.0f;
                        velocity_ned_yaw.down_m_s = 0.0f;
                        velocity_ned_yaw.yaw_deg = 0.0f;
                        
                        // 设置 offboard 控制
                        auto* offboard_ptr = static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                        offboard_ptr->set_position_ned(position_ned_yaw);
                        offboard_ptr->set_velocity_ned(velocity_ned_yaw);
                        
                        // 启动 offboard 模式
                        auto offboard_result = offboard_ptr->start();
                        if (offboard_result == mavsdk::Offboard::Result::Success) {
                            std::cout << "Successfully started offboard mode for drone " << drone_id << std::endl;
                            return true;
                        } else {
                            std::cout << "Failed to start offboard mode for drone " << drone_id << ": " << offboard_result << std::endl;
                            return false;
                        }
                    } catch (const std::exception& e) {
                        std::cout << "Exception starting offboard mode: " << e.what() << std::endl;
                        return false;
                    }
                } else if (offboard_cmd == "stop_offboard") {
                    // 停止 offboard 模式
                    try {
                        auto* offboard_ptr = static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                        auto offboard_result = offboard_ptr->stop();
                        if (offboard_result == mavsdk::Offboard::Result::Success) {
                            std::cout << "Successfully stopped offboard mode for drone " << drone_id << std::endl;
                            return true;
                        } else {
                            std::cout << "Failed to stop offboard mode for drone " << drone_id << ": " << offboard_result << std::endl;
                            return false;
                        }
                    } catch (const std::exception& e) {
                        std::cout << "Exception stopping offboard mode: " << e.what() << std::endl;
                        return false;
                    }
                } else {
                    std::cout << "Unknown offboard command: " << offboard_cmd << std::endl;
                    return false;
                }
            }
            else if (command == "move_relative") {
                // 相对坐标飞行
                float north = 0.0f, east = 0.0f, down = 0.0f, yaw = 0.0f;
                
                auto north_param = parameters.find("north");
                if (north_param != parameters.end()) {
                    try { north = std::stof(north_param->second); } catch (...) {}
                }
                auto east_param = parameters.find("east");
                if (east_param != parameters.end()) {
                    try { east = std::stof(east_param->second); } catch (...) {}
                }
                auto down_param = parameters.find("down");
                if (down_param != parameters.end()) {
                    try { down = std::stof(down_param->second); } catch (...) {}
                }
                auto yaw_param = parameters.find("yaw");
                if (yaw_param != parameters.end()) {
                    try { yaw = std::stof(yaw_param->second); } catch (...) {}
                }
                
                std::cout << "Moving drone " << drone_id << " relatively: N=" << north 
                         << " E=" << east << " D=" << down << " Yaw=" << yaw << std::endl;
                
                // 使用MAVSDK Mission模式实现相对移动
                try {
                    std::cout << "Executing relative movement: N=" << north 
                             << " E=" << east << " D=" << down << " Yaw=" << yaw << std::endl;
                    
                    // 获取当前位置
                    auto position = it->second->telemetry->position();
                    
                    // 计算目标位置（相对当前位置）
                    double target_lat = position.latitude_deg + (north / 111000.0); // 粗略转换
                    double target_lon = position.longitude_deg + (east / (111000.0 * cos(position.latitude_deg * M_PI / 180.0)));
                    double target_alt = position.absolute_altitude_m - down; // down是正值表示下降
                    
                    // 创建航点任务
                    std::vector<mavsdk::Mission::MissionItem> mission_items;
                    mavsdk::Mission::MissionItem mission_item;
                    mission_item.latitude_deg = target_lat;
                    mission_item.longitude_deg = target_lon;
                    mission_item.relative_altitude_m = target_alt;
                    mission_item.speed_m_s = 5.0f; // 默认速度5m/s
                    mission_item.is_fly_through = true;
                    mission_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                    mission_item.loiter_time_s = 0.0f;
                    mission_item.camera_photo_interval_s = 0.0f;
                    
                    mission_items.push_back(mission_item);
                    
                    // 创建MissionPlan
                    mavsdk::Mission::MissionPlan mission_plan;
                    mission_plan.mission_items = mission_items;
                    
                    // 上传并执行任务
                    auto upload_result = it->second->mission->upload_mission(mission_plan);
                    if (upload_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to upload relative movement mission: " << upload_result << std::endl;
                        return false;
                    }
                    
                    auto start_result = it->second->mission->start_mission();
                    if (start_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to start relative movement mission: " << start_result << std::endl;
                        return false;
                    }
                    
                    std::cout << "Successfully executed relative movement for drone " << drone_id << std::endl;
                    return true;
                } catch (const std::exception& e) {
                    std::cout << "Exception during relative movement: " << e.what() << std::endl;
                    return false;
                }
            }
            else if (command == "move_to") {
                // 绝对坐标飞行
                double latitude = 0.0, longitude = 0.0, altitude = 0.0, yaw = 0.0;
                
                auto lat_param = parameters.find("latitude");
                if (lat_param != parameters.end()) {
                    try { latitude = std::stod(lat_param->second); } catch (...) {}
                }
                auto lon_param = parameters.find("longitude");
                if (lon_param != parameters.end()) {
                    try { longitude = std::stod(lon_param->second); } catch (...) {}
                }
                auto alt_param = parameters.find("altitude");
                if (alt_param != parameters.end()) {
                    try { altitude = std::stod(alt_param->second); } catch (...) {}
                }
                auto yaw_param = parameters.find("yaw");
                if (yaw_param != parameters.end()) {
                    try { yaw = std::stod(yaw_param->second); } catch (...) {}
                }
                
                std::cout << "Moving drone " << drone_id << " to: Lat=" << latitude 
                         << " Lon=" << longitude << " Alt=" << altitude << " Yaw=" << yaw << std::endl;
                
                // 使用MAVSDK Mission模式实现绝对坐标飞行
                try {
                    std::cout << "Executing absolute movement to: Lat=" << latitude 
                             << " Lon=" << longitude << " Alt=" << altitude << " Yaw=" << yaw << std::endl;
                    
                    // 创建航点任务
                    std::vector<mavsdk::Mission::MissionItem> mission_items;
                    mavsdk::Mission::MissionItem mission_item;
                    mission_item.latitude_deg = latitude;
                    mission_item.longitude_deg = longitude;
                    mission_item.relative_altitude_m = altitude;
                    mission_item.speed_m_s = 5.0f; // 默认速度5m/s
                    mission_item.is_fly_through = true;
                    mission_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                    mission_item.loiter_time_s = 0.0f;
                    mission_item.camera_photo_interval_s = 0.0f;
                    
                    mission_items.push_back(mission_item);
                    
                    // 创建MissionPlan
                    mavsdk::Mission::MissionPlan mission_plan;
                    mission_plan.mission_items = mission_items;
                    
                    // 上传并执行任务
                    auto upload_result = it->second->mission->upload_mission(mission_plan);
                    if (upload_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to upload absolute movement mission: " << upload_result << std::endl;
                        return false;
                    }
                    
                    auto start_result = it->second->mission->start_mission();
                    if (start_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to start absolute movement mission: " << start_result << std::endl;
                        return false;
                    }
                    
                    std::cout << "Successfully executed absolute movement for drone " << drone_id << std::endl;
                    return true;
                } catch (const std::exception& e) {
                    std::cout << "Exception during absolute movement: " << e.what() << std::endl;
                    return false;
                }
            }
            else {
                std::cout << "Unknown command: " << command << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cout << "Exception sending command '" << command << "' to drone " << drone_id << ": " << e.what() << std::endl;
            return false;
        }
    }
#endif
    
    // 模拟命令发送
    std::cout << "Simulating command '" << command << "' to drone " << drone_id << std::endl;
    return true;
}

std::optional<ExtendedDroneState> MAVLinkManager::getDroneState(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return std::nullopt;
    }
    
    return it->second->state;
}

void MAVLinkManager::setAuthenticationProvider(std::unique_ptr<AuthenticationProvider> provider) {
    auth_provider_ = std::move(provider);
    std::cout << "Authentication provider set: " << (auth_provider_ ? auth_provider_->getProviderType() : "none") << std::endl;
}

bool MAVLinkManager::authenticateDrone(DroneId drone_id, const std::string& credential, const std::string& credential_type) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        std::cout << "Cannot authenticate unknown drone " << drone_id << std::endl;
        return false;
    }
    
    if (it->second->status != ConnectionStatus::CONNECTED) {
        std::cout << "Cannot authenticate disconnected drone " << drone_id << std::endl;
        return false;
    }
    
    // 存储认证凭据
    it->second->authentication_credential = credential;
    it->second->credential_type = credential_type;
    it->second->status = ConnectionStatus::AUTHENTICATING;
    
    std::cout << "Starting authentication for drone " << drone_id << " with " << credential_type << std::endl;
    
    // 通知状态变化
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::AUTHENTICATING);
    }
    
    return true;
}

bool MAVLinkManager::isDroneAuthenticated(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return false;
    }
    
    return it->second->is_authenticated && it->second->status == ConnectionStatus::AUTHENTICATED;
}

void MAVLinkManager::setStateManager(std::shared_ptr<DroneStateManager> state_manager) {
    state_manager_ = state_manager;
    std::cout << "State manager set" << std::endl;
}

void MAVLinkManager::setAccessControlEngine(std::shared_ptr<AccessControlEngine> engine) {
    access_control_engine_ = engine;
    std::cout << "Access control engine set for MAVLink manager" << std::endl;
}

void MAVLinkManager::runConnectionCycleOnce() {
#ifdef HAVE_MAVSDK
    if (discovery_mode_) {
        runDiscoveryCycleOnce();
        return;
    }
#endif
    std::vector<DroneId> connecting_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTING) {
                connecting_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : connecting_drones) {
        std::unique_ptr<DroneConnection>* conn_ptr = nullptr;
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto it = connections_.find(drone_id);
            if (it == connections_.end()) continue;
            conn_ptr = &it->second;
        }
        bool connection_success = establishConnection(**conn_ptr);
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto it = connections_.find(drone_id);
            if (it != connections_.end()) {
                if (connection_success) {
                    it->second->status = ConnectionStatus::CONNECTED;
                    it->second->retry_count = 0;
                    for (auto& callback : connection_callbacks_) {
                        callback(drone_id, ConnectionStatus::CONNECTED);
                    }
                    if (state_manager_) {
                        state_manager_->setDroneOnline(drone_id);
                        state_manager_->updateDroneState(drone_id, it->second->state);
                    }
                    std::cout << "Successfully connected to drone " << drone_id << std::endl;
                } else {
                    it->second->retry_count++;
                    if (it->second->retry_count >= MAX_RETRY_COUNT) {
                        it->second->status = ConnectionStatus::FAILED;
                        for (auto& callback : connection_callbacks_) {
                            callback(drone_id, ConnectionStatus::FAILED);
                        }
                        std::cout << "Failed to connect to drone " << drone_id
                                  << " after " << MAX_RETRY_COUNT << " retries" << std::endl;
                    }
                }
            }
        }
    }
}

#ifdef HAVE_MAVSDK
static uint64_t systemKey(const std::shared_ptr<mavsdk::System>& sys) {
    uint8_t sysid = sys->get_system_id();
    auto comp_ids = sys->component_ids();
    uint8_t compid = comp_ids.empty() ? 0 : comp_ids[0];
    return (static_cast<uint64_t>(sysid) << 8) | compid;
}

void MAVLinkManager::runDiscoveryCycleOnce() {
    if (!mavsdk_ || !discovery_mode_) return;
    auto systems = mavsdk_->systems();
    std::set<uint64_t> current_keys;
    for (const auto& sys : systems) {
        if (!sys) continue;
        uint64_t key = systemKey(sys);
        current_keys.insert(key);
        std::lock_guard<std::mutex> lock(connections_mutex_);
        if (system_key_to_drone_id_.count(key)) continue;
        DroneId drone_id = generateDroneId();
        auto conn = std::make_unique<DroneConnection>();
        conn->drone_id = drone_id;
        conn->connection_url = discovery_listen_url_;
        conn->status = ConnectionStatus::CONNECTING;
        conn->system = sys;
        if (establishConnectionFromSystem(*conn, sys)) {
            conn->status = ConnectionStatus::CONNECTED;
            conn->last_heartbeat = std::chrono::steady_clock::now();
            system_key_to_drone_id_[key] = drone_id;
            connections_[drone_id] = std::move(conn);
            for (auto& cb : connection_callbacks_) cb(drone_id, ConnectionStatus::CONNECTED);
            if (state_manager_) {
                state_manager_->setDroneOnline(drone_id);
                state_manager_->updateDroneState(drone_id, connections_[drone_id]->state);
            }
            std::cout << "Discovered and connected drone " << drone_id
                      << " (sysid=" << (int)sys->get_system_id() << ")" << std::endl;
        }
    }
    std::vector<DroneId> to_remove;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto it = system_key_to_drone_id_.begin(); it != system_key_to_drone_id_.end(); ) {
            if (current_keys.count(it->first)) { ++it; continue; }
            to_remove.push_back(it->second);
            it = system_key_to_drone_id_.erase(it);
        }
    }
    for (DroneId did : to_remove) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto cit = connections_.find(did);
        if (cit != connections_.end()) {
            if (state_manager_) state_manager_->setDroneOffline(did);
            for (auto& cb : connection_callbacks_) cb(did, ConnectionStatus::DISCONNECTED);
            connections_.erase(cit);
        }
    }
}

bool MAVLinkManager::establishConnectionFromSystem(DroneConnection& conn, const std::shared_ptr<mavsdk::System>& system) {
    if (!system) return false;
    try {
        conn.system = system;
        conn.telemetry = std::make_unique<mavsdk::Telemetry>(system);
        conn.action = std::make_unique<mavsdk::Action>(system);
        conn.mission = std::make_unique<mavsdk::Mission>(system);
        conn.mavlink_passthrough = std::make_unique<mavsdk::MavlinkPassthrough>(system);
        conn.offboard = std::unique_ptr<void, void(*)(void*)>(
            new mavsdk::Offboard(system),
            [](void* p) { delete static_cast<mavsdk::Offboard*>(p); }
        );
        conn.ftp = std::make_unique<mavsdk::Ftp>(system);
        setupMavlinkMessageHandlers(conn);
        conn.state.drone_id = conn.drone_id;
        conn.state.drone_model = "PX4";
        conn.state.owner_organization = "discovered";
        conn.state.last_update = std::chrono::system_clock::now();
        return true;
    } catch (const std::exception& e) {
        std::cout << "establishConnectionFromSystem: " << e.what() << std::endl;
        return false;
    }
}
#endif

void MAVLinkManager::connectionWorker() {
    while (running_.load()) {
        runConnectionCycleOnce();
        std::this_thread::sleep_for(RETRY_INTERVAL);
    }
}

void MAVLinkManager::runTelemetryCycleOnce() {
    if (telemetry_cycle_count_ == 0) {
        last_telemetry_status_report_ = std::chrono::steady_clock::now();
        last_telemetry_callback_time_ = std::chrono::steady_clock::now();
    }
    auto cycle_start = std::chrono::high_resolution_clock::now();
    std::vector<DroneId> connected_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (const auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTED || conn->status == ConnectionStatus::AUTHENTICATED) {
                connected_drones.push_back(drone_id);
            }
        }
    }
    if (connected_drones.empty()) {
        static int empty_count = 0;
        if (empty_count % 100 == 0) {
            std::cout << "📊 遥测线程：等待已认证的无人机..." << std::endl;
        }
        empty_count++;
        telemetry_cycle_count_++;
        return;
    }
    for (DroneId drone_id : connected_drones) {
        ExtendedDroneState state_copy;
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto it = connections_.find(drone_id);
            if (it == connections_.end()) continue;
            auto now = std::chrono::system_clock::now();
            it->second->state.last_update = now;
            it->second->last_heartbeat = std::chrono::steady_clock::now();
            it->second->state.drone_id = drone_id;
#ifdef HAVE_MAVSDK
            if (it->second->telemetry) {
                try {
                    auto position = it->second->telemetry->position();
                    it->second->state.position.latitude = position.latitude_deg;
                    it->second->state.position.longitude = position.longitude_deg;
                    it->second->state.position.altitude = position.relative_altitude_m;
                    if (telemetry_cycle_count_ % 50 == 0) {
                        std::cout << "📊 无人机" << drone_id << " 位置更新: 纬度=" << position.latitude_deg
                                  << ", 经度=" << position.longitude_deg
                                  << ", 相对高度=" << position.relative_altitude_m << "米" << std::endl;
                    }
                    if (telemetry_cycle_count_ % 10 == 0) {
                        auto velocity_ned = it->second->telemetry->velocity_ned();
                        it->second->state.velocity_north = velocity_ned.north_m_s;
                        it->second->state.velocity_east = velocity_ned.east_m_s;
                        it->second->state.velocity_down = velocity_ned.down_m_s;
                        it->second->state.speed = std::sqrt(
                            velocity_ned.north_m_s * velocity_ned.north_m_s +
                            velocity_ned.east_m_s * velocity_ned.east_m_s +
                            velocity_ned.down_m_s * velocity_ned.down_m_s);
                    }
                    if (telemetry_cycle_count_ % 5 == 0) {
                        auto battery = it->second->telemetry->battery();
                        it->second->state.battery_percentage = battery.remaining_percent * 100.0;
                    }
                    if (telemetry_cycle_count_ % 10 == 0) {
                        auto flight_mode = it->second->telemetry->flight_mode();
                        switch (flight_mode) {
                            case mavsdk::Telemetry::FlightMode::Ready:
                                it->second->state.flight_status = FlightStatus::LANDED;
                                break;
                            case mavsdk::Telemetry::FlightMode::Takeoff:
                                it->second->state.flight_status = FlightStatus::TAKING_OFF;
                                break;
                            case mavsdk::Telemetry::FlightMode::Hold:
                            case mavsdk::Telemetry::FlightMode::Mission:
                            case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                            case mavsdk::Telemetry::FlightMode::Offboard:
                            case mavsdk::Telemetry::FlightMode::Manual:
                            case mavsdk::Telemetry::FlightMode::Altctl:
                            case mavsdk::Telemetry::FlightMode::Posctl:
                                it->second->state.flight_status = FlightStatus::IN_AIR;
                                break;
                            case mavsdk::Telemetry::FlightMode::Land:
                                it->second->state.flight_status = FlightStatus::LANDING;
                                break;
                            default:
                                it->second->state.flight_status = FlightStatus::UNKNOWN;
                                break;
                        }
                    }
                    if (telemetry_cycle_count_ % 3 == 0) {
                        it->second->state.is_armed = it->second->telemetry->armed();
                    }
                    if (telemetry_cycle_count_ % 2 == 0) {
                        auto velocity_ned = it->second->telemetry->velocity_ned();
                        it->second->state.velocity_north = velocity_ned.north_m_s;
                        it->second->state.velocity_east = velocity_ned.east_m_s;
                        it->second->state.velocity_down = velocity_ned.down_m_s;
                        it->second->state.ground_speed = std::sqrt(
                            velocity_ned.north_m_s * velocity_ned.north_m_s +
                            velocity_ned.east_m_s * velocity_ned.east_m_s);
                    }
                    if (telemetry_cycle_count_ % 5 == 0) {
                        auto heading = it->second->telemetry->heading();
                        it->second->state.heading_deg = heading.heading_deg;
                    }
                } catch (const std::exception&) {}
            }
#endif
            state_copy = it->second->state;
        }
        if (state_manager_) {
            state_manager_->updateDroneState(drone_id, state_copy);
        }
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_callback = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_telemetry_callback_time_).count();
        if (!telemetry_callbacks_.empty() && time_since_last_callback >= 100) {
            try {
                for (auto& callback : telemetry_callbacks_) {
                    callback(state_copy);
                }
                total_telemetry_callbacks_processed_++;
                last_telemetry_callback_time_ = now;
            } catch (const std::exception& e) {
                std::cout << "[错误] 遥测回调异常: " << e.what() << std::endl;
            }
        }
    }
    telemetry_cycle_count_++;
    auto cycle_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - cycle_start).count();
    if (telemetry_cycle_count_ % 50 == 0) {
        last_telemetry_status_report_ = std::chrono::steady_clock::now();
        std::cout << "📈 遥测性能报告 - 周期: " << telemetry_cycle_count_
                  << ", 周期耗时: " << (cycle_duration / 1000.0) << "ms"
                  << ", 已处理回调: " << total_telemetry_callbacks_processed_
                  << ", 连接无人机数: " << connected_drones.size() << std::endl;
    }
    if (cycle_duration > TELEMETRY_INTERVAL.count() * 1000) {
        std::cout << "[警告]  遥测周期耗时过长: " << (cycle_duration / 1000.0) << "ms" << std::endl;
    }
}

void MAVLinkManager::telemetryWorker() {
    std::cout << "🔄 遥测数据工作线程已启动，更新间隔: " << TELEMETRY_INTERVAL.count() << "ms" << std::endl;
    while (running_.load()) {
        runTelemetryCycleOnce();
        std::this_thread::sleep_for(TELEMETRY_INTERVAL);
    }
    std::cout << "🛑 遥测数据工作线程已停止，总处理周期: " << telemetry_cycle_count_
              << ", 总回调数: " << total_telemetry_callbacks_processed_ << std::endl;
}

void MAVLinkManager::runHeartbeatCycleOnce() {
    auto now = std::chrono::steady_clock::now();
    std::vector<DroneId> lost_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::AUTHENTICATED) {
                auto elapsed = now - conn->last_heartbeat;
                if (elapsed > HEARTBEAT_TIMEOUT) {
                    lost_drones.push_back(drone_id);
                }
            }
        }
    }
    for (DroneId drone_id : lost_drones) {
        handleConnectionLost(drone_id);
    }
}

void MAVLinkManager::heartbeatChecker() {
    while (running_.load()) {
        runHeartbeatCycleOnce();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void MAVLinkManager::runAuthenticationCycleOnce() {
    std::vector<DroneId> authenticating_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::AUTHENTICATING) {
                authenticating_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : authenticating_drones) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(drone_id);
        if (it != connections_.end()) {
            if (performAuthentication(*it->second)) {
                it->second->status = ConnectionStatus::AUTHENTICATED;
                it->second->is_authenticated = true;
                it->second->state.is_authenticated = true;
                it->second->state.authentication_type = it->second->credential_type.empty() ? "unknown" : it->second->credential_type;
                auto attr_org = it->second->extracted_attributes.find("organization");
                if (attr_org != it->second->extracted_attributes.end()) {
                    it->second->state.owner_organization = attr_org->second;
                }
                auto attr_cert = it->second->extracted_attributes.find("certificate_id");
                if (attr_cert != it->second->extracted_attributes.end()) {
                    it->second->state.certificate_id = attr_cert->second;
                }
                auto attr_trust = it->second->extracted_attributes.find("trust_level");
                if (attr_trust != it->second->extracted_attributes.end()) {
                    it->second->state.trust_level = attr_trust->second;
                }
                it->second->state.auth_timestamp = std::chrono::system_clock::now();
                it->second->state.auth_attributes = it->second->extracted_attributes;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::AUTHENTICATED);
                }
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, it->second->state);
                }
                handleAuthenticationResult(drone_id, true, "Authentication successful");
                std::cout << "Successfully authenticated drone " << drone_id << std::endl;
            } else {
                it->second->status = ConnectionStatus::AUTHENTICATION_FAILED;
                it->second->is_authenticated = false;
                it->second->state.is_authenticated = false;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::AUTHENTICATION_FAILED);
                }
                handleAuthenticationResult(drone_id, false, "Authentication failed");
                std::cout << "Authentication failed for drone " << drone_id << std::endl;
            }
        }
    }
}

void MAVLinkManager::authenticationWorker() {
    while (running_.load()) {
        runAuthenticationCycleOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MAVLinkManager::runReconnectionCycleOnce() {
    std::vector<DroneId> reconnecting_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTING && conn->retry_count <= MAX_RETRY_COUNT) {
                reconnecting_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : reconnecting_drones) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(drone_id);
        if (it != connections_.end()) {
            std::cout << "🔄 正在重连无人机 " << drone_id << "..." << std::endl;
            if (establishConnection(*it->second)) {
                it->second->status = ConnectionStatus::CONNECTED;
                it->second->retry_count = 0;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::CONNECTED);
                }
                if (state_manager_) {
                    state_manager_->setDroneOnline(drone_id);
                    state_manager_->updateDroneState(drone_id, it->second->state);
                }
                std::cout << "无人机 " << drone_id << " 重连成功" << std::endl;
            } else {
                std::cout << "[错误] 无人机 " << drone_id << " 重连失败，将在 " << RECONNECT_INTERVAL.count() << " 秒后重试" << std::endl;
            }
        }
    }
}

void MAVLinkManager::reconnectionWorker() {
    while (running_.load()) {
        runReconnectionCycleOnce();
        std::this_thread::sleep_for(RECONNECT_INTERVAL);
    }
}

bool MAVLinkManager::establishConnection(DroneConnection& conn) {
#ifdef HAVE_MAVSDK
    if (!mavsdk_) {
        return false;
    }
    
    try {
        // 添加连接
        auto connection_result = mavsdk_->add_any_connection(conn.connection_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            std::cout << "Failed to add connection: " << connection_result << std::endl;
            return false;
        }
        
        // 等待系统发现，最多等待10秒
        auto systems = mavsdk_->systems();
        int wait_count = 0;
        while (systems.empty() && wait_count < 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            systems = mavsdk_->systems();
            wait_count++;
        }
        
        if (systems.empty()) {
            std::cout << "No systems found after waiting" << std::endl;
            return false;
        }
        
        // 使用第一个发现的系统
        conn.system = systems[0];
        
        // 创建插件实例
        conn.telemetry = std::make_unique<mavsdk::Telemetry>(conn.system);
        conn.action = std::make_unique<mavsdk::Action>(conn.system);
        conn.mission = std::make_unique<mavsdk::Mission>(conn.system);
        conn.mavlink_passthrough = std::make_unique<mavsdk::MavlinkPassthrough>(conn.system);
        conn.offboard = std::unique_ptr<void, void(*)(void*)>(
            new mavsdk::Offboard(conn.system),
            [](void* ptr) { delete static_cast<mavsdk::Offboard*>(ptr); }
        );
        
        // 初始化FTP插件
        conn.ftp = std::make_unique<mavsdk::Ftp>(conn.system);
        
        // 设置MAVLink消息处理器
        setupMavlinkMessageHandlers(conn);
        
        // 完全避免MAVSDK回调机制，改用轮询方式获取所有数据
        // 这样可以完全避免回调队列溢出问题
        std::cout << "[警告]  使用轮询模式获取遥测数据，避免回调队列溢出" << std::endl;
        
        // 其他数据通过轮询获取，避免回调队列积压
        // 这些数据对访问控制不是关键的，可以降低更新频率
        
        // 初始化无人机状态
        conn.state.drone_id = conn.drone_id;
        conn.state.drone_model = "PX4_SITL";
        conn.state.owner_organization = "px4_simulation";
        conn.state.last_update = std::chrono::system_clock::now();
        
        std::cout << "Successfully established MAVLink connection to PX4 SITL" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cout << "Exception in establishConnection: " << e.what() << std::endl;
        return false;
    }
#else
    // 模拟连接建立
    std::cout << "Simulating connection to " << conn.connection_url << std::endl;
    conn.last_heartbeat = std::chrono::steady_clock::now();
    
    // 设置模拟状态
    conn.state.drone_id = conn.drone_id;
    conn.state.position = Position(39.9042, 116.4074, 50.0);  // 示例位置
    conn.state.flight_status = FlightStatus::LANDED;
    conn.state.battery_percentage = 95.0;
    conn.state.is_armed = false;
    conn.state.drone_model = "PX4_SITL";
    conn.state.owner_organization = "test_org";
    
    return true;
#endif
}

void MAVLinkManager::handleConnectionLost(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it != connections_.end()) {
        // 保存连接信息用于重连
        std::string connection_url = it->second->connection_url;
        
        it->second->status = ConnectionStatus::DISCONNECTED;
        it->second->is_authenticated = false;
        
        // 更新状态管理器
        if (state_manager_) {
            state_manager_->setDroneOffline(drone_id);
        }
        
        // 通知连接丢失
        for (auto& callback : connection_callbacks_) {
            callback(drone_id, ConnectionStatus::DISCONNECTED);
        }
        
        std::cout << "Connection lost to drone " << drone_id << std::endl;
        
        // 启动重连逻辑 - 将状态设置为CONNECTING以便重连
        if (it->second->retry_count < MAX_RETRY_COUNT) {
            it->second->status = ConnectionStatus::CONNECTING;
            it->second->retry_count++;
            std::cout << "🔄 尝试重连无人机 " << drone_id << " (第 " << it->second->retry_count << " 次)" << std::endl;
        } else {
            std::cout << "[错误] 无人机 " << drone_id << " 重连次数已达上限，停止重连" << std::endl;
        }
    }
}

void MAVLinkManager::updateDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it != connections_.end()) {
        it->second->state = state;
        it->second->last_heartbeat = std::chrono::steady_clock::now();
    }
}

bool MAVLinkManager::performAuthentication(DroneConnection& conn) {
    if (!auth_provider_) {
        std::cout << "No authentication provider configured, allowing connection" << std::endl;
        return true;  // 如果没有认证提供者，允许连接
    }
    
    if (conn.authentication_credential.empty()) {
        std::cout << "No authentication credential provided for drone " << conn.drone_id << std::endl;
        return false;
    }
    
    try {
        // 创建认证请求
        AuthenticationRequest request;
        request.credential = conn.authentication_credential;
        request.credential_type = conn.credential_type;
        request.drone_id = std::to_string(conn.drone_id);
        
        // 执行认证
        AuthenticationResult result = auth_provider_->authenticate(request);
        
        if (result.success) {
            // 存储提取的属性
            conn.extracted_attributes = result.attributes;
            
            // 更新无人机状态中的属性信息
            if (!result.attributes.empty()) {
                for (const auto& [key, value] : result.attributes) {
                    if (key == "organization") {
                        conn.state.owner_organization = value;
                    } else if (key == "role") {
                        // 可以根据角色设置特权状态
                        if (value == "government" || value == "police" || value == "emergency") {
                            conn.state.is_privileged = true;
                        }
                    }
                }
            }
            // 同步认证结果到状态
            conn.state.is_authenticated = true;
            conn.state.authentication_type = conn.credential_type.empty() ? "unknown" : conn.credential_type;
            // 优先使用返回字段的 trust_level
            if (!result.trust_level.empty()) {
                conn.state.trust_level = result.trust_level;
            } else {
                auto tl = conn.extracted_attributes.find("trust_level");
                if (tl != conn.extracted_attributes.end()) {
                    conn.state.trust_level = tl->second;
                }
            }
            auto cert_it = conn.extracted_attributes.find("certificate_id");
            if (cert_it != conn.extracted_attributes.end()) {
                conn.state.certificate_id = cert_it->second;
            }
            conn.state.auth_timestamp = std::chrono::system_clock::now();
            conn.state.auth_attributes = conn.extracted_attributes;
            
            // 即时写入状态管理器，便于前端展示
            if (state_manager_) {
                state_manager_->updateDroneState(conn.drone_id, conn.state);
            }
            
            std::cout << "Authentication successful for drone " << conn.drone_id 
                      << " (trust level: " << result.trust_level << ")" << std::endl;
            return true;
        } else {
            std::cout << "Authentication failed for drone " << conn.drone_id 
                      << ": " << result.error_message << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cout << "Exception during authentication for drone " << conn.drone_id 
                  << ": " << e.what() << std::endl;
        return false;
    }
}

void MAVLinkManager::handleAuthenticationResult(DroneId drone_id, bool success, const std::string& message) {
    // 通知认证回调
    for (auto& callback : authentication_callbacks_) {
        try {
            callback(drone_id, success, message);
        } catch (const std::exception& e) {
            std::cout << "Exception in authentication callback: " << e.what() << std::endl;
        }
    }
    
    // 如果认证失败，可以选择断开连接
    if (!success) {
        std::cout << "Considering disconnection for unauthenticated drone " << drone_id << std::endl;
        // 这里可以根据策略决定是否断开连接
        // disconnectFromDrone(drone_id);
    }
}

DroneId MAVLinkManager::generateDroneId() {
    return next_drone_id_++;
}

void MAVLinkManager::handleMissionPurposeUpload(uint32_t drone_id, const std::string& mission_name,
                                               const std::string& target_location, const std::string& operation_type,
                                               const std::string& certificate_hash, const std::string& mission_description,
                                               uint64_t /*timestamp*/) {
    MissionPurposeCache cache_entry;
    cache_entry.mission_name = mission_name;
    cache_entry.target_location = target_location;
    cache_entry.operation_type = operation_type;
    cache_entry.certificate_hash = certificate_hash;
    cache_entry.mission_description = mission_description;
    cache_entry.last_evaluated = std::chrono::steady_clock::time_point::min();
    cache_entry.last_decision_granted = false;
    cache_entry.last_decision_reason.clear();

    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        mission_purpose_cache_[drone_id] = cache_entry;
    }

#ifdef HAVE_MAVSDK
    // 发送任务确认命令
    sendMissionConfirmation(drone_id);
    // 等待PX4端处理确认命令（确保状态更新）
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    // 请求证书上传
    requestCertificateUpload(drone_id);
#else
    sendAccessControlResult(drone_id, false, "MAVSDK unavailable");
#endif
}

bool MAVLinkManager::confirmMissionAndRequestCertificate(uint32_t drone_id) {
#ifdef HAVE_MAVSDK
    if (!isDroneConnected(drone_id)) {
        std::cout << "[MAVLink] 无人机 " << drone_id << " 未连接，无法确认任务" << std::endl;
        return false;
    }
    sendMissionConfirmation(drone_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    requestCertificateUpload(drone_id);
    std::cout << "[MAVLink] 已向无人机 " << drone_id << " 发送任务确认并请求证书" << std::endl;
    return true;
#else
    (void)drone_id;
    return false;
#endif
}

AccessRequest MAVLinkManager::constructAccessRequest(uint32_t drone_id, const std::string& mission_name,
                                                   const std::string& target_location, const std::string& operation_type,
                                                   const std::string& certificate_hash, const std::string& mission_description) {
    AccessRequest request;
    
    // 基本信息
    request.drone_id = drone_id;
    request.request_time = std::chrono::system_clock::now();
    request.request_source = "mission_purpose_upload";
    
    // 从任务目的中提取信息
    request.target_location = target_location;
    request.operation_type = operation_type;
    
    // 尝试获取无人机状态信息
    if (state_manager_) {
        auto drone_state = state_manager_->getDroneState(drone_id);
        if (drone_state) {
            request.context["drone_model"] = drone_state->drone_model;
            request.context["owner_organization"] = drone_state->owner_organization;
            request.context["is_privileged"] = drone_state->is_privileged ? "true" : "false";
            request.context["battery_percentage"] = std::to_string(drone_state->battery_percentage);
            request.context["flight_status"] = std::to_string(static_cast<int>(drone_state->flight_status));
            request.context["current_latitude"] = std::to_string(drone_state->position.latitude);
            request.context["current_longitude"] = std::to_string(drone_state->position.longitude);
            request.context["current_altitude"] = std::to_string(drone_state->position.altitude);
        }
    }
    
    // 从任务目的中提取证书信息
    if (!certificate_hash.empty()) {
        request.context["certificate_hash"] = certificate_hash;
    }
    
    // 添加证书内容到访问请求中（如果存在）
    // 注意：这里需要从mission_purpose_cache_中获取证书内容
    // 但由于constructAccessRequest是静态函数，我们需要通过其他方式传递
    
    // 设置任务相关属性
    request.context["mission_name"] = mission_name;
    request.context["mission_description"] = mission_description;
    
    // 根据操作类型设置特定属性
    if (operation_type == "delivery") {
        request.context["operation_category"] = "commercial";
        request.context["payload_type"] = "package";
    } else if (operation_type == "patrol") {
        request.context["operation_category"] = "surveillance";
        request.context["payload_type"] = "camera";
    } else if (operation_type == "emergency") {
        request.context["operation_category"] = "emergency";
        request.context["payload_type"] = "medical";
    } else {
        request.context["operation_category"] = "general";
        request.context["payload_type"] = "unknown";
    }
    
    std::cout << "[AccessControl] 构造访问请求，drone=" << request.drone_id
              << ", operation=" << request.operation_type
              << ", ctx=" << request.context.size() << std::endl;
    
    return request;
}

void MAVLinkManager::setupMavlinkMessageHandlers(DroneConnection& conn) {
#ifdef HAVE_MAVSDK
    if (!conn.mavlink_passthrough) {
        std::cout << "[错误] MAVLink passthrough plugin not available for drone " << conn.drone_id << std::endl;
        return;
    }
    
    // 设置消息接收回调 - 使用标准STATUSTEXT消息
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_STATUSTEXT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            handleMavlinkMessage(drone_id, message);
        }
    );
    
    // 尝试设置一个通用的消息监听器来捕获所有消息
    // 使用心跳消息作为测试，然后检查是否有其他消息
    // 取消高频心跳的控制台输出，避免刷屏
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            (void)drone_id; (void)message;
        }
    );
    
    conn.mavlink_passthrough->subscribe_message(
        200,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            handleMavlinkMessage(drone_id, message);
        }
    );
    
    // 注册自定义消息ID 12921 (MISSION_PURPOSE_UPLOAD)
    conn.mavlink_passthrough->subscribe_message(
        12921,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            std::cout << "[MAVLink] 收到消息ID 12921 (MISSION_PURPOSE_UPLOAD)" << std::endl;
            handleMavlinkMessage(drone_id, message);
        }
    );
    
    std::cout << "[MAVLink] 已在无人机 " << conn.drone_id << " 上注册消息回调" << std::endl;
#else
    std::cout << "[MAVLink] MAVSDK 不可用，无法注册消息回调" << std::endl;
#endif
}

void MAVLinkManager::handleMavlinkMessage(DroneId drone_id, const mavlink_message_t& message) {
#ifdef HAVE_MAVSDK
    // 注意：不要在这里过早返回，否则会阻断 STATUSTEXT 分片聚合
    
    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) { // 处理STATUSTEXT消息
        mavlink_statustext_t statustext;
        mavlink_msg_statustext_decode(&message, &statustext);
        
        // 检查是否已经收到过任务目的消息，如果是则完全忽略后续消息
        // 注意：只有在已经处理完任务目的（包括证书上传）后才忽略
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto cache_it = mission_purpose_cache_.find(drone_id);
            if (cache_it != mission_purpose_cache_.end()) {
                // 检查是否已经有证书内容，如果有说明已经处理完，可以忽略
                if (!cache_it->second.certificate_content.empty()) {
                    // 已收到任务目的消息且证书已下载，完全忽略后续的STATUSTEXT消息
                    return;
                }
                // 如果还没有证书，可能是第一次处理失败，允许重试
            }
        }
        
        // 处理分片的STATUSTEXT消息
        std::string current_text(statustext.text);
        auto now = std::chrono::steady_clock::now();
        
        // 先检查当前消息是否直接包含完整的MISSION_PURPOSE消息（单条消息情况）
        if (current_text.find("MISSION_PURPOSE:") == 0) {
            // 单条完整消息，直接处理
            std::string full_message = current_text;
            
            // 解析任务目的信息
            std::vector<std::string> parts;
            std::stringstream ss(full_message);
            std::string item;
            
            while (std::getline(ss, item, ':')) {
                parts.push_back(item);
            }
            
            if (parts.size() >= 4) {
                // 创建任务目的缓存条目
                MissionPurposeCache cache_entry;
                cache_entry.mission_name = parts.size() > 1 ? parts[1] : "";
                cache_entry.target_location = parts.size() > 2 ? parts[2] : "";
                cache_entry.operation_type = parts.size() > 3 ? parts[3] : "";
                cache_entry.certificate_hash = parts.size() > 4 ? parts[4] : "government_drone_001";
                cache_entry.mission_description = "无人机前往" + cache_entry.target_location + "执行" + cache_entry.operation_type + "任务";
                cache_entry.last_evaluated = std::chrono::steady_clock::now();
                cache_entry.last_decision_granted = false;
                cache_entry.last_decision_reason = "";
                
                {
                    std::lock_guard<std::mutex> lock(connections_mutex_);
                    mission_purpose_cache_[drone_id] = cache_entry;
                    // 清除缓冲区
                    statustext_buffers_.erase(drone_id);
                }
                
                std::cout << "[MAVLink] 收到无人机 " << drone_id << " 的任务目的，目标: " << cache_entry.target_location << std::endl;
                
                // 处理任务目的上传（会发送确认并请求证书上传）
                auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                handleMissionPurposeUpload(drone_id, cache_entry.mission_name, cache_entry.target_location, 
                                         cache_entry.operation_type, cache_entry.certificate_hash, 
                                         cache_entry.mission_description, current_time);
                
                return;
            }
        }
        
        // 处理分片消息（多条消息拼接）
        // 更新或创建缓冲区
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto& buffer = statustext_buffers_[drone_id];
            buffer.buffer += current_text;
            buffer.last_update = now;
        }
        
        // 检查是否包含完整的任务目的消息
        std::string full_message;
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            full_message = statustext_buffers_[drone_id].buffer;
        }
        
        // 检查是否是完整的任务目的消息
        // 注意：消息格式为 "MISSION_PURPOSE:任务名称:目标位置:操作类型:证书哈希"
        if (full_message.find("MISSION_PURPOSE:") == 0) {
            // 解析任务目的信息
            std::vector<std::string> parts;
            std::stringstream ss(full_message);
            std::string item;
            
            while (std::getline(ss, item, ':')) {
                parts.push_back(item);
            }
            
            if (parts.size() >= 4) {  // 至少需要4个字段：MISSION_PURPOSE, 任务名称, 目标位置, 操作类型
                // 创建任务目的缓存条目
                MissionPurposeCache cache_entry;
                cache_entry.mission_name = parts.size() > 1 ? parts[1] : "";
                cache_entry.target_location = parts.size() > 2 ? parts[2] : "";
                cache_entry.operation_type = parts.size() > 3 ? parts[3] : "";
                cache_entry.certificate_hash = parts.size() > 4 ? parts[4] : "government_drone_001";
                cache_entry.mission_description = "无人机前往" + cache_entry.target_location + "执行" + cache_entry.operation_type + "任务";
                cache_entry.last_evaluated = std::chrono::steady_clock::now();
                cache_entry.last_decision_granted = false;
                cache_entry.last_decision_reason = "";
                
                {
                    std::lock_guard<std::mutex> lock(connections_mutex_);
                    mission_purpose_cache_[drone_id] = cache_entry;
                    // 清除缓冲区
                    statustext_buffers_.erase(drone_id);
                }
                
                std::cout << "[MAVLink] 收到无人机 " << drone_id << " 的任务目的，目标: " << cache_entry.target_location << std::endl;
                
                // 处理任务目的上传（会发送确认并请求证书上传）
                auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                handleMissionPurposeUpload(drone_id, cache_entry.mission_name, cache_entry.target_location, 
                                         cache_entry.operation_type, cache_entry.certificate_hash, 
                                         cache_entry.mission_description, current_time);
                
                // 立即返回，不再处理后续消息
                return;
            }
        }
        
        // 检查是否已经收到过任务目的消息，如果是则忽略后续的消息
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto cache_it = mission_purpose_cache_.find(drone_id);
            if (cache_it != mission_purpose_cache_.end()) {
                // 已收到任务目的消息，忽略重复消息（不输出日志避免泛滥）
                return;
            }
        }
        return;
    }
    
    if (message.msgid == 12921) { // MAVLINK_MSG_ID_MISSION_PURPOSE_UPLOAD
        // 检查是否已经开始接收证书，如果是则忽略后续的任务目的消息
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto cache_it = mission_purpose_cache_.find(drone_id);
            if (cache_it != mission_purpose_cache_.end()) {
                // 已开始证书接收流程，忽略重复的任务目的消息（不输出日志避免泛滥）
                return;
            }
        }
        
        // 手动解析消息内容（因为我们没有生成对应的解码函数）
        // 消息结构：timestamp(8) + drone_id(4) + mission_name(32) + target_location(32) + 
        //          operation_type(16) + certificate_hash(64) + mission_description(64)
        const uint8_t* payload = (const uint8_t*)message.payload64;
        
        // 提取时间戳（8字节）
        uint64_t timestamp = 0;
        memcpy(&timestamp, payload, 8);
        payload += 8;
        
        // 提取无人机ID（4字节）
        uint32_t mission_drone_id = 0;
        memcpy(&mission_drone_id, payload, 4);
        payload += 4;
        
        // 提取字符串字段（确保以null结尾）
        char mission_name[33] = {0};
        char target_location[33] = {0};
        char operation_type[17] = {0};
        char certificate_hash[65] = {0};
        char mission_description[65] = {0};
        
        strncpy(mission_name, (const char*)payload, 32);
        payload += 32;
        strncpy(target_location, (const char*)payload, 32);
        payload += 32;
        strncpy(operation_type, (const char*)payload, 16);
        payload += 16;
        strncpy(certificate_hash, (const char*)payload, 64);
        payload += 64;
        strncpy(mission_description, (const char*)payload, 64);
        
        // 调用处理函数
        handleMissionPurposeUpload(
            mission_drone_id,
            std::string(mission_name),
            std::string(target_location),
            std::string(operation_type),
            std::string(certificate_hash),
            std::string(mission_description),
            timestamp
        );
    } else if (message.msgid == 200) { // MAVLINK_MSG_ID_CERTIFICATE_UPLOAD
        // 手动解析消息内容
        const uint8_t* payload = (const uint8_t*)message.payload64;
        
        // 提取字段
        uint32_t cert_drone_id = 0;
        uint32_t certificate_size = 0;
        uint16_t chunk_index = 0;
        uint16_t total_chunks = 0;
        uint64_t timestamp = 0;
        char certificate_data[201] = {0};
        
        memcpy(&cert_drone_id, payload, 4);
        payload += 4;
        memcpy(&certificate_size, payload, 4);
        payload += 4;
        memcpy(&chunk_index, payload, 2);
        payload += 2;
        memcpy(&total_chunks, payload, 2);
        payload += 2;
        memcpy(certificate_data, payload, 200);
        payload += 200;
        memcpy(&timestamp, payload, 8);
        
        // 调用处理函数
        handleCertificateUpload(
            cert_drone_id,
            certificate_size,
            chunk_index,
            total_chunks,
            std::string(certificate_data),
            timestamp
        );
    } else {
        // 其他消息类型可以在这里处理
        std::cout << "📨 收到其他MAVLink消息: ID=" << message.msgid << " from drone " << drone_id << std::endl;
    }
#else
    std::cout << "[警告] MAVSDK 不可用，无法处理 MAVLink 消息" << std::endl;
#endif
}

void MAVLinkManager::handleCertificateUpload(uint32_t drone_id, uint32_t certificate_size, uint16_t chunk_index,
                                            uint16_t total_chunks, const std::string& certificate_data, uint64_t timestamp) {
    std::cout << "处理证书上传消息:" << std::endl;
    std::cout << "   无人机ID: " << drone_id << std::endl;
    std::cout << "   📏 证书大小: " << certificate_size << " 字节" << std::endl;
    std::cout << "   分块信息: " << (chunk_index + 1) << "/" << total_chunks << std::endl;
    std::cout << "   时间戳: " << timestamp << std::endl;
    std::cout << "   当前分块数据长度: " << certificate_data.length() << " 字节" << std::endl;
    std::cout << "   当前分块数据预览: " << certificate_data.substr(0, std::min(50, (int)certificate_data.length())) << "..." << std::endl;

    // 检查是否是第一个分块
    if (chunk_index == 0) {
        std::cout << "   🆕 初始化证书缓存，无人机ID: " << drone_id << std::endl;
        // 初始化证书缓存
        certificate_cache_[drone_id] = std::string();
        certificate_cache_[drone_id].reserve(certificate_size);
        std::cout << "   证书缓存已初始化，预留空间: " << certificate_size << " 字节" << std::endl;
    }

    // 检查缓存是否存在
    auto cache_it = certificate_cache_.find(drone_id);
    if (cache_it == certificate_cache_.end()) {
        std::cout << "[错误] 证书缓存不存在，忽略分块 " << chunk_index << std::endl;
        return;
    }

    // 添加证书数据到缓存
    std::cout << "   添加分块数据到缓存，当前缓存大小: " << cache_it->second.length() << " 字节" << std::endl;
    cache_it->second += certificate_data;
    std::cout << "   添加后缓存大小: " << cache_it->second.length() << " 字节" << std::endl;

    // 检查是否接收完所有分块
    if (chunk_index == total_chunks - 1) {
        // 验证证书完整性
        if (cache_it->second.length() != certificate_size) {
            certificate_cache_.erase(cache_it);
            std::cout << "[MAVLink] 证书大小不匹配，无法验证" << std::endl;
            return;
        }
        std::cout << "[MAVLink] 无人机 " << drone_id << " 证书已完整接收，开始解码验证" << std::endl;

        std::string decoded_cert = base64Decode(cache_it->second);
        if (decoded_cert.empty()) {
            std::cout << "[MAVLink] 证书解码失败" << std::endl;
            certificate_cache_.erase(cache_it);
            return;
        }

        if (auth_provider_) {
            AuthenticationRequest request;
            request.credential = decoded_cert;
            request.credential_type = "x509_pem";
            request.drone_id = std::to_string(drone_id);

            AuthenticationResult result = auth_provider_->authenticate(request);
            
            if (result.success) {
                // 更新无人机连接状态
                auto conn_it = connections_.find(drone_id);
                if (conn_it != connections_.end() && conn_it->second) {
                    conn_it->second->authentication_credential = decoded_cert;
                    conn_it->second->credential_type = "x509_pem";
                    conn_it->second->extracted_attributes = result.attributes;
                    conn_it->second->is_authenticated = true;
                    conn_it->second->status = ConnectionStatus::AUTHENTICATED;
                    
                    // 更新无人机状态
                    if (result.getAttribute("organization") == "government" || 
                        result.getAttribute("organization") == "police" || 
                        result.getAttribute("organization") == "emergency") {
                        conn_it->second->state.is_privileged = true;
                    }
                    conn_it->second->state.owner_organization = result.getAttribute("organization");
                    conn_it->second->state.is_authenticated = true;
                    conn_it->second->state.authentication_type = conn_it->second->credential_type;
                    conn_it->second->state.trust_level = result.trust_level;
                    // 若属性中提供证书ID，则填充
                    auto cert_it = result.attributes.find("certificate_id");
                    if (cert_it != result.attributes.end()) {
                        conn_it->second->state.certificate_id = cert_it->second;
                    }
                    conn_it->second->state.auth_timestamp = std::chrono::system_clock::now();
                    conn_it->second->state.auth_attributes = result.attributes;

                    // 立刻广播连接状态变更与状态同步
                    for (auto& callback : connection_callbacks_) {
                        callback(drone_id, ConnectionStatus::AUTHENTICATED);
                    }
                    if (state_manager_) {
                        state_manager_->updateDroneState(drone_id, conn_it->second->state);
                    }
                }

                // 证书验证成功，现在进行访问控制评估
                // 检查是否有待评估的任务目的
                auto mission_it = mission_purpose_cache_.find(drone_id);
                if (mission_it != mission_purpose_cache_.end()) {
                    performAccessControlEvaluation(drone_id);
                } else {
                    std::cout << "[AccessControl] 无任务目的缓存，跳过评估" << std::endl;
                }
                
                // 存储验证成功的证书
                validated_certificates_[drone_id] = {
                    decoded_cert,
                    std::chrono::system_clock::now(),
                    result.attributes
                };
                
            } else {
                std::cout << "[MAVLink] 证书验证失败: " << result.error_message << std::endl;
            }
        } else {
            std::cout << "[MAVLink] 未配置认证提供者，跳过证书验证" << std::endl;
        }

        // 清理缓存
        certificate_cache_.erase(cache_it);
    }
}

std::string MAVLinkManager::base64Decode(const std::string& encoded) {
    // 简单的base64解码实现
    const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string decoded;
    int val = 0, valb = -8;
    
    for (char c : encoded) {
        if (chars.find(c) == std::string::npos) break;
        val = (val << 6) + chars.find(c);
        valb += 6;
        if (valb >= 0) {
            decoded.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    
    return decoded;
}

void MAVLinkManager::downloadCertificateViaFTP(DroneId drone_id) {
    downloadCertificateViaFTPWithRetry(drone_id, 1);
}

void MAVLinkManager::downloadCertificateViaFTPWithRetry(DroneId drone_id, int max_retries) {
    std::cout << "[MAVLink] 开始通过FTP下载无人机 " << drone_id << " 的证书文件（最多重试 " << max_retries << " 次）..." << std::endl;
    
#ifdef HAVE_MAVSDK
    const std::string remote_file_path = "./fs/microsd/certificates/drone_cert.pem";
    const std::string local_file_path = "/tmp/drone_" + std::to_string(drone_id) + "_cert.pem";
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        std::cout << "[MAVLink] FTP下载尝试 " << attempt << "/" << max_retries << "..." << std::endl;
        
        // 获取连接锁
        std::unique_lock<std::mutex> lock(connections_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            std::cout << "[MAVLink] 无法获取连接锁，等待后重试..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            lock = std::unique_lock<std::mutex>(connections_mutex_, std::try_to_lock);
            if (!lock.owns_lock()) {
                std::cout << "[MAVLink] 重试后仍无法获取连接锁" << std::endl;
                if (attempt < max_retries) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                } else {
                    std::cout << "[MAVLink] 达到最大重试次数，放弃FTP下载" << std::endl;
                    return;
                }
            }
        }
        
        auto it = connections_.find(drone_id);
        if (it == connections_.end()) {
            std::cout << "[MAVLink] 无人机 " << drone_id << " 连接不存在" << std::endl;
            return;
        }
        
        if (!it->second->ftp) {
            std::cout << "[MAVLink] 无人机 " << drone_id << " FTP 未初始化" << std::endl;
            return;
        }
        
        // 获取FTP插件的原始指针，然后释放锁
        auto* ftp_plugin = it->second->ftp.get();
        lock.unlock(); // 释放锁，避免长时间持有
        
        // 使用同步方式下载证书文件（更可靠）
        std::cout << "[MAVLink] 尝试下载证书文件: " << remote_file_path << " -> " << local_file_path << std::endl;
        
        // 使用同步下载，带超时
        std::atomic<bool> download_complete{false};
        std::atomic<bool> download_success{false};
        
        ftp_plugin->download_async(
            remote_file_path,
            local_file_path,
            false, // use_burst = false，使用普通下载模式
            [&download_complete, &download_success, drone_id, local_file_path, this](mavsdk::Ftp::Result result, mavsdk::Ftp::ProgressData progress) {
                if (result == mavsdk::Ftp::Result::Success) {
                    std::cout << "[MAVLink] 无人机 " << drone_id << " 证书文件下载成功" << std::endl;
                    download_success = true;
                    download_complete = true;
                    handleCertificateFileDownloaded(drone_id, local_file_path, true);
                } else if (result == mavsdk::Ftp::Result::ProtocolError || 
                          result == mavsdk::Ftp::Result::InvalidParameter ||
                          result == mavsdk::Ftp::Result::NoSystem) {
                    // 这些错误不应该重试
                    std::cout << "[MAVLink] 证书文件下载失败（不可恢复错误），result=" << static_cast<int>(result) << std::endl;
                    download_success = false;
                    download_complete = true;
                    handleCertificateFileDownloaded(drone_id, local_file_path, false);
                } else {
                    // 其他错误可以重试
                    std::cout << "[MAVLink] 证书文件下载失败（可重试），result=" << static_cast<int>(result) << std::endl;
                    download_success = false;
                    download_complete = true;
                }
            }
        );
        
        // 等待下载完成，最多等待10秒
        auto start_time = std::chrono::steady_clock::now();
        while (!download_complete) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > std::chrono::seconds(10)) {
                std::cout << "[MAVLink] FTP下载超时（10秒）" << std::endl;
                break;
            }
        }
        
        if (download_success) {
            std::cout << "[MAVLink] 证书下载成功完成" << std::endl;
            return; // 成功，退出
        }
        
        // 如果失败且还有重试机会，等待后重试
        if (attempt < max_retries) {
            std::cout << "[MAVLink] 等待 " << (attempt * 2) << " 秒后重试..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(attempt * 2));
        }
    }
    
    std::cout << "[MAVLink] 达到最大重试次数，证书下载失败" << std::endl;
    handleCertificateFileDownloaded(drone_id, local_file_path, false);
#else
    std::cout << "[MAVLink] MAVSDK 不可用，无法执行FTP下载" << std::endl;
    (void)drone_id;
    (void)max_retries;
#endif
}

void MAVLinkManager::handleCertificateFileDownloaded(DroneId drone_id, const std::string& file_path, bool success) {
    if (!success) {
        std::cout << "[MAVLink] 无人机 " << drone_id << " 的证书文件下载失败" << std::endl;
        return;
    }
    
    // 检查下载的路径是否是目录（MAVSDK FTP下载可能创建目录）
    std::string actual_file_path = file_path;
    struct stat path_stat;
    if (stat(file_path.c_str(), &path_stat) == 0) {
        if (S_ISDIR(path_stat.st_mode)) {
            // 如果是目录，查找目录中的实际文件
            actual_file_path = file_path + "/drone_cert.pem";
        }
    }
    
    // 读取证书文件内容
    std::ifstream cert_file(actual_file_path, std::ios::binary);
    if (!cert_file.is_open()) {
        std::cout << "[MAVLink] 无法打开证书文件: " << actual_file_path << std::endl;
        return;
    }
    
    // 读取文件内容
    std::string certificate_data((std::istreambuf_iterator<char>(cert_file)),
                                std::istreambuf_iterator<char>());
    cert_file.close();
    
    if (certificate_data.empty()) {
        std::cout << "[MAVLink] 证书文件为空" << std::endl;
        return;
    }
    
    // 将证书内容存储到缓存中
    if (auto mission_it = mission_purpose_cache_.find(drone_id); mission_it != mission_purpose_cache_.end()) {
        mission_it->second.certificate_content = certificate_data;
        std::cout << "[MAVLink] 已存储无人机 " << drone_id << " 的证书内容，准备评估" << std::endl;
    }
    
    // 触发访问控制评估
    performAccessControlEvaluation(drone_id);
}


} // namespace drone_control