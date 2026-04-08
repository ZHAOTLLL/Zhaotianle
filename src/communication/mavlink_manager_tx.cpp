/**
 * MAVLinkManager 消息发送
 * 聚合任务确认、证书请求、访问结果、围栏凭证、飞行计划下发逻辑。
 */
#include "communication/mavlink_manager.hpp"
#include "flight_control/flight_plan.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#ifdef HAVE_MAVSDK
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/common/mavlink.h>
#endif

namespace drone_control {

#ifdef HAVE_MAVSDK
namespace {

mavsdk::MavlinkPassthrough::Result sendCommandLongQueued(
    mavsdk::MavlinkPassthrough& passthrough,
    uint16_t command,
    float param1,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f,
    float param5 = 0.0f,
    float param6 = 0.0f,
    float param7 = 0.0f,
    uint8_t target_system = 1,
    uint8_t target_component = MAV_COMP_ID_AUTOPILOT1,
    uint8_t confirmation = 0)
{
    return passthrough.queue_message([=](MavlinkAddress address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_command_long_pack_chan(
            address.system_id,
            address.component_id,
            channel,
            &message,
            target_system,
            target_component,
            command,
            confirmation,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7);
        return message;
    });
}

} // namespace
#endif

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

    auto result = sendCommandLongQueued(*it->second->mavlink_passthrough, MAV_CMD_USER_1, 1.0f);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "[MAVLink] 已向无人机 " << drone_id << " 发送任务确认" << std::endl;
    } else {
        std::cout << "[MAVLink] 任务确认发送失败，无人机 " << drone_id
                  << ", result=" << static_cast<int>(result) << std::endl;
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

    auto result = sendCommandLongQueued(*it->second->mavlink_passthrough, MAV_CMD_USER_2, 1.0f);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "[MAVLink] 已请求无人机 " << drone_id << " 上传证书" << std::endl;
        std::cout << "[MAVLink] 等待PX4端准备证书文件..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        std::thread ftp_thread([this, drone_id]() { downloadCertificateViaFTPWithRetry(drone_id); });
        ftp_thread.detach();
    } else {
        std::cout << "[MAVLink] 证书上传请求失败，无人机 " << drone_id
                  << ", result=" << static_cast<int>(result) << std::endl;
    }
#else
    (void)drone_id;
#endif
}

void MAVLinkManager::sendAccessControlResult(
    uint32_t drone_id, bool access_granted, const std::string& reason) {
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

    auto result = sendCommandLongQueued(
        *conn->mavlink_passthrough, MAV_CMD_USER_3, access_granted ? 1.0f : 0.0f);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "访问控制结果已发送给无人机 " << drone_id << ": "
                  << (access_granted ? "ACCESS_GRANTED" : "ACCESS_DENIED") << std::endl;
        std::cout << "   原因: " << reason << std::endl;
    } else {
        std::cout << "[错误] 发送访问控制结果失败: " << static_cast<int>(result) << std::endl;
    }
#else
    (void)drone_id;
    (void)access_granted;
    (void)reason;
    std::cout << "[警告] MAVSDK 不可用，无法发送访问控制结果" << std::endl;
#endif
}

void MAVLinkManager::sendGeofenceSignature(
    uint32_t drone_id, const std::string& signature, const std::chrono::seconds& validity_duration) {
#ifdef HAVE_MAVSDK
    std::lock_guard<std::mutex> lock(connections_mutex_);

    auto it = connections_.find(drone_id);
    if (it == connections_.end() || !it->second->mavlink_passthrough) {
        std::cout << "[错误] 无人机 " << drone_id
                  << " 未连接或MAVLink通道未建立，无法发送地理围栏解锁凭证" << std::endl;
        return;
    }

    auto result = sendCommandLongQueued(
        *it->second->mavlink_passthrough,
        31013,
        1.0f,
        static_cast<float>(validity_duration.count()));
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "地理围栏解锁凭证命令已发送给无人机 " << drone_id
                  << "，有效期: " << validity_duration.count() << " 秒" << std::endl;
        std::cout << "   签名长度: " << signature.length() << " 字节" << std::endl;
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
        std::cout << "[错误] 无人机 " << drone_id
                  << " 未连接或Mission插件未初始化，无法发送飞行计划" << std::endl;
        return;
    }

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
        mission_items.push_back(item);
    }

    if (mission_items.empty()) {
        std::cout << "[警告] 飞行计划为空，无法发送" << std::endl;
        return;
    }

    mavsdk::Mission::MissionPlan mission_plan;
    mission_plan.mission_items = mission_items;
    std::cout << "上传飞行计划到无人机 " << drone_id << "，包含 " << mission_items.size()
              << " 个航点" << std::endl;

    auto upload_result = it->second->mission->upload_mission(mission_plan);
    if (upload_result == mavsdk::Mission::Result::Success) {
        std::cout << "飞行计划上传成功" << std::endl;
        auto start_result = it->second->mission->start_mission();
        if (start_result == mavsdk::Mission::Result::Success) {
            std::cout << "飞行计划已启动" << std::endl;
        } else {
            std::cout << "[警告] 飞行计划上传成功但启动失败: "
                      << static_cast<int>(start_result) << std::endl;
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

} // namespace drone_control

