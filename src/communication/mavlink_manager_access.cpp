/**
 * MAVLinkManager 访问控制流程实现
 * 执行评估请求构造、决策处理、状态更新与结果下发。
 */
#include "communication/mavlink_manager.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/access_request.hpp"
#include "access_control/access_decision.hpp"
#include "state/drone_state_manager.hpp"

#include <chrono>
#include <iostream>

namespace drone_control {

void MAVLinkManager::performAccessControlEvaluation(uint32_t drone_id) {
    std::cout << "开始访问控制评估..." << std::endl;

    if (!access_control_engine_) {
        std::cout << "[错误] 访问控制引擎未设置" << std::endl;
        sendAccessControlResult(drone_id, false, "Access control engine not available");
        return;
    }

    auto mission_it = mission_purpose_cache_.find(drone_id);
    if (mission_it == mission_purpose_cache_.end()) {
        std::cout << "[错误] 未找到无人机 " << drone_id << " 的任务目的信息" << std::endl;
        sendAccessControlResult(drone_id, false, "Mission purpose not found");
        return;
    }

    const auto& mission_info = mission_it->second;
    auto conn_it = connections_.find(drone_id);
    if (conn_it == connections_.end() || !conn_it->second) {
        std::cout << "[错误] 无人机 " << drone_id << " 连接不存在" << std::endl;
        sendAccessControlResult(drone_id, false, "Drone connection not found");
        return;
    }

    auto& conn = conn_it->second;
    (void)conn;

    AccessRequest request = constructAccessRequest(
        drone_id,
        mission_info.mission_name,
        mission_info.target_location,
        mission_info.operation_type,
        mission_info.certificate_hash,
        mission_info.mission_description);

    if (!mission_info.certificate_content.empty()) {
        request.context["credential"] = mission_info.certificate_content;
        std::cout << "已将证书内容添加到访问请求中，大小: "
                  << mission_info.certificate_content.size() << " 字节" << std::endl;
    } else {
        std::cout << "[警告] 任务目的缓存中没有证书内容" << std::endl;
    }

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
    std::cout << "   评估时间: "
              << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;

    if (decision.granted) {
        std::cout << "生成的访问控制数据:" << std::endl;
        if (!decision.geofence_signature.empty()) {
            std::cout << "   地理围栏解锁凭证（D.cred）: " << decision.geofence_signature << std::endl;
        } else {
            std::cout << "   [错误] 地理围栏解锁凭证未生成" << std::endl;
        }
        if (decision.flight_plan.has_value()) {
            std::cout << "   飞行计划（D.plan）: " << decision.flight_plan->plan_id << " ("
                      << decision.flight_plan->waypoints.size() << " 个航点)" << std::endl;
        } else {
            std::cout << "   [错误] 飞行计划未生成" << std::endl;
        }
    }

    try {
        auto conn_it2 = connections_.find(drone_id);
        if (conn_it2 != connections_.end() && conn_it2->second) {
            auto& dconn = conn_it2->second;
            if (decision.granted) {
                dconn->is_authenticated = true;
                dconn->status = ConnectionStatus::AUTHENTICATED;
                dconn->state.is_authenticated = true;
                dconn->state.authentication_type =
                    dconn->credential_type.empty() ? "access_control" : dconn->credential_type;
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
                dconn->state.auth_attributes = dconn->extracted_attributes;
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, dconn->state);
                }
                std::cout << "[Access] 无人机 " << drone_id << " 认证通过，trust_level="
                          << dconn->state.trust_level
                          << ", auth_type=" << dconn->state.authentication_type
                          << ", cert_id=" << dconn->state.certificate_id << std::endl;
            } else {
                dconn->is_authenticated = false;
                dconn->state.is_authenticated = false;
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, dconn->state);
                }
                std::cout << "[Access] 无人机 " << drone_id
                          << " 认证被拒绝，原因: " << decision.reason << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[警告] 更新地面端认证状态时异常: " << e.what() << std::endl;
    }

    std::cout << "自动发送访问控制结果给无人机 " << drone_id << std::endl;
    sendAccessControlResult(drone_id, decision.granted, decision.reason);

    if (decision.granted) {
        std::cout << "准备发送访问控制数据给无人机 " << drone_id << "..." << std::endl;
        if (!decision.geofence_signature.empty()) {
            std::cout << "[发送] 地理围栏解锁凭证（D.cred）: " << decision.geofence_signature
                      << std::endl;
            sendGeofenceSignature(drone_id, decision.geofence_signature, decision.validity_duration);
        } else {
            std::cout << "[发送] 地理围栏解锁凭证为空，跳过发送" << std::endl;
        }
        if (decision.flight_plan.has_value()) {
            std::cout << "[发送] 飞行计划（D.plan）: " << decision.flight_plan->plan_id << " ("
                      << decision.flight_plan->waypoints.size() << " 个航点)" << std::endl;
            sendFlightPlan(drone_id, decision.flight_plan.value());
        } else {
            std::cout << "[发送] 飞行计划为空，跳过发送" << std::endl;
        }
    }

    std::cout << "访问控制评估完成，保持缓存以防止重复接收消息" << std::endl;
}

} // namespace drone_control

