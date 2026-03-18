/**
 * 无人机 API 实现
 * 从 MAVLink 与状态管理器读取数据，返回无人机列表、状态列表与单机详情 JSON。
 */
#include "drone_api.hpp"
#include "communication/mavlink_manager.hpp"
#include "state/drone_state_manager.hpp"
#include <sstream>
#include <chrono>

namespace drone_control {
namespace api {

DroneAPI::DroneAPI(std::shared_ptr<MAVLinkManager> mavlink_mgr, 
                   std::shared_ptr<DroneStateManager> state_mgr)
    : mavlink_mgr_(mavlink_mgr), state_mgr_(state_mgr) {
}

std::string DroneAPI::handleDrones(const std::string& path, const std::string& method) {
    // 从 MAVLinkManager 读取真实连接列表；无连接时返回空数组
    std::ostringstream out;
    out << "[";
    bool first = true;
    if (mavlink_mgr_) {
        auto ids = mavlink_mgr_->getConnectedDrones();
        for (auto id : ids) {
            if (!first) out << ","; else first = false;
            // 简化返回：仅返回ID与状态；位置/可信度可由 Telemetry/StateManager 扩展
            out << "{";
            out << "\"drone_id\": \"" << id << "\",";
            out << "\"status\": \"connected\"";
            out << "}";
        }
    }
    out << "]";
    return out.str();
}

std::string DroneAPI::handleDroneStates(const std::string& path, const std::string& method) {
    std::ostringstream out;
    out << "[";
    bool first = true;
    if (mavlink_mgr_) {
        auto ids = mavlink_mgr_->getConnectedDrones();
        for (auto id : ids) {
            auto st = mavlink_mgr_->getDroneState(id);
            if (!st.has_value()) continue;
            const auto& s = st.value();
            if (!first) out << ","; else first = false;
            out << "{";
            out << "\"drone_id\": " << id << ",";
            out << "\"position\": {";
            out << "\"latitude\": " << s.position.latitude << ",";
            out << "\"longitude\": " << s.position.longitude << ",";
            out << "\"altitude\": " << s.position.altitude << "},";
            // 前端以数字判断模式，这里返回整型
            out << "\"flight_mode\": " << static_cast<int>(s.flight_status) << ",";
            out << "\"battery_percentage\": " << s.battery_percentage << ",";
            out << "\"is_armed\": " << (s.is_armed ? "true" : "false") << ",";
            out << "\"is_privileged\": " << (s.is_privileged ? "true" : "false") << ",";
            out << "\"drone_model\": \"" << s.drone_model << "\",";
            out << "\"owner_organization\": \"" << s.owner_organization << "\",";
            out << "\"velocity\": {";
            out << "\"north\": " << s.velocity_north << ",";
            out << "\"east\": " << s.velocity_east << ",";
            out << "\"down\": " << s.velocity_down << ",";
            out << "\"ground_speed\": " << s.ground_speed;
            out << "},";
            out << "\"heading\": " << s.heading_deg << ",";
            out << "\"speed\": " << s.speed << ",";
            // last_update 返回为秒级Unix时间戳（字符串->改为数字更易用）
            out << "\"last_update\": " << std::chrono::duration_cast<std::chrono::seconds>(
                s.last_update.time_since_epoch()).count() << ",";
            
            // 认证状态信息（包含认证属性）
            out << "\"authentication\": {";
            out << "\"is_authenticated\": " << (s.is_authenticated ? "true" : "false") << ",";
            out << "\"authentication_type\": \"" << s.authentication_type << "\",";
            out << "\"trust_level\": \"" << s.trust_level << "\",";
            out << "\"certificate_id\": \"" << s.certificate_id << "\",";
            out << "\"auth_timestamp\": \"" << std::chrono::duration_cast<std::chrono::seconds>(
                s.auth_timestamp.time_since_epoch()).count() << "\",";
            // 输出认证属性，便于前端展示证书与身份详情
            out << "\"attributes\": {";
            {
                bool first_attr = true;
                for (const auto& kv : s.auth_attributes) {
                    if (!first_attr) out << ","; else first_attr = false;
                    out << "\"" << kv.first << "\": \"" << kv.second << "\"";
                }
            }
            out << "}"; // end attributes
            out << "},";
            
            // 签名状态信息
            out << "\"signature\": {";
            out << "\"has_valid_signature\": " << (s.has_valid_signature ? "true" : "false") << ",";
            out << "\"signature_id\": \"" << s.signature_id << "\",";
            out << "\"signature_region\": \"" << s.signature_region << "\",";
            out << "\"signature_operation\": \"" << s.signature_operation << "\",";
            out << "\"signature_expiry\": \"" << std::chrono::duration_cast<std::chrono::seconds>(
                s.signature_expiry.time_since_epoch()).count() << "\"";
            out << "}";
            
            out << "}";
        }
    }
    out << "]";
    return out.str();
}

std::string DroneAPI::handleDroneDetail(const std::string& path, const std::string& method) {
    // 解析末尾ID
    int id = parseDroneIdFromPath(path);
    if (id == -1) {
        return "{}";
    }
    
    if (!mavlink_mgr_) return "{}";
    auto st = mavlink_mgr_->getDroneState(id);
    if (!st.has_value()) return "{}";
    
    const auto& s = st.value();
    std::ostringstream out;
    out << "{";
    out << "\"drone_id\": " << id << ",";
    out << "\"position\": {";
    out << "\"lat\": " << s.position.latitude << ",";
    out << "\"lon\": " << s.position.longitude << ",";
    out << "\"alt\": " << s.position.altitude << "}";
    out << "}";
    return out.str();
}

int DroneAPI::parseDroneIdFromPath(const std::string& path) {
    // 粗略解析：查找最后一个'/' 后的数字
    size_t pos = path.find_last_of('/');
    if (pos == std::string::npos || pos + 1 >= path.size()) {
        return -1;
    }
    try { 
        return std::stoi(path.substr(pos + 1)); 
    } catch (...) { 
        return -1; 
    }
}

} // namespace api
} // namespace drone_control


