/**
 * 飞行控制 API 实现
 * 处理自动飞行请求、各类飞行命令及飞行状态查询，通过飞行控制模块与 MAVLink 下发指令。
 */
#include "flight_api.hpp"
#include "communication/mavlink_manager.hpp"
#include "flight_control/flight_control_module.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <thread>

namespace drone_control {
namespace api {

FlightAPI::FlightAPI(std::shared_ptr<MAVLinkManager> mavlink_mgr, 
                     std::shared_ptr<FlightControlModule> flight_control)
    : mavlink_mgr_(mavlink_mgr), flight_control_(flight_control) {
}

std::string FlightAPI::handleAutoFlightRequest(const std::string& path, const std::string& method) {
    // 检查HTTP方法
    if (method != "POST") {
        return ResponseUtils::createErrorResponse("Method not allowed");
    }
    
    std::cout << "🚁 [FlightAPI] 收到自动飞行任务请求" << std::endl;
    
    // 执行从桂园餐厅到蕙园9栋的飞行任务
    bool mission_success = executePatrolMission();
    
    // 创建自动飞行任务响应
    std::ostringstream response;
    response << "{";
    response << "\"status\": " << (mission_success ? "\"success\"" : "\"failed\"") << ",";
    response << "\"message\": " << (mission_success ? "\"送餐任务执行成功\"" : "\"送餐任务执行失败\"") << ",";
    response << "\"task_id\": \"patrol_mission_001\",";
    response << "\"route\": \"桂园餐厅 → 蕙园9栋\",";
    response << "\"timestamp\": \"" << ResponseUtils::getCurrentTimestamp() << "\"";
    response << "}";
    
    std::cout << (mission_success ? "✅ [FlightAPI] 送餐任务执行成功" : "❌ [FlightAPI] 送餐任务执行失败") << std::endl;
    
    return response.str();
}

std::string FlightAPI::handleFlightCommand(const std::string& path, const std::string& method) {
    // 检查HTTP方法
    if (method != "POST") {
        return ResponseUtils::createErrorResponse("Method not allowed");
    }
    
    // 解析路径获取无人机ID和命令
    int drone_id;
    std::string command;
    if (!parseCommandFromPath(path, drone_id, command)) {
        return ResponseUtils::createErrorResponse("Invalid path format");
    }
    
    // 检查无人机连接状态
    if (!isDroneConnected(drone_id)) {
        return ResponseUtils::createErrorResponse("Drone not connected");
    }
    
    // 准备命令参数
    std::map<std::string, std::string> parameters;
    
    // 为起飞命令添加默认高度参数
    if (command == "takeoff") {
        parameters["altitude"] = "10.0"; // 默认高度10米
    }
    
    // 发送MAVLink命令
    bool success = mavlink_mgr_->sendCommand(drone_id, command, parameters);
    
    // 构建响应JSON
    std::ostringstream response;
    response << "{";
    response << "\"drone_id\": " << drone_id << ",";
    response << "\"command\": \"" << command << "\",";
    
    // 为起飞命令添加高度信息到响应中
    if (command == "takeoff") {
        response << "\"altitude\": 10.0,";
    }
    
    response << "\"success\": " << (success ? "true" : "false") << ",";
    response << "\"timestamp\": \"" << ResponseUtils::getCurrentTimestamp() << "\"";
    response << "}";
    
    return response.str();
}

std::string FlightAPI::handleTakeoff(const std::string& path, const std::string& method) {
    // 检查HTTP方法
    if (method != "POST") {
        return ResponseUtils::createErrorResponse("Method not allowed");
    }
    
    // 解析起飞命令路径: /api/v1/drones/{id}/takeoff
    std::istringstream path_stream(path);
    std::string segment;
    std::vector<std::string> segments;
    
    // 按'/'分割路径
    while (std::getline(path_stream, segment, '/')) {
        if (!segment.empty()) {
            segments.push_back(segment);
        }
    }
    
    // 验证路径格式: api/v1/drones/{id}/takeoff
    if (segments.size() < 5 || segments[0] != "api" || segments[1] != "v1" || 
        segments[2] != "drones" || segments[4] != "takeoff") {
        return ResponseUtils::createErrorResponse("Invalid path format");
    }
    
    // 解析无人机ID
    int drone_id = 0;
    try {
        drone_id = std::stoi(segments[3]);
    } catch (const std::exception& e) {
        return ResponseUtils::createErrorResponse("Invalid drone ID");
    }
    
    // 检查无人机连接状态
    if (!isDroneConnected(drone_id)) {
        return ResponseUtils::createErrorResponse("Drone not connected");
    }
    
    // 准备起飞命令参数
    std::map<std::string, std::string> parameters;
    parameters["altitude"] = "10.0"; // 默认高度10米
    
    // 发送起飞命令
    bool success = mavlink_mgr_->sendCommand(drone_id, "takeoff", parameters);
    
    // 构建响应JSON
    std::ostringstream response;
    response << "{";
    response << "\"drone_id\": " << drone_id << ",";
    response << "\"command\": \"takeoff\",";
    response << "\"altitude\": 10.0,";
    response << "\"success\": " << (success ? "true" : "false") << ",";
    response << "\"timestamp\": \"" << ResponseUtils::getCurrentTimestamp() << "\"";
    response << "}";
    
    return response.str();
}

std::string FlightAPI::handleFlightCommandWithParams(const std::string& path, const std::string& method, const std::string& body) {
    // 检查HTTP方法
    if (method != "POST") {
        return ResponseUtils::createErrorResponse("Method not allowed");
    }
    
    // 解析路径获取无人机ID和命令
    int drone_id;
    std::string command;
    if (!parseCommandFromPath(path, drone_id, command)) {
        return ResponseUtils::createErrorResponse("Invalid path format");
    }
    
    // 检查无人机连接状态
    if (!isDroneConnected(drone_id)) {
        return ResponseUtils::createErrorResponse("Drone not connected");
    }
    
    // 解析JSON请求体中的参数
    std::map<std::string, std::string> parameters = parseJsonParameters(body);
    
    // 发送带参数的MAVLink命令
    bool success = mavlink_mgr_->sendCommand(drone_id, command, parameters);
    
    // 构建响应JSON，包含参数信息
    std::ostringstream response;
    response << "{";
    response << "\"drone_id\": " << drone_id << ",";
    response << "\"command\": \"" << command << "\",";
    response << "\"parameters\": {";
    bool first = true;
    for (const auto& param : parameters) {
        if (!first) response << ",";
        response << "\"" << param.first << "\": \"" << param.second << "\"";
        first = false;
    }
    response << "},";
    response << "\"success\": " << (success ? "true" : "false") << ",";
    response << "\"timestamp\": \"" << ResponseUtils::getCurrentTimestamp() << "\"";
    response << "}";
    
    return response.str();
}

std::string FlightAPI::handleFlightStatus(const std::string& path, const std::string& method) {
    // 检查HTTP方法
    if (method != "GET") {
        return ResponseUtils::createErrorResponse("Method not allowed");
    }
    
    // 构建飞行状态响应
    std::ostringstream response;
    response << "{";
    response << "\"status\": \"success\",";
    response << "\"flight_status\": \"active\",";
    response << "\"active_drones\": 1,";
    response << "\"timestamp\": \"" << ResponseUtils::getCurrentTimestamp() << "\"";
    response << "}";
    
    return response.str();
}

bool FlightAPI::parseCommandFromPath(const std::string& path, int& drone_id, std::string& command) {
    // 解析路径格式: /api/v1/drones/{id}/command/{command}
    std::istringstream path_stream(path);
    std::string segment;
    std::vector<std::string> segments;
    
    // 按'/'分割路径
    while (std::getline(path_stream, segment, '/')) {
        if (!segment.empty()) {
            segments.push_back(segment);
        }
    }
    
    // 验证路径格式: api/v1/drones/{id}/command/{command}
    if (segments.size() < 6 || segments[0] != "api" || segments[1] != "v1" || 
        segments[2] != "drones" || segments[4] != "command") {
        return false;
    }
    
    // 解析无人机ID和命令
    try {
        drone_id = std::stoi(segments[3]);
        command = segments[5];
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

std::map<std::string, std::string> FlightAPI::parseJsonParameters(const std::string& body) {
    std::map<std::string, std::string> parameters;
    
    // 简单的JSON解析（仅处理基本键值对格式）
    std::istringstream stream(body);
    std::string line;
    
    // 逐行解析JSON
    while (std::getline(stream, line)) {
        // 查找键值对分隔符
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);
            
            // 清理键和值中的空白字符和引号
            key.erase(0, key.find_first_not_of(" \t\""));
            key.erase(key.find_last_not_of(" \t\"") + 1);
            value.erase(0, value.find_first_not_of(" \t\""));
            value.erase(value.find_last_not_of(" \t\",}") + 1);
            
            // 添加非空的键值对
            if (!key.empty() && !value.empty()) {
                parameters[key] = value;
            }
        }
    }
    
    return parameters;
}

bool FlightAPI::isDroneConnected(int drone_id) {
    // 检查MAVLink连接状态
    if (!mavlink_mgr_) {
        std::cout << "❌ [FlightAPI] MAVLinkManager 不可用" << std::endl;
        return false;
    }
    
    // 检查连接状态
    auto status = mavlink_mgr_->getConnectionStatus(drone_id);
    bool is_connected = (status == MAVLinkManager::ConnectionStatus::CONNECTED || 
                        status == MAVLinkManager::ConnectionStatus::AUTHENTICATED);
    
    std::cout << "🔍 [FlightAPI] 检查无人机 " << drone_id << " 连接状态: " 
              << static_cast<int>(status) << " (连接: " << (is_connected ? "是" : "否") << ")" << std::endl;
    
    return is_connected;
}
//示例
bool FlightAPI::executePatrolMission() {
    std::cout << "🚁 [FlightAPI] 启动送餐任务：桂园餐厅 → 蕙园9栋" << std::endl;
    
    if (!flight_control_) {
        std::cout << "❌ [FlightAPI] FlightControlModule 不可用" << std::endl;
        return false;
    }
    
    // 准备任务参数
    std::map<std::string, std::string> parameters;
    parameters["start_lat"] = "31.7653";
    parameters["start_lon"] = "117.178";
    parameters["end_lat"] = "31.7759";
    parameters["end_lon"] = "117.181";
    parameters["altitude"] = "30.0";
    parameters["mission_name"] = "送餐任务";
    
    // 调用 FlightControlModule 执行任务
    bool success = flight_control_->executeMission(1, "delivery", parameters);
    
    if (success) {
        std::cout << "✅ [FlightAPI] 送餐任务已启动，在后台异步执行" << std::endl;
    } else {
        std::cout << "❌ [FlightAPI] 送餐任务启动失败" << std::endl;
    }
    
    return success;
}



void FlightAPI::waitForPositionReached(int drone_id, double target_lat, double target_lon, double target_alt, double timeout_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    const double horizontal_tolerance = 20.0; // 水平距离容差20米
    const double vertical_tolerance = 5.0;    // 垂直高度容差5米
    const int min_wait_seconds = 5;           // 最少等待5秒
    const int max_wait_seconds = 3000;         // 最多等待300秒（50分钟）
    
    std::cout << "   🎯 等待到达目标位置: 纬度=" << target_lat << ", 经度=" << target_lon << ", 高度=" << target_alt << "米" << std::endl;
    std::cout << "   📏 容差设置: 水平距离<" << horizontal_tolerance << "米, 高度差<" << vertical_tolerance << "米" << std::endl;
    
    bool position_reached = false;
    int consecutive_reached = 0; // 连续到达次数
    const int required_consecutive = 3; // 需要连续3次都到达才认为真正到达
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        // 检查超时
        if (elapsed >= max_wait_seconds) {
            std::cout << "   ⚠️ 等待超时(" << max_wait_seconds << "秒)，继续执行下一个航点" << std::endl;
            break;
        }
        
        if (flight_control_) {
            auto position = flight_control_->getCurrentPosition(drone_id);
            if (position) {
                // 计算水平距离（使用更精确的Haversine公式）
                double lat1_rad = target_lat * M_PI / 180.0;
                double lat2_rad = position->latitude * M_PI / 180.0;
                double delta_lat = (position->latitude - target_lat) * M_PI / 180.0;
                double delta_lon = (position->longitude - target_lon) * M_PI / 180.0;
                
                double a = std::sin(delta_lat/2) * std::sin(delta_lat/2) +
                          std::cos(lat1_rad) * std::cos(lat2_rad) *
                          std::sin(delta_lon/2) * std::sin(delta_lon/2);
                double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
                double distance = 6371000 * c; // 地球半径6371km
                
                double alt_diff = std::abs(position->altitude - target_alt);
                
                std::cout << "   📊 当前位置: 纬度=" << std::fixed << std::setprecision(6) << position->latitude 
                          << ", 经度=" << position->longitude 
                          << ", 高度=" << std::setprecision(1) << position->altitude << "米" << std::endl;
                std::cout << "   📊 距离目标: " << std::setprecision(0) << distance << "米, 高度差: " << alt_diff << "米" << std::endl;
                
                // 检查是否到达目标位置
                bool current_reached = (distance < horizontal_tolerance && alt_diff < vertical_tolerance);
                
                if (current_reached) {
                    consecutive_reached++;
                    std::cout << "   ✅ 位置接近目标 (连续" << consecutive_reached << "/" << required_consecutive << "次)" << std::endl;
                    
                    if (consecutive_reached >= required_consecutive) {
                        position_reached = true;
                        std::cout << "   🎯 已稳定到达目标位置！" << std::endl;
                        break;
                    }
                } else {
                    consecutive_reached = 0; // 重置连续计数
                    if (elapsed >= min_wait_seconds) {
                        std::cout << "   ⏳ 继续等待到达目标位置..." << std::endl;
                    }
                }
            } else {
                std::cout << "   ⚠️ 无法获取无人机位置信息" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (position_reached) {
        std::cout << "   🎉 位置验证成功，继续执行下一步" << std::endl;
    } else {
        std::cout << "   ⚠️ 位置验证未完成，但继续执行下一步" << std::endl;
    }
}

std::string FlightAPI::getMissionStatus() {
    if (!flight_control_) {
        return R"({"status": "error", "message": "FlightControlModule not available"})";
    }
    
    return flight_control_->getMissionStatus(1);
}

} // namespace api
} // namespace drone_control
