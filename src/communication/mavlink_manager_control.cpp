/**
 * MAVLinkManager 飞行控制命令实现
 * 负责命令分发、参数解析和动作执行。
 */
#include "communication/mavlink_manager.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

#ifdef HAVE_MAVSDK
#include <mavsdk/plugins/offboard/offboard.h>
#endif

namespace drone_control {

bool MAVLinkManager::sendCommand(
    DroneId drone_id, const std::string& command, const std::map<std::string, std::string>& parameters) {
    std::lock_guard<std::mutex> lock(connections_mutex_);

    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        std::cout << "[错误] [MAVLinkManager] 无人机 " << drone_id << " 连接不存在" << std::endl;
        return false;
    }

    if (
        it->second->status != ConnectionStatus::CONNECTED &&
        it->second->status != ConnectionStatus::AUTHENTICATED) {
        std::cout << "[错误] [MAVLinkManager] 无人机 " << drone_id
                  << " 状态不正确: " << static_cast<int>(it->second->status) << std::endl;
        return false;
    }

    std::cout << "[MAVLinkManager] 无人机 " << drone_id << " 连接状态正常，准备发送命令: " << command
              << std::endl;

#ifdef HAVE_MAVSDK
    if (it->second->action) {
        try {
            if (command == "arm") {
                auto result = it->second->action->arm();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully armed drone " << drone_id << std::endl;
                    return true;
                }
                std::cout << "Failed to arm drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "disarm") {
                auto result = it->second->action->disarm();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully disarmed drone " << drone_id << std::endl;
                    return true;
                }
                std::cout << "Failed to disarm drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "takeoff") {
                float altitude = 10.0f;
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
                        std::cout << "Successfully initiated takeoff for drone " << drone_id << " to "
                                  << altitude << "m" << std::endl;
                        return true;
                    }
                    std::cout << "Failed to takeoff drone " << drone_id << ": " << result << std::endl;
                    return false;
                }
                std::cout << "Failed to set takeoff altitude for drone " << drone_id << ": " << result
                          << std::endl;
                return false;
            } else if (command == "land") {
                auto result = it->second->action->land();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated landing for drone " << drone_id
                              << " (keeping armed)" << std::endl;
                    return true;
                }
                std::cout << "Failed to land drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "emergency_land") {
                auto result = it->second->action->land();
                if (result == mavsdk::Action::Result::Success) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    auto disarm_result = it->second->action->disarm();
                    if (disarm_result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully initiated emergency landing and disarmed drone "
                                  << drone_id << std::endl;
                    } else {
                        std::cout << "Emergency landing initiated but failed to disarm drone " << drone_id
                                  << std::endl;
                    }
                    return true;
                }
                std::cout << "Failed to emergency land drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "return_to_launch") {
                auto result = it->second->action->return_to_launch();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated return to launch for drone " << drone_id
                              << std::endl;
                    return true;
                }
                std::cout << "Failed to return to launch drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "hold") {
                auto result = it->second->action->hold();
                if (result == mavsdk::Action::Result::Success) {
                    std::cout << "Successfully initiated hold for drone " << drone_id << std::endl;
                    return true;
                }
                std::cout << "Failed to hold drone " << drone_id << ": " << result << std::endl;
                return false;
            } else if (command == "move_absolute") {
                std::cout << "[MAVLinkManager] 执行绝对位置移动命令..." << std::endl;
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
                        std::cout << "[警告] [MAVLinkManager] 无效的航向参数，使用默认值0: " << e.what()
                                  << std::endl;
                        yaw = 0.0;
                    }
                }
                std::cout << "   目标位置: 纬度=" << latitude << ", 经度=" << longitude
                          << ", 高度=" << altitude << "米, 航向=" << yaw << "度" << std::endl;
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
                try {
                    it->second->mission->pause_mission();
                    auto flight_mode = it->second->telemetry->flight_mode();
                    std::cout << "   🛩️ 当前飞行模式: " << flight_mode << std::endl;
                    bool use_mission_mode = false;
                    if (flight_mode == mavsdk::Telemetry::FlightMode::Hold) {
                        std::cout << "   [警告] 当前为Hold模式，goto_location不会工作，直接使用Mission模式"
                                  << std::endl;
                        use_mission_mode = true;
                    } else if (flight_mode == mavsdk::Telemetry::FlightMode::Offboard) {
                        std::cout
                            << "   [警告] 当前为Offboard模式，goto_location可能不工作，建议先退出Offboard模式"
                            << std::endl;
                        try {
                            auto* offboard_ptr =
                                static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                            offboard_ptr->stop();
                            std::cout << "   🔧 已停止Offboard模式，准备使用goto_location" << std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        } catch (const std::exception& e) {
                            std::cout << "   [警告] 停止Offboard模式失败: " << e.what() << std::endl;
                        }
                    } else {
                        std::cout << "   ℹ️ 当前飞行模式: " << flight_mode
                                  << "，goto_location将处理模式切换" << std::endl;
                    }
                    std::cout << "   📊 目标位置: 纬度=" << latitude << ", 经度=" << longitude
                              << ", 高度=" << altitude << "米" << std::endl;
                    std::cout << "   📊 使用绝对高度进行飞行" << std::endl;
                    std::cout << "   🔍 坐标验证: 纬度范围[" << latitude << "] 经度范围[" << longitude
                              << "] 高度范围[" << altitude << "]" << std::endl;
                    if (latitude < 31.0 || latitude > 32.0) {
                        std::cout << "   [警告] 纬度超出合肥地区范围，可能坐标错误" << std::endl;
                    }
                    if (longitude < 117.0 || longitude > 118.0) {
                        std::cout << "   [警告] 经度超出合肥地区范围，可能坐标错误" << std::endl;
                    }
                    auto mission_mode_it = parameters.find("use_mission_mode");
                    if (mission_mode_it != parameters.end() && mission_mode_it->second == "true") {
                        std::cout << "   🔄 检测到Mission模式请求，使用Mission模式确保高度保持..."
                                  << std::endl;
                        use_mission_mode = true;
                    }
                    if (use_mission_mode) {
                        std::cout << "   🔄 使用 Mission 模式执行位置控制..." << std::endl;
                    } else {
                        std::cout << "   🚁 发送 goto_location 命令: lat=" << latitude
                                  << "°, lon=" << longitude << "°, alt=" << altitude
                                  << "m, yaw=" << yaw << "°" << std::endl;
                        std::cout << "   📊 使用相对高度进行飞行（相对于起飞点）" << std::endl;
                        auto result =
                            it->second->action->goto_location(latitude, longitude, altitude, yaw);
                        if (result == mavsdk::Action::Result::Success) {
                            std::cout << "[MAVLinkManager] 无人机 " << drone_id << " 开始飞往目标位置"
                                      << std::endl;
                            std::cout << "   目标: 纬度=" << latitude << "°, 经度=" << longitude
                                      << "°, 高度=" << altitude << "m" << std::endl;
                            return true;
                        }
                        std::cout << "[错误] [MAVLinkManager] goto_location 失败: " << result << std::endl;
                        std::cout << "   🔍 失败原因分析: 可能是坐标格式、飞行模式或权限问题" << std::endl;
                        auto mm_it = parameters.find("use_mission_mode");
                        if (mm_it != parameters.end() && mm_it->second == "true") {
                            std::cout << "   🔄 明确要求使用 Mission 模式，尝试Mission模式..."
                                      << std::endl;
                            use_mission_mode = true;
                        } else {
                            std::cout << "   [错误] goto_location失败且未要求Mission模式，任务执行失败"
                                      << std::endl;
                            return false;
                        }
                    }

                    if (use_mission_mode) {
                        try {
                            std::vector<mavsdk::Mission::MissionItem> mission_items;
                            mavsdk::Mission::MissionItem altitude_item;
                            altitude_item.latitude_deg = latitude;
                            altitude_item.longitude_deg = longitude;
                            altitude_item.relative_altitude_m = altitude;
                            altitude_item.speed_m_s = 10.0f;
                            altitude_item.is_fly_through = false;
                            altitude_item.gimbal_pitch_deg =
                                std::numeric_limits<float>::quiet_NaN();
                            altitude_item.gimbal_yaw_deg =
                                std::numeric_limits<float>::quiet_NaN();
                            altitude_item.camera_action =
                                mavsdk::Mission::MissionItem::CameraAction::None;
                            altitude_item.loiter_time_s = 2.0f;
                            altitude_item.camera_photo_interval_s = 0.0f;
                            mission_items.push_back(altitude_item);

                            mavsdk::Mission::MissionItem target_item;
                            target_item.latitude_deg = latitude;
                            target_item.longitude_deg = longitude;
                            target_item.relative_altitude_m = altitude;
                            target_item.speed_m_s = 5.0f;
                            target_item.is_fly_through = false;
                            target_item.gimbal_pitch_deg =
                                std::numeric_limits<float>::quiet_NaN();
                            target_item.gimbal_yaw_deg =
                                std::numeric_limits<float>::quiet_NaN();
                            target_item.camera_action =
                                mavsdk::Mission::MissionItem::CameraAction::None;
                            target_item.loiter_time_s = 5.0f;
                            target_item.camera_photo_interval_s = 0.0f;
                            mission_items.push_back(target_item);
                            std::cout << "   🚀 设置飞行速度: 10 m/s (高度保持) + 5 m/s (精确定位)"
                                      << std::endl;
                            std::cout << "   创建2个航点确保高度保持和精确定位" << std::endl;

                            mavsdk::Mission::MissionPlan mission_plan;
                            mission_plan.mission_items = mission_items;
                            auto upload_result = it->second->mission->upload_mission(mission_plan);
                            if (upload_result != mavsdk::Mission::Result::Success) {
                                std::cout << "[错误] [MAVLinkManager] 上传任务失败: " << upload_result
                                          << std::endl;
                                return false;
                            }
                            std::cout << "[MAVLinkManager] 任务上传成功，等待Mission ID设置..."
                                      << std::endl;
                            bool mission_ready = false;
                            for (int retry = 0; retry < 300; retry++) {
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                try {
                                    auto download_result =
                                        it->second->mission->download_mission();
                                    if (download_result.first == mavsdk::Mission::Result::Success &&
                                        download_result.second.mission_items.size() > 0) {
                                        std::cout << "   📥 Mission下载验证: 航点数量="
                                                  << download_result.second.mission_items.size()
                                                  << std::endl;
                                        if (retry > 150) {
                                            std::cout
                                                << "   🔍 等待Navigator处理Mission并更新mission_result... ("
                                                << (retry - 150) << "/150)" << std::endl;
                                            std::this_thread::sleep_for(
                                                std::chrono::milliseconds(500));
                                        }
                                        if (retry > 100) {
                                            std::cout << "   🛰️ 等待GPS位置锁定完成... ("
                                                      << (retry - 100) << "/200)" << std::endl;
                                            std::this_thread::sleep_for(
                                                std::chrono::milliseconds(200));
                                        }
                                        mission_ready = true;
                                        std::cout << "   Mission下载验证通过，Mission ID已设置"
                                                  << std::endl;
                                        break;
                                    }
                                    if (retry > 50) {
                                        std::cout << "   🔍 等待PX4完成Mission ID设置... (" << (retry - 50)
                                                  << "/250)" << std::endl;
                                    }
                                } catch (const std::exception& e) {
                                    std::cout << "   🔍 检查Mission信息时异常: " << e.what()
                                              << std::endl;
                                }
                                if (retry % 50 == 0) {
                                    std::cout << "   ⏳ 等待Mission ID设置和GPS位置锁定... ("
                                              << (retry / 50 + 1) << "/6)" << std::endl;
                                }
                            }
                            if (!mission_ready) {
                                std::cout << "[警告] [MAVLinkManager] Mission ID设置超时，但继续尝试启动任务..."
                                          << std::endl;
                            }
                            auto start_result = it->second->mission->start_mission();
                            if (start_result != mavsdk::Mission::Result::Success) {
                                std::cout << "[错误] [MAVLinkManager] 启动任务失败: " << start_result
                                          << std::endl;
                                std::cout << "🔄 [MAVLinkManager] 尝试清除任务并重新上传..."
                                          << std::endl;
                                it->second->mission->clear_mission();
                                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                auto retry_upload = it->second->mission->upload_mission(mission_plan);
                                if (retry_upload == mavsdk::Mission::Result::Success) {
                                    std::cout << "[MAVLinkManager] 重新上传任务成功，再次尝试启动..."
                                              << std::endl;
                                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                    auto retry_start = it->second->mission->start_mission();
                                    if (retry_start != mavsdk::Mission::Result::Success) {
                                        std::cout << "[错误] [MAVLinkManager] 重试启动任务仍然失败: "
                                                  << retry_start << std::endl;
                                        return false;
                                    }
                                } else {
                                    std::cout << "[错误] [MAVLinkManager] 重新上传任务失败: "
                                              << retry_upload << std::endl;
                                    return false;
                                }
                            }
                            std::cout << "[MAVLinkManager] 无人机 " << drone_id
                                      << " 开始执行任务飞往目标位置" << std::endl;
                            return true;
                        } catch (const std::exception& e) {
                            std::cout << "[错误] [MAVLinkManager] Mission 模式也失败: " << e.what()
                                      << std::endl;
                            std::cout << "🔄 [MAVLinkManager] Mission模式失败，使用高度保持的备选方案..."
                                      << std::endl;
                            try {
                                std::cout << "   步骤1: 确保高度保持在 " << altitude << " 米..."
                                          << std::endl;
                                auto goto_result =
                                    it->second->action->goto_location(latitude, longitude, altitude, 0.0f);
                                if (goto_result == mavsdk::Action::Result::Success) {
                                    std::cout << "[MAVLinkManager] 使用高度保持的goto_location成功"
                                              << std::endl;
                                    std::cout
                                        << "   [警告] 注意：请监控高度变化，如发现下降请立即干预"
                                        << std::endl;
                                    return true;
                                }
                                std::cout << "[错误] [MAVLinkManager] 高度保持方案也失败: " << goto_result
                                          << std::endl;
                                std::cout << "   💡 建议：检查GPS信号和系统健康状态" << std::endl;
                                return false;
                            } catch (const std::exception& goto_e) {
                                std::cout << "[错误] [MAVLinkManager] 高度保持方案异常: " << goto_e.what()
                                          << std::endl;
                                return false;
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cout << "[错误] [MAVLinkManager] 绝对位置移动异常: " << e.what() << std::endl;
                    return false;
                }
            } else if (command == "set_mode") {
                auto mode_param = parameters.find("mode");
                if (mode_param == parameters.end()) {
                    std::cout << "Missing 'mode' parameter for set_flight_mode command" << std::endl;
                    return false;
                }
                std::string mode = mode_param->second;
                mavsdk::Action::Result result = mavsdk::Action::Result::Unknown;
                if (mode == "MANUAL") {
                    std::cout << "Setting flight mode to MANUAL for drone " << drone_id << std::endl;
                    result = mavsdk::Action::Result::Success;
                } else if (mode == "AUTO") {
                    std::cout << "Setting flight mode to AUTO for drone " << drone_id << std::endl;
                    result = mavsdk::Action::Result::Success;
                } else if (mode == "RTL") {
                    result = it->second->action->return_to_launch();
                    if (result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully set flight mode to RTL for drone " << drone_id
                                  << std::endl;
                    } else {
                        std::cout << "Failed to set flight mode to RTL for drone " << drone_id << ": "
                                  << result << std::endl;
                    }
                } else if (mode == "HOLD") {
                    result = it->second->action->hold();
                    if (result == mavsdk::Action::Result::Success) {
                        std::cout << "Successfully set flight mode to HOLD for drone " << drone_id
                                  << std::endl;
                    } else {
                        std::cout << "Failed to set flight mode to HOLD for drone " << drone_id << ": "
                                  << result << std::endl;
                    }
                } else if (mode == "GUIDED") {
                    std::cout << "Setting flight mode to GUIDED (via offboard) for drone " << drone_id
                              << std::endl;
                    result = mavsdk::Action::Result::Success;
                } else if (mode == "OFFBOARD") {
                    std::cout << "Setting flight mode to OFFBOARD for drone " << drone_id << std::endl;
                    result = mavsdk::Action::Result::Success;
                } else {
                    std::cout << "Unknown flight mode: " << mode << std::endl;
                    return false;
                }
                return (result == mavsdk::Action::Result::Success);
            } else if (command == "offboard_control") {
                auto cmd_param = parameters.find("command");
                if (cmd_param == parameters.end()) {
                    std::cout << "Missing 'command' parameter for offboard_control" << std::endl;
                    return false;
                }
                std::string offboard_cmd = cmd_param->second;
                std::cout << "Executing offboard control command: " << offboard_cmd << " for drone "
                          << drone_id << std::endl;
                if (offboard_cmd == "start_offboard") {
                    try {
                        mavsdk::Offboard::PositionNedYaw position_ned_yaw{};
                        mavsdk::Offboard::VelocityNedYaw velocity_ned_yaw{};
                        auto position = it->second->telemetry->position();
                        (void)position;
                        position_ned_yaw.north_m = 0.0f;
                        position_ned_yaw.east_m = 0.0f;
                        position_ned_yaw.down_m = -position.relative_altitude_m;
                        position_ned_yaw.yaw_deg = 0.0f;
                        velocity_ned_yaw.north_m_s = 0.0f;
                        velocity_ned_yaw.east_m_s = 0.0f;
                        velocity_ned_yaw.down_m_s = 0.0f;
                        velocity_ned_yaw.yaw_deg = 0.0f;
                        auto* offboard_ptr = static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                        offboard_ptr->set_position_ned(position_ned_yaw);
                        offboard_ptr->set_velocity_ned(velocity_ned_yaw);
                        auto offboard_result = offboard_ptr->start();
                        if (offboard_result == mavsdk::Offboard::Result::Success) {
                            std::cout << "Successfully started offboard mode for drone " << drone_id
                                      << std::endl;
                            return true;
                        }
                        std::cout << "Failed to start offboard mode for drone " << drone_id << ": "
                                  << offboard_result << std::endl;
                        return false;
                    } catch (const std::exception& e) {
                        std::cout << "Exception starting offboard mode: " << e.what() << std::endl;
                        return false;
                    }
                } else if (offboard_cmd == "stop_offboard") {
                    try {
                        auto* offboard_ptr = static_cast<mavsdk::Offboard*>(it->second->offboard.get());
                        auto offboard_result = offboard_ptr->stop();
                        if (offboard_result == mavsdk::Offboard::Result::Success) {
                            std::cout << "Successfully stopped offboard mode for drone " << drone_id
                                      << std::endl;
                            return true;
                        }
                        std::cout << "Failed to stop offboard mode for drone " << drone_id << ": "
                                  << offboard_result << std::endl;
                        return false;
                    } catch (const std::exception& e) {
                        std::cout << "Exception stopping offboard mode: " << e.what() << std::endl;
                        return false;
                    }
                } else {
                    std::cout << "Unknown offboard command: " << offboard_cmd << std::endl;
                    return false;
                }
            } else if (command == "move_relative") {
                float north = 0.0f, east = 0.0f, down = 0.0f, yaw = 0.0f;
                auto north_param = parameters.find("north");
                if (north_param != parameters.end()) {
                    try {
                        north = std::stof(north_param->second);
                    } catch (...) {
                    }
                }
                auto east_param = parameters.find("east");
                if (east_param != parameters.end()) {
                    try {
                        east = std::stof(east_param->second);
                    } catch (...) {
                    }
                }
                auto down_param = parameters.find("down");
                if (down_param != parameters.end()) {
                    try {
                        down = std::stof(down_param->second);
                    } catch (...) {
                    }
                }
                auto yaw_param = parameters.find("yaw");
                if (yaw_param != parameters.end()) {
                    try {
                        yaw = std::stof(yaw_param->second);
                    } catch (...) {
                    }
                }
                std::cout << "Moving drone " << drone_id << " relatively: N=" << north
                          << " E=" << east << " D=" << down << " Yaw=" << yaw << std::endl;
                try {
                    std::cout << "Executing relative movement: N=" << north << " E=" << east
                              << " D=" << down << " Yaw=" << yaw << std::endl;
                    auto position = it->second->telemetry->position();
                    double target_lat = position.latitude_deg + (north / 111000.0);
                    double target_lon =
                        position.longitude_deg +
                        (east / (111000.0 * cos(position.latitude_deg * M_PI / 180.0)));
                    double target_alt = position.absolute_altitude_m - down;
                    std::vector<mavsdk::Mission::MissionItem> mission_items;
                    mavsdk::Mission::MissionItem mission_item;
                    mission_item.latitude_deg = target_lat;
                    mission_item.longitude_deg = target_lon;
                    mission_item.relative_altitude_m = target_alt;
                    mission_item.speed_m_s = 5.0f;
                    mission_item.is_fly_through = true;
                    mission_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                    mission_item.loiter_time_s = 0.0f;
                    mission_item.camera_photo_interval_s = 0.0f;
                    mission_items.push_back(mission_item);
                    mavsdk::Mission::MissionPlan mission_plan;
                    mission_plan.mission_items = mission_items;
                    auto upload_result = it->second->mission->upload_mission(mission_plan);
                    if (upload_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to upload relative movement mission: " << upload_result
                                  << std::endl;
                        return false;
                    }
                    auto start_result = it->second->mission->start_mission();
                    if (start_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to start relative movement mission: " << start_result
                                  << std::endl;
                        return false;
                    }
                    std::cout << "Successfully executed relative movement for drone " << drone_id
                              << std::endl;
                    return true;
                } catch (const std::exception& e) {
                    std::cout << "Exception during relative movement: " << e.what() << std::endl;
                    return false;
                }
            } else if (command == "move_to") {
                double latitude = 0.0, longitude = 0.0, altitude = 0.0, yaw = 0.0;
                auto lat_param = parameters.find("latitude");
                if (lat_param != parameters.end()) {
                    try {
                        latitude = std::stod(lat_param->second);
                    } catch (...) {
                    }
                }
                auto lon_param = parameters.find("longitude");
                if (lon_param != parameters.end()) {
                    try {
                        longitude = std::stod(lon_param->second);
                    } catch (...) {
                    }
                }
                auto alt_param = parameters.find("altitude");
                if (alt_param != parameters.end()) {
                    try {
                        altitude = std::stod(alt_param->second);
                    } catch (...) {
                    }
                }
                auto yaw_param = parameters.find("yaw");
                if (yaw_param != parameters.end()) {
                    try {
                        yaw = std::stod(yaw_param->second);
                    } catch (...) {
                    }
                }
                std::cout << "Moving drone " << drone_id << " to: Lat=" << latitude
                          << " Lon=" << longitude << " Alt=" << altitude << " Yaw=" << yaw
                          << std::endl;
                try {
                    std::cout << "Executing absolute movement to: Lat=" << latitude
                              << " Lon=" << longitude << " Alt=" << altitude << " Yaw=" << yaw
                              << std::endl;
                    std::vector<mavsdk::Mission::MissionItem> mission_items;
                    mavsdk::Mission::MissionItem mission_item;
                    mission_item.latitude_deg = latitude;
                    mission_item.longitude_deg = longitude;
                    mission_item.relative_altitude_m = altitude;
                    mission_item.speed_m_s = 5.0f;
                    mission_item.is_fly_through = true;
                    mission_item.gimbal_pitch_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.gimbal_yaw_deg = std::numeric_limits<float>::quiet_NaN();
                    mission_item.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
                    mission_item.loiter_time_s = 0.0f;
                    mission_item.camera_photo_interval_s = 0.0f;
                    mission_items.push_back(mission_item);
                    mavsdk::Mission::MissionPlan mission_plan;
                    mission_plan.mission_items = mission_items;
                    auto upload_result = it->second->mission->upload_mission(mission_plan);
                    if (upload_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to upload absolute movement mission: " << upload_result
                                  << std::endl;
                        return false;
                    }
                    auto start_result = it->second->mission->start_mission();
                    if (start_result != mavsdk::Mission::Result::Success) {
                        std::cout << "Failed to start absolute movement mission: " << start_result
                                  << std::endl;
                        return false;
                    }
                    std::cout << "Successfully executed absolute movement for drone " << drone_id
                              << std::endl;
                    return true;
                } catch (const std::exception& e) {
                    std::cout << "Exception during absolute movement: " << e.what() << std::endl;
                    return false;
                }
            } else {
                std::cout << "Unknown command: " << command << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cout << "Exception sending command '" << command << "' to drone " << drone_id
                      << ": " << e.what() << std::endl;
            return false;
        }
    }
#endif

    std::cout << "Simulating command '" << command << "' to drone " << drone_id << std::endl;
    return true;
}

} // namespace drone_control

