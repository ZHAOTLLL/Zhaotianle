/**
 * 飞行控制模块
 * 封装授权无人机的飞行控制与任务执行，与 MAVLink 交互完成起飞、航点与任务状态更新。
 */
#include "flight_control/flight_control_module.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

namespace drone_control {
namespace {
constexpr double kEarthRadiusMeters = 6'371'000.0;

double parseDouble(const std::map<std::string, std::string>& params,
                   const std::string& key,
                   double fallback) {
    auto it = params.find(key);
    if (it == params.end()) {
        return fallback;
    }
    try {
        return std::stod(it->second);
    } catch (...) {
        return fallback;
    }
}
} // namespace

FlightControlModule::FlightControlModule()
    : initialized_(false) {}

FlightControlModule::~FlightControlModule() {
    shutdown();
}

bool FlightControlModule::initialize(std::shared_ptr<MAVLinkManager> mavlink_manager) {
    if (initialized_) {
        return true;
    }
    if (!mavlink_manager) {
        std::cout << "[FlightControl] 初始化失败：MAVLinkManager 不可用" << std::endl;
        return false;
    }

    mavlink_manager_ = std::move(mavlink_manager);
    initialized_ = true;
    std::cout << "[FlightControl] 初始化完成" << std::endl;
    return true;
}

void FlightControlModule::shutdown() {
    if (!initialized_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_sessions_.clear();
    }

    initialized_ = false;
    std::cout << "[FlightControl] 已关闭" << std::endl;
}

bool FlightControlModule::isInitialized() const {
    return initialized_.load();
}

std::optional<Position> FlightControlModule::getCurrentPosition(DroneId drone_id) const {
    if (!mavlink_manager_) {
        return std::nullopt;
    }

    auto drone_state = mavlink_manager_->getDroneState(drone_id);
    if (!drone_state) {
        return std::nullopt;
    }

    Position pos;
    pos.latitude = drone_state->position.latitude;
    pos.longitude = drone_state->position.longitude;
    pos.altitude = drone_state->position.altitude;
    return pos;
}

bool FlightControlModule::executeMission(DroneId drone_id,
                                         const std::string& mission_type,
                                         const std::map<std::string, std::string>& parameters) {
    if (!initialized_ || !mavlink_manager_) {
        std::cout << "[FlightControl] 无法启动任务：模块未初始化" << std::endl;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        auto it = mission_sessions_.find(drone_id);
        if (it != mission_sessions_.end() && it->second.status == "running") {
            std::cout << "[FlightControl] 无人机 " << drone_id << " 已有任务在运行" << std::endl;
            return false;
        }

        MissionSession session(drone_id);
        session.mission_type = mission_type;
        session.parameters = parameters;
        session.status = "running";
        session.current_step = "starting";
        mission_sessions_.insert_or_assign(drone_id, session);
    }

    std::thread(&FlightControlModule::executeMissionAsync,
                this,
                drone_id,
                mission_type,
                parameters).detach();

    std::cout << "[FlightControl] 无人机 " << drone_id << " 开始任务: " << mission_type << std::endl;
    return true;
}

std::string FlightControlModule::getMissionStatus(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    auto it = mission_sessions_.find(drone_id);
    if (it == mission_sessions_.end()) {
        return R"({"status": "idle", "mission_type": "", "progress": 0.0})";
    }

    const auto& session = it->second;
    std::ostringstream out;
    out << "{";
    out << "\"status\": \"" << session.status << "\",";
    out << "\"mission_type\": \"" << session.mission_type << "\",";
    out << "\"current_step\": \"" << session.current_step << "\",";
    out << "\"progress\": " << session.progress_percentage << ",";
    out << "\"start_time\": "
        << std::chrono::duration_cast<std::chrono::seconds>(session.start_time.time_since_epoch()).count()
        << ",";
    out << "\"last_update\": "
        << std::chrono::duration_cast<std::chrono::seconds>(session.last_update.time_since_epoch()).count();
    out << "}";
    return out.str();
}

void FlightControlModule::executeMissionAsync(DroneId drone_id,
                                              std::string mission_type,
                                              std::map<std::string, std::string> parameters) {
    bool success = false;
    if (mission_type == "delivery" || mission_type == "patrol" ||
        mission_type == "inspection" || mission_type == "custom") {
        success = runDeliveryMission(drone_id, parameters);
    } else {
        std::cout << "[FlightControl] 未知任务类型: " << mission_type << std::endl;
    }

    updateSession(drone_id, [success](MissionSession& session) {
        session.status = success ? "completed" : "failed";
        session.current_step = session.status;
        session.progress_percentage = success ? 100.0 : session.progress_percentage;
    });

    std::cout << "[FlightControl] 无人机 " << drone_id
              << " 任务结束: " << (success ? "成功" : "失败") << std::endl;
}

bool FlightControlModule::runDeliveryMission(
    DroneId drone_id,
    const std::map<std::string, std::string>& parameters) {

    auto home_position = getCurrentPosition(drone_id);
    double start_lat = parseDouble(parameters, "start_lat",
                                   home_position ? home_position->latitude : 0.0);
    double start_lon = parseDouble(parameters, "start_lon",
                                   home_position ? home_position->longitude : 0.0);
    double end_lat = parseDouble(parameters, "end_lat", start_lat);
    double end_lon = parseDouble(parameters, "end_lon", start_lon);
    double cruise_alt = parseDouble(parameters, "altitude",
                                    home_position ? home_position->altitude + 20.0 : 30.0);

    auto sendCommand = [this, drone_id](const std::string& command,
                                        const std::map<std::string, std::string>& params) {
        if (!mavlink_manager_->sendCommand(drone_id, command, params)) {
            std::cout << "[FlightControl] command '" << command
                      << "' 发送失败 (drone " << drone_id << ")" << std::endl;
            return false;
        }
        return true;
    };

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "arming";
        session.progress_percentage = 5.0;
    });
    if (!sendCommand("arm", {})) {
        return false;
    }

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "takeoff";
        session.progress_percentage = 15.0;
    });
    if (!sendCommand("takeoff", {{"altitude", std::to_string(cruise_alt)}})) {
        return false;
    }
    if (!waitForAltitude(drone_id, cruise_alt, std::chrono::seconds(30))) {
        std::cout << "[FlightControl] 起飞阶段未达到目标高度" << std::endl;
        return false;
    }

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "enroute_target";
        session.progress_percentage = 40.0;
    });
    if (!sendCommand("move_absolute",
                     {{"latitude", std::to_string(end_lat)},
                      {"longitude", std::to_string(end_lon)},
                      {"altitude", std::to_string(cruise_alt)}})) {
        return false;
    }
    if (!waitForPosition(drone_id, end_lat, end_lon, cruise_alt, std::chrono::seconds(180))) {
        return false;
    }

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "holding";
        session.progress_percentage = 60.0;
    });
    std::this_thread::sleep_for(std::chrono::seconds(5));

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "returning";
        session.progress_percentage = 75.0;
    });
    if (!sendCommand("move_absolute",
                     {{"latitude", std::to_string(start_lat)},
                      {"longitude", std::to_string(start_lon)},
                      {"altitude", std::to_string(cruise_alt)}})) {
        return false;
    }
    if (!waitForPosition(drone_id, start_lat, start_lon, cruise_alt, std::chrono::seconds(180))) {
        return false;
    }

    updateSession(drone_id, [](MissionSession& session) {
        session.current_step = "landing";
        session.progress_percentage = 90.0;
    });
    if (!sendCommand("land", {})) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    sendCommand("disarm", {});

    return true;
}

bool FlightControlModule::waitForAltitude(DroneId drone_id,
                                          double target_altitude,
                                          std::chrono::seconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto position = getCurrentPosition(drone_id);
        if (position) {
            if (std::abs(position->altitude - target_altitude) <= 2.0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

bool FlightControlModule::waitForPosition(DroneId drone_id,
                                          double target_lat,
                                          double target_lon,
                                          double target_alt,
                                          std::chrono::seconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto position = getCurrentPosition(drone_id);
        if (position) {
            double lat1 = target_lat * M_PI / 180.0;
            double lat2 = position->latitude * M_PI / 180.0;
            double delta_lat = (position->latitude - target_lat) * M_PI / 180.0;
            double delta_lon = (position->longitude - target_lon) * M_PI / 180.0;
            double a = std::sin(delta_lat / 2) * std::sin(delta_lat / 2) +
                       std::cos(lat1) * std::cos(lat2) *
                       std::sin(delta_lon / 2) * std::sin(delta_lon / 2);
            double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
            double distance = kEarthRadiusMeters * c;
            double alt_diff = std::abs(position->altitude - target_alt);

            if (distance < 20.0 && alt_diff < 5.0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

void FlightControlModule::updateSession(
    DroneId drone_id,
    const std::function<void(MissionSession&)>& updater) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    auto it = mission_sessions_.find(drone_id);
    if (it != mission_sessions_.end()) {
        updater(it->second);
        it->second.last_update = std::chrono::steady_clock::now();
    }
}

} // namespace drone_control
