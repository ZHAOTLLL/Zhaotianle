/**
 * 飞行控制模块
 * 封装授权无人机的飞行控制与任务执行（起飞、航点、任务状态），供 API 与访问控制调用。
 */
#pragma once

#include "common/types.hpp"
#include "communication/mavlink_manager.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace drone_control {

class FlightControlModule {
public:
    FlightControlModule();
    ~FlightControlModule();

    bool initialize(std::shared_ptr<MAVLinkManager> mavlink_manager);
    void shutdown();
    bool isInitialized() const;

    // 任务接口（当前仅供 HTTP API 使用）
    bool executeMission(DroneId drone_id, const std::string& mission_type, 
                       const std::map<std::string, std::string>& parameters);
    std::string getMissionStatus(DroneId drone_id) const;
    std::optional<Position> getCurrentPosition(DroneId drone_id) const;
    
private:
    struct MissionSession {
        DroneId drone_id;
        std::string mission_type;
        std::map<std::string, std::string> parameters;
        std::string status;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point last_update;
        std::string current_step;
        double progress_percentage;
        
        MissionSession(DroneId id)
            : drone_id(id)
            , status("idle")
            , progress_percentage(0.0)
            , start_time(std::chrono::steady_clock::now())
            , last_update(start_time) {}
    };

    void executeMissionAsync(DroneId drone_id, std::string mission_type,
                             std::map<std::string, std::string> parameters);
    bool runDeliveryMission(DroneId drone_id, const std::map<std::string, std::string>& parameters);
    bool waitForAltitude(DroneId drone_id, double target_altitude, std::chrono::seconds timeout);
    bool waitForPosition(DroneId drone_id,
                         double target_lat, double target_lon, double target_altitude,
                         std::chrono::seconds timeout);
    void updateSession(DroneId drone_id, const std::function<void(MissionSession&)>& updater);

    std::shared_ptr<MAVLinkManager> mavlink_manager_;
    mutable std::mutex mission_mutex_;
    std::map<DroneId, MissionSession> mission_sessions_;
    std::atomic<bool> initialized_;
};

} // namespace drone_control