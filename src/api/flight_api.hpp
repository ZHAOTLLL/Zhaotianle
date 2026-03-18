#pragma once

/**
 * 飞行控制 API
 * 处理自动飞行请求、飞行命令（arm/disarm/land/takeoff 等）、带参命令（set_mode/set_altitude/move_to）、飞行状态查询。
 */
#include <string>
#include <memory>
#include <map>
#include <atomic>
#include "response_utils.hpp"
#include "flight_control/flight_control_module.hpp"

namespace drone_control {
    class MAVLinkManager;
}

namespace drone_control {
namespace api {

/** 处理 /api/v1/flight/* 与 /api/v1/drones/{id}/command/* */
class FlightAPI {
public:
    FlightAPI(std::shared_ptr<MAVLinkManager> mavlink_mgr,
              std::shared_ptr<FlightControlModule> flight_control);

    std::string handleAutoFlightRequest(const std::string& path, const std::string& method);
    std::string handleFlightCommand(const std::string& path, const std::string& method);
    std::string handleTakeoff(const std::string& path, const std::string& method);
    std::string handleFlightCommandWithParams(const std::string& path, const std::string& method, const std::string& body);
    std::string handleFlightStatus(const std::string& path, const std::string& method);

private:
    std::shared_ptr<MAVLinkManager> mavlink_mgr_;
    std::shared_ptr<FlightControlModule> flight_control_;

    bool parseCommandFromPath(const std::string& path, int& drone_id, std::string& command);
    std::map<std::string, std::string> parseJsonParameters(const std::string& body);
    bool isDroneConnected(int drone_id);
    bool sendCommandViaFlightControl(int drone_id, const std::string& command,
                                   const std::map<std::string, std::string>& parameters);
    bool executePatrolMission();
    void waitForPositionReached(int drone_id, double target_lat, double target_lon, double target_alt, double timeout_seconds);
    std::string getMissionStatus();
};

} // namespace api
} // namespace drone_control


