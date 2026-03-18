#pragma once

/**
 * 无人机相关 API
 * 处理无人机列表、状态列表、单机详情的 HTTP 请求，数据来自 MAVLink 与状态管理器。
 */
#include <string>
#include <memory>
#include "response_utils.hpp"

namespace drone_control {
    class MAVLinkManager;
    class DroneStateManager;
}

namespace drone_control {
namespace api {

/** 处理 /api/v1/drones、/api/v1/drones/states、/api/v1/drones/{id} */
class DroneAPI {
public:
    DroneAPI(std::shared_ptr<MAVLinkManager> mavlink_mgr,
             std::shared_ptr<DroneStateManager> state_mgr);

    std::string handleDrones(const std::string& path, const std::string& method);
    std::string handleDroneStates(const std::string& path, const std::string& method);
    std::string handleDroneDetail(const std::string& path, const std::string& method);

private:
    std::shared_ptr<MAVLinkManager> mavlink_mgr_;
    std::shared_ptr<DroneStateManager> state_mgr_;

    /** 从路径中解析无人机 ID，例如 /api/v1/drones/2 -> 2 */
    int parseDroneIdFromPath(const std::string& path);
};

} // namespace api
} // namespace drone_control


