#pragma once

/**
 * 后端统一初始化与关闭
 * 供 C++ gRPC 服务端使用：加载配置、MAVLink 直连无人机、访问控制、飞行控制及各 API 模块。
 */
#include <memory>
#include <string>

namespace drone_control {
class MAVLinkManager;
class DroneStateManager;
class FlightControlModule;
class AccessControlEngine;
namespace api {
class DroneAPI;
class FlightAPI;
class AccessAPI;
}  // namespace api
}  // namespace drone_control

namespace backend {

/** 执行一次初始化；成功返回 true，失败返回 false */
bool init(const std::string& config_file);

/** 关闭并释放所有后端资源 */
void shutdown();

std::shared_ptr<drone_control::MAVLinkManager> getMavLinkManager();
std::shared_ptr<drone_control::DroneStateManager> getStateManager();
std::shared_ptr<drone_control::FlightControlModule> getFlightControl();
drone_control::AccessControlEngine* getAccessControlEngine();
drone_control::api::DroneAPI* getDroneAPI();
drone_control::api::FlightAPI* getFlightAPI();
drone_control::api::AccessAPI* getAccessAPI();

/** 手动确认任务并请求证书（发 USER_1 + USER_2），触发访问控制流程。返回是否成功发送。 */
bool confirmMission(uint32_t drone_id);

}  // namespace backend
