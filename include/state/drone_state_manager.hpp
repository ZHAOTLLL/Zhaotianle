#pragma once

/**
 * 无人机状态管理
 * 维护各无人机的遥测、在线状态与属性，供 API 查询与访问控制使用。
 */
#include "common/types.hpp"
#include "common/drone_attributes.hpp"
#include "database/database_manager.hpp"
#include "resources/airspace_resource_manager.hpp"
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include <chrono>
#include <functional>
#include <optional>

namespace drone_control {

class DroneStateManager {
public:
    // 状态变化回调类型
    using StateChangeCallback = std::function<void(const ExtendedDroneState&)>;
    using DroneOfflineCallback = std::function<void(DroneId)>;

    DroneStateManager();
    ~DroneStateManager();

    // 状态管理
    void updateDroneState(DroneId drone_id, const ExtendedDroneState& state);
    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id) const;
    
    // 批量状态操作
    void updateDronePosition(DroneId drone_id, const Position& position);
    void updateDroneFlightStatus(DroneId drone_id, FlightStatus status);
    void updateDroneBattery(DroneId drone_id, double battery_percentage);
    void updateDroneAttributes(DroneId drone_id, const DroneAttributes& attributes);
    
    // 查询接口
    std::vector<DroneId> getActiveDrones() const;
    std::vector<DroneId> getDronesByStatus(FlightStatus status) const;
    std::vector<DroneId> getDronesByOrganization(const std::string& organization) const;
    std::vector<ExtendedDroneState> getAllDroneStates() const;
    
    // 在线状态管理
    void setDroneOnline(DroneId drone_id);
    void setDroneOffline(DroneId drone_id);
    bool isDroneOnline(DroneId drone_id) const;
    std::vector<DroneId> getOnlineDrones() const;
    std::vector<DroneId> getOfflineDrones() const;
    
    // 超时检查
    void checkTimeouts();
    void setTimeoutDuration(std::chrono::seconds timeout);
    
    // 回调注册
    void registerStateChangeCallback(StateChangeCallback callback);
    void registerDroneOfflineCallback(DroneOfflineCallback callback);
    
    // 统计信息
    size_t getTotalDroneCount() const;
    size_t getOnlineDroneCount() const;
    std::map<FlightStatus, size_t> getStatusStatistics() const;
    std::map<std::string, size_t> getOrganizationStatistics() const;
    
    // 清理操作
    void removeOfflineDrones();
    void clearAllStates();

public:
    // 依赖注入
    void setAirspaceResourceManager(std::shared_ptr<resources::AirspaceResourceManager> airspace_manager) {
        airspace_manager_ = airspace_manager;
    }
    
    void setDatabaseManager(std::shared_ptr<DatabaseManager> database_manager) {
        database_manager_ = database_manager;
    }

private:
    struct DroneStateInfo {
        ExtendedDroneState state;
        bool is_online;
        std::chrono::steady_clock::time_point last_update;
        
        DroneStateInfo() : is_online(false), last_update(std::chrono::steady_clock::now()) {}
    };

    // 内部方法
    void notifyStateChange(const ExtendedDroneState& state);
    void notifyDroneOffline(DroneId drone_id);
    bool isStateExpired(const DroneStateInfo& info) const;
    
    // 成员变量
    mutable std::mutex states_mutex_;
    std::map<DroneId, std::unique_ptr<DroneStateInfo>> drone_states_;
    
    std::vector<StateChangeCallback> state_change_callbacks_;
    std::vector<DroneOfflineCallback> drone_offline_callbacks_;
    
    std::chrono::seconds timeout_duration_;
    
    // 依赖
    std::shared_ptr<resources::AirspaceResourceManager> airspace_manager_;
    std::shared_ptr<DatabaseManager> database_manager_;
    
    // 统计缓存（可选优化）
    mutable std::mutex stats_mutex_;
    mutable std::chrono::steady_clock::time_point last_stats_update_;
    mutable std::map<FlightStatus, size_t> cached_status_stats_;
    mutable std::map<std::string, size_t> cached_org_stats_;
    
    static constexpr std::chrono::seconds DEFAULT_TIMEOUT{60};
    static constexpr std::chrono::seconds STATS_CACHE_DURATION{5};
};

} // namespace drone_control