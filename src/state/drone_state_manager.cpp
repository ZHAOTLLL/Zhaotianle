/**
 * 无人机状态管理实现
 * 维护各机遥测、在线状态与属性，支持超时检测与回调通知。
 */
#include "state/drone_state_manager.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

namespace drone_control {

DroneStateManager::DroneStateManager() 
    : timeout_duration_(DEFAULT_TIMEOUT)
    , last_stats_update_(std::chrono::steady_clock::now()) {
    std::cout << "DroneStateManager initialized" << std::endl;
}

DroneStateManager::~DroneStateManager() {
    clearAllStates();
}

void DroneStateManager::updateDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it == drone_states_.end()) {
        // 创建新的状态信息
        auto state_info = std::make_unique<DroneStateInfo>();
        state_info->state = state;
        state_info->state.drone_id = drone_id;  // 确保ID正确
        state_info->is_online = true;
        state_info->last_update = std::chrono::steady_clock::now();
        
        drone_states_[drone_id] = std::move(state_info);
        
        std::cout << "Added new drone state for drone " << drone_id << std::endl;
    } else {
        // 更新现有状态
        it->second->state = state;
        it->second->state.drone_id = drone_id;  // 确保ID正确
        it->second->last_update = std::chrono::steady_clock::now();
        
        if (!it->second->is_online) {
            it->second->is_online = true;
            std::cout << "Drone " << drone_id << " came back online" << std::endl;
        }
    }
    
    // 通知状态变化
    notifyStateChange(state);
}

std::optional<ExtendedDroneState> DroneStateManager::getDroneState(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it == drone_states_.end()) {
        return std::nullopt;
    }
    
    return it->second->state;
}

void DroneStateManager::updateDronePosition(DroneId drone_id, const Position& position) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end()) {
        it->second->state.position = position;
        it->second->last_update = std::chrono::steady_clock::now();
        
        // 通知状态变化
        notifyStateChange(it->second->state);
    }
}

void DroneStateManager::updateDroneFlightStatus(DroneId drone_id, FlightStatus status) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end()) {
        FlightStatus old_status = it->second->state.flight_status;
        it->second->state.flight_status = status;
        it->second->last_update = std::chrono::steady_clock::now();
        
        if (old_status != status) {
            std::cout << "Drone " << drone_id << " flight status changed from " 
                      << static_cast<int>(old_status) << " to " << static_cast<int>(status) << std::endl;
        }
        
        // 通知状态变化
        notifyStateChange(it->second->state);
    }
}

void DroneStateManager::updateDroneBattery(DroneId drone_id, double battery_percentage) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end()) {
        it->second->state.battery_percentage = battery_percentage;
        it->second->last_update = std::chrono::steady_clock::now();
        
        // 检查低电量警告
        if (battery_percentage < 20.0) {
            std::cout << "Warning: Drone " << drone_id << " battery low: " 
                      << battery_percentage << "%" << std::endl;
        }
        
        // 通知状态变化
        notifyStateChange(it->second->state);
    }
}

void DroneStateManager::updateDroneAttributes(DroneId drone_id, const DroneAttributes& attributes) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end()) {
        // 更新相关属性到状态中
        it->second->state.owner_organization = attributes.organization;
        it->second->last_update = std::chrono::steady_clock::now();
        
        // 通知状态变化
        notifyStateChange(it->second->state);
    }
}

std::vector<DroneId> DroneStateManager::getActiveDrones() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<DroneId> active_drones;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && !isStateExpired(*state_info)) {
            active_drones.push_back(drone_id);
        }
    }
    
    return active_drones;
}

std::vector<DroneId> DroneStateManager::getDronesByStatus(FlightStatus status) const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<DroneId> matching_drones;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && state_info->state.flight_status == status) {
            matching_drones.push_back(drone_id);
        }
    }
    
    return matching_drones;
}

std::vector<DroneId> DroneStateManager::getDronesByOrganization(const std::string& organization) const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<DroneId> matching_drones;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && state_info->state.owner_organization == organization) {
            matching_drones.push_back(drone_id);
        }
    }
    
    return matching_drones;
}

std::vector<ExtendedDroneState> DroneStateManager::getAllDroneStates() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<ExtendedDroneState> all_states;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online) {
            all_states.push_back(state_info->state);
        }
    }
    
    return all_states;
}

void DroneStateManager::setDroneOnline(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end()) {
        if (!it->second->is_online) {
            it->second->is_online = true;
            it->second->last_update = std::chrono::steady_clock::now();
            std::cout << "Drone " << drone_id << " set online" << std::endl;
        }
    } else {
        // 创建新的在线状态
        auto state_info = std::make_unique<DroneStateInfo>();
        state_info->state.drone_id = drone_id;
        state_info->is_online = true;
        state_info->last_update = std::chrono::steady_clock::now();
        
        drone_states_[drone_id] = std::move(state_info);
        std::cout << "Created new online drone state for drone " << drone_id << std::endl;
    }
}

void DroneStateManager::setDroneOffline(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it != drone_states_.end() && it->second->is_online) {
        it->second->is_online = false;
        std::cout << "Drone " << drone_id << " set offline" << std::endl;
        
        // 通知无人机离线
        notifyDroneOffline(drone_id);
    }
}

bool DroneStateManager::isDroneOnline(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.find(drone_id);
    if (it == drone_states_.end()) {
        return false;
    }
    
    return it->second->is_online && !isStateExpired(*it->second);
}

std::vector<DroneId> DroneStateManager::getOnlineDrones() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<DroneId> online_drones;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && !isStateExpired(*state_info)) {
            online_drones.push_back(drone_id);
        }
    }
    
    return online_drones;
}

std::vector<DroneId> DroneStateManager::getOfflineDrones() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::vector<DroneId> offline_drones;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (!state_info->is_online || isStateExpired(*state_info)) {
            offline_drones.push_back(drone_id);
        }
    }
    
    return offline_drones;
}

void DroneStateManager::checkTimeouts() {
    std::vector<DroneId> timed_out_drones;
    
    {
        std::lock_guard<std::mutex> lock(states_mutex_);
        for (auto& [drone_id, state_info] : drone_states_) {
            if (state_info->is_online && isStateExpired(*state_info)) {
                state_info->is_online = false;
                timed_out_drones.push_back(drone_id);
            }
        }
    }
    
    // 通知超时的无人机
    for (DroneId drone_id : timed_out_drones) {
        std::cout << "Drone " << drone_id << " timed out" << std::endl;
        notifyDroneOffline(drone_id);
    }
}

void DroneStateManager::setTimeoutDuration(std::chrono::seconds timeout) {
    timeout_duration_ = timeout;
    std::cout << "Timeout duration set to " << timeout.count() << " seconds" << std::endl;
}

void DroneStateManager::registerStateChangeCallback(StateChangeCallback callback) {
    state_change_callbacks_.push_back(callback);
}

void DroneStateManager::registerDroneOfflineCallback(DroneOfflineCallback callback) {
    drone_offline_callbacks_.push_back(callback);
}

size_t DroneStateManager::getTotalDroneCount() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    return drone_states_.size();
}

size_t DroneStateManager::getOnlineDroneCount() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    size_t count = 0;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && !isStateExpired(*state_info)) {
            count++;
        }
    }
    
    return count;
}

std::map<FlightStatus, size_t> DroneStateManager::getStatusStatistics() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::map<FlightStatus, size_t> stats;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && !isStateExpired(*state_info)) {
            stats[state_info->state.flight_status]++;
        }
    }
    
    return stats;
}

std::map<std::string, size_t> DroneStateManager::getOrganizationStatistics() const {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    std::map<std::string, size_t> stats;
    for (const auto& [drone_id, state_info] : drone_states_) {
        if (state_info->is_online && !isStateExpired(*state_info)) {
            const std::string& org = state_info->state.owner_organization;
            if (!org.empty()) {
                stats[org]++;
            }
        }
    }
    
    return stats;
}

void DroneStateManager::removeOfflineDrones() {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    auto it = drone_states_.begin();
    while (it != drone_states_.end()) {
        if (!it->second->is_online || isStateExpired(*it->second)) {
            std::cout << "Removing offline drone " << it->first << std::endl;
            it = drone_states_.erase(it);
        } else {
            ++it;
        }
    }
}

void DroneStateManager::clearAllStates() {
    std::lock_guard<std::mutex> lock(states_mutex_);
    
    size_t count = drone_states_.size();
    drone_states_.clear();
    
    if (count > 0) {
        std::cout << "Cleared " << count << " drone states" << std::endl;
    }
}

void DroneStateManager::notifyStateChange(const ExtendedDroneState& state) {
    for (auto& callback : state_change_callbacks_) {
        try {
            callback(state);
        } catch (const std::exception& e) {
            std::cout << "Exception in state change callback: " << e.what() << std::endl;
        }
    }
}

void DroneStateManager::notifyDroneOffline(DroneId drone_id) {
    for (auto& callback : drone_offline_callbacks_) {
        try {
            callback(drone_id);
        } catch (const std::exception& e) {
            std::cout << "Exception in drone offline callback: " << e.what() << std::endl;
        }
    }
}

bool DroneStateManager::isStateExpired(const DroneStateInfo& info) const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - info.last_update;
    return elapsed > timeout_duration_;
}

} // namespace drone_control