/**
 * 数据库管理器实现
 */
#include "database/database_manager.hpp"
// 直接包含实现，因为我们把实现放在了源文件中
// 注意：这种方式不是最佳实践，但是为了简化编译，暂时这样处理
#include <iostream>

namespace drone_control {

DatabaseManager::DatabaseManager() {
}

DatabaseManager::~DatabaseManager() {
}

bool DatabaseManager::initialize(const std::string& postgres_host, int postgres_port, const std::string& postgres_db, 
                               const std::string& postgres_user, const std::string& postgres_password,
                               const std::string& redis_host, int redis_port, const std::string& redis_password) {
    // 初始化PostgreSQL
    postgres_db_ = std::make_unique<PostgreSQLDatabase>();
    if (!postgres_db_->connect(postgres_host, postgres_port, postgres_db, postgres_user, postgres_password)) {
        std::cerr << "[数据库] PostgreSQL初始化失败" << std::endl;
        return false;
    }

    // 初始化Redis
    redis_db_ = std::make_unique<RedisDatabase>();
    if (!redis_db_->connect(redis_host, redis_port, redis_password)) {
        std::cerr << "[数据库] Redis初始化失败" << std::endl;
        return false;
    }

    std::cout << "[数据库] 初始化成功" << std::endl;
    return true;
}

bool DatabaseManager::saveDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    // 先保存到PostgreSQL
    if (!postgres_db_->saveDroneState(drone_id, state)) {
        return false;
    }

    // 再缓存到Redis
    redis_db_->cacheDroneState(drone_id, state);
    return true;
}

std::optional<ExtendedDroneState> DatabaseManager::getDroneState(DroneId drone_id) {
    // 先尝试从Redis缓存获取
    auto cached_state = redis_db_->getCachedDroneState(drone_id);
    if (cached_state) {
        return cached_state;
    }

    // 从PostgreSQL获取
    return postgres_db_->getDroneState(drone_id);
}

bool DatabaseManager::saveDroneAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) {
    // 这里可以实现保存无人机属性到数据库的逻辑
    // 简化实现，实际应该将属性存储到PostgreSQL
    return true;
}

std::optional<std::map<std::string, std::string>> DatabaseManager::getDroneAttributes(DroneId drone_id) {
    // 这里可以实现从数据库获取无人机属性的逻辑
    return std::nullopt;
}

bool DatabaseManager::saveEdgeDeviceInfo(const std::string& device_id, const std::string& device_name, 
                                       const std::string& location, bool online) {
    return postgres_db_->saveEdgeDeviceInfo(device_id, device_name, location, online);
}

std::optional<std::map<std::string, std::string>> DatabaseManager::getEdgeDeviceInfo(const std::string& device_id) {
    return postgres_db_->getEdgeDeviceInfo(device_id);
}

bool DatabaseManager::saveAccessDecision(DroneId drone_id, bool granted, const std::string& reason, const std::string& timestamp) {
    // 这里可以实现保存访问决策到数据库的逻辑
    return true;
}

std::vector<std::map<std::string, std::string>> DatabaseManager::getAccessDecisions(DroneId drone_id, int limit) {
    // 这里可以实现从数据库获取访问决策的逻辑
    return {};
}

bool DatabaseManager::setCache(const std::string& key, const std::string& value, int expiration) {
    return redis_db_->setCache(key, value, expiration);
}

std::optional<std::string> DatabaseManager::getCache(const std::string& key) {
    return redis_db_->getCache(key);
}

bool DatabaseManager::deleteCache(const std::string& key) {
    return redis_db_->deleteCache(key);
}

bool DatabaseManager::isPostgresConnected() const {
    return postgres_db_ && postgres_db_->isConnected();
}

bool DatabaseManager::isRedisConnected() const {
    return redis_db_ && redis_db_->isConnected();
}

} // namespace drone_control