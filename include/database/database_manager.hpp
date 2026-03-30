/**
 * 数据库管理器
 * 负责管理PostgreSQL和Redis连接，提供数据存储和缓存功能
 */
#pragma once

#include "common/types.hpp"
#include "common/drone_attributes.hpp"
#include <memory>
#include <string>
#include <map>
#include <optional>

namespace drone_control {

// 前向声明
class PostgreSQLDatabase;
class RedisDatabase;

/** 数据库管理器 */
class DatabaseManager {
public:
    DatabaseManager();
    ~DatabaseManager();

    // 初始化
    bool initialize(const std::string& postgres_host, int postgres_port, const std::string& postgres_db, 
                   const std::string& postgres_user, const std::string& postgres_password,
                   const std::string& redis_host, int redis_port, const std::string& redis_password);

    // 无人机数据管理
    bool saveDroneState(DroneId drone_id, const ExtendedDroneState& state);
    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id);
    bool saveDroneAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes);
    std::optional<std::map<std::string, std::string>> getDroneAttributes(DroneId drone_id);

    // 边缘设备数据管理
    bool saveEdgeDeviceInfo(const std::string& device_id, const std::string& device_name, const std::string& location, bool online);
    std::optional<std::map<std::string, std::string>> getEdgeDeviceInfo(const std::string& device_id);

    // 访问控制数据管理
    bool saveAccessDecision(DroneId drone_id, bool granted, const std::string& reason, const std::string& timestamp);
    std::vector<std::map<std::string, std::string>> getAccessDecisions(DroneId drone_id, int limit = 10);

    // 缓存管理
    bool setCache(const std::string& key, const std::string& value, int expiration = 3600);
    std::optional<std::string> getCache(const std::string& key);
    bool deleteCache(const std::string& key);

    // 健康检查
    bool isPostgresConnected() const;
    bool isRedisConnected() const;

private:
    std::unique_ptr<PostgreSQLDatabase> postgres_db_;
    std::unique_ptr<RedisDatabase> redis_db_;
};

} // namespace drone_control