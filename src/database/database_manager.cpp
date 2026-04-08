/**
 * 数据库管理器实现
 */
#include "database/database_manager.hpp"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <regex>
#include <libpq-fe.h>
#include "hiredis.h"
#include "resources/airspace_resource_manager.hpp"

namespace drone_control {

// PostgreSQLDatabase 实现
class PostgreSQLDatabase {
public:
    PostgreSQLDatabase() : conn_(nullptr) {}

    ~PostgreSQLDatabase() {
        if (conn_) {
            PQfinish(conn_);
            conn_ = nullptr;
        }
    }

    bool connect(const std::string& host, int port, const std::string& dbname, 
                const std::string& user, const std::string& password) {
        std::cout << "[PostgreSQL] 尝试连接: " << host << ":" << port << "/" << dbname << std::endl;
        std::ostringstream conninfo;
        conninfo << "host=" << host
                 << " port=" << port
                 << " dbname=" << dbname
                 << " user=" << user
                 << " password=" << password;
        conn_ = PQconnectdb(conninfo.str().c_str());
        if (!conn_ || PQstatus(conn_) != CONNECTION_OK) {
            std::cerr << "[PostgreSQL] 连接失败: "
                      << (conn_ ? PQerrorMessage(conn_) : "unknown error") << std::endl;
            if (conn_) {
                PQfinish(conn_);
                conn_ = nullptr;
            }
            return false;
        }
        std::cout << "[PostgreSQL] 连接成功" << std::endl;
        return createTables();
    }

    bool isConnected() const {
        return conn_ != nullptr;
    }

    bool saveDroneState(DroneId drone_id, const ExtendedDroneState& state) {
        if (!isConnected()) {
            return false;
        }
        const char* values[8];
        std::string drone_id_s = std::to_string(drone_id);
        std::string latitude_s = std::to_string(state.position.latitude);
        std::string longitude_s = std::to_string(state.position.longitude);
        std::string altitude_s = std::to_string(state.position.altitude);
        std::string battery_s = std::to_string(state.battery_percentage);
        std::string armed_s = state.is_armed ? "true" : "false";
        std::string flight_status_s = std::to_string(static_cast<int>(state.flight_status));
        std::string owner_org = state.owner_organization;

        values[0] = drone_id_s.c_str();
        values[1] = latitude_s.c_str();
        values[2] = longitude_s.c_str();
        values[3] = altitude_s.c_str();
        values[4] = battery_s.c_str();
        values[5] = armed_s.c_str();
        values[6] = flight_status_s.c_str();
        values[7] = owner_org.c_str();

        PGresult* res = PQexecParams(
            conn_,
            "INSERT INTO drone_state (drone_id, latitude, longitude, altitude, battery_percentage, is_armed, flight_status, owner_organization, updated_at) "
            "VALUES ($1::int, $2::double precision, $3::double precision, $4::double precision, $5::double precision, $6::boolean, $7::int, $8::text, NOW())",
            8, nullptr, values, nullptr, nullptr, 0);

        const bool ok = res && PQresultStatus(res) == PGRES_COMMAND_OK;
        if (!ok) {
            std::cerr << "[PostgreSQL] saveDroneState 失败: " << (res ? PQresultErrorMessage(res) : "null result") << std::endl;
        }
        if (res) PQclear(res);
        return ok;
    }

    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id) {
        if (!isConnected()) {
            return std::nullopt;
        }
        std::string drone_id_s = std::to_string(drone_id);
        const char* values[1] = {drone_id_s.c_str()};
        PGresult* res = PQexecParams(
            conn_,
            "SELECT drone_id, latitude, longitude, altitude, battery_percentage, is_armed, flight_status, owner_organization "
            "FROM drone_state WHERE drone_id = $1::int ORDER BY updated_at DESC LIMIT 1",
            1, nullptr, values, nullptr, nullptr, 0);
        if (!res || PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
            if (res) PQclear(res);
            return std::nullopt;
        }

        ExtendedDroneState state{};
        state.drone_id = std::stoi(PQgetvalue(res, 0, 0));
        state.position.latitude = std::stod(PQgetvalue(res, 0, 1));
        state.position.longitude = std::stod(PQgetvalue(res, 0, 2));
        state.position.altitude = std::stod(PQgetvalue(res, 0, 3));
        state.battery_percentage = std::stod(PQgetvalue(res, 0, 4));
        state.is_armed = std::string(PQgetvalue(res, 0, 5)) == "t";
        state.flight_status = static_cast<FlightStatus>(std::stoi(PQgetvalue(res, 0, 6)));
        state.owner_organization = PQgetvalue(res, 0, 7);

        PQclear(res);
        return state;
    }

    bool saveEdgeDeviceInfo(const std::string& device_id, const std::string& device_name, 
                           const std::string& location, bool online) {
        if (!isConnected()) {
            return false;
        }
        const char* values[4];
        std::string online_s = online ? "true" : "false";
        values[0] = device_id.c_str();
        values[1] = device_name.c_str();
        values[2] = location.c_str();
        values[3] = online_s.c_str();
        PGresult* res = PQexecParams(
            conn_,
            "INSERT INTO edge_device_info (device_id, device_name, location, online, updated_at) "
            "VALUES ($1::text, $2::text, $3::text, $4::boolean, NOW()) "
            "ON CONFLICT (device_id) DO UPDATE "
            "SET device_name=EXCLUDED.device_name, location=EXCLUDED.location, online=EXCLUDED.online, updated_at=NOW()",
            4, nullptr, values, nullptr, nullptr, 0);
        const bool ok = res && PQresultStatus(res) == PGRES_COMMAND_OK;
        if (!ok) {
            std::cerr << "[PostgreSQL] saveEdgeDeviceInfo 失败: " << (res ? PQresultErrorMessage(res) : "null result") << std::endl;
        }
        if (res) PQclear(res);
        return ok;
    }

    std::optional<std::map<std::string, std::string>> getEdgeDeviceInfo(const std::string& device_id) {
        if (!isConnected()) {
            return std::nullopt;
        }
        const char* values[1] = {device_id.c_str()};
        PGresult* res = PQexecParams(
            conn_,
            "SELECT device_id, device_name, location, online FROM edge_device_info WHERE device_id = $1::text LIMIT 1",
            1, nullptr, values, nullptr, nullptr, 0);
        if (!res || PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
            if (res) PQclear(res);
            return std::nullopt;
        }
        std::map<std::string, std::string> info;
        info["device_id"] = PQgetvalue(res, 0, 0);
        info["device_name"] = PQgetvalue(res, 0, 1);
        info["location"] = PQgetvalue(res, 0, 2);
        info["online"] = std::string(PQgetvalue(res, 0, 3)) == "t" ? "true" : "false";
        PQclear(res);
        return info;
    }

    bool hasPrivacySeedData() {
        if (!isConnected()) {
            return false;
        }
        PGresult* res = PQexec(conn_, "SELECT COUNT(*) FROM privacy_location");
        if (!res || PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
            if (res) {
                PQclear(res);
            }
            return false;
        }
        const long cnt = std::strtol(PQgetvalue(res, 0, 0), nullptr, 10);
        PQclear(res);
        return cnt > 0;
    }

    bool savePrivacyLocation(const resources::AirspaceResourceManager::Location& loc) {
        if (!isConnected() || loc.name.empty()) {
            return false;
        }
        const char* values[6];
        std::string lat_s = std::to_string(loc.latitude);
        std::string lon_s = std::to_string(loc.longitude);
        std::string alt_s = std::to_string(loc.altitude);
        values[0] = loc.name.c_str();
        values[1] = loc.code.c_str();
        values[2] = lat_s.c_str();
        values[3] = lon_s.c_str();
        values[4] = alt_s.c_str();
        values[5] = loc.description.c_str();

        PGresult* res = PQexecParams(
            conn_,
            "INSERT INTO privacy_location (name, code, latitude, longitude, altitude, description, updated_at) "
            "VALUES ($1::text, $2::text, $3::double precision, $4::double precision, $5::double precision, $6::text, NOW()) "
            "ON CONFLICT (name) DO UPDATE SET "
            "code=EXCLUDED.code, latitude=EXCLUDED.latitude, longitude=EXCLUDED.longitude, "
            "altitude=EXCLUDED.altitude, description=EXCLUDED.description, updated_at=NOW()",
            6, nullptr, values, nullptr, nullptr, 0);
        const bool ok = res && PQresultStatus(res) == PGRES_COMMAND_OK;
        if (res) {
            PQclear(res);
        }
        return ok;
    }

    bool savePrivacyZoneRule(const std::string& zone_name, const resources::AirspaceResourceManager::AreaRestriction& rule) {
        if (!isConnected() || zone_name.empty()) {
            return false;
        }
        std::ostringstream ops;
        for (size_t i = 0; i < rule.allowed_operations.size(); ++i) {
            if (i > 0) {
                ops << ",";
            }
            ops << rule.allowed_operations[i];
        }
        const char* values[4];
        std::string privacy_s = rule.privacy_protection ? "true" : "false";
        std::string max_alt_s = std::to_string(rule.max_altitude);
        std::string ops_s = ops.str();
        values[0] = zone_name.c_str();
        values[1] = privacy_s.c_str();
        values[2] = max_alt_s.c_str();
        values[3] = ops_s.c_str();

        PGresult* res = PQexecParams(
            conn_,
            "INSERT INTO privacy_zone_rule (zone_name, privacy_protection, max_altitude, allowed_operations, updated_at) "
            "VALUES ($1::text, $2::boolean, $3::double precision, $4::text, NOW()) "
            "ON CONFLICT (zone_name) DO UPDATE SET "
            "privacy_protection=EXCLUDED.privacy_protection, max_altitude=EXCLUDED.max_altitude, "
            "allowed_operations=EXCLUDED.allowed_operations, updated_at=NOW()",
            4, nullptr, values, nullptr, nullptr, 0);
        const bool ok = res && PQresultStatus(res) == PGRES_COMMAND_OK;
        if (res) {
            PQclear(res);
        }
        return ok;
    }

private:
    PGconn* conn_;

    bool createTables() {
        if (!isConnected()) {
            return false;
        }
        const char* sql1 =
            "CREATE TABLE IF NOT EXISTS drone_state ("
            "id SERIAL PRIMARY KEY,"
            "drone_id INTEGER NOT NULL,"
            "latitude DOUBLE PRECISION DEFAULT 0,"
            "longitude DOUBLE PRECISION DEFAULT 0,"
            "altitude DOUBLE PRECISION DEFAULT 0,"
            "battery_percentage DOUBLE PRECISION DEFAULT 0,"
            "is_armed BOOLEAN DEFAULT FALSE,"
            "flight_status INTEGER DEFAULT 0,"
            "owner_organization TEXT DEFAULT '',"
            "updated_at TIMESTAMP NOT NULL DEFAULT NOW()"
            ")";
        const char* sql2 =
            "CREATE TABLE IF NOT EXISTS edge_device_info ("
            "device_id TEXT PRIMARY KEY,"
            "device_name TEXT DEFAULT '',"
            "location TEXT DEFAULT '',"
            "online BOOLEAN DEFAULT FALSE,"
            "updated_at TIMESTAMP NOT NULL DEFAULT NOW()"
            ")";
        const char* sql3 =
            "CREATE TABLE IF NOT EXISTS privacy_location ("
            "name TEXT PRIMARY KEY,"
            "code TEXT DEFAULT '',"
            "latitude DOUBLE PRECISION DEFAULT 0,"
            "longitude DOUBLE PRECISION DEFAULT 0,"
            "altitude DOUBLE PRECISION DEFAULT 0,"
            "description TEXT DEFAULT '',"
            "updated_at TIMESTAMP NOT NULL DEFAULT NOW()"
            ")";
        const char* sql4 =
            "CREATE TABLE IF NOT EXISTS privacy_zone_rule ("
            "zone_name TEXT PRIMARY KEY,"
            "privacy_protection BOOLEAN DEFAULT FALSE,"
            "max_altitude DOUBLE PRECISION DEFAULT 0,"
            "allowed_operations TEXT DEFAULT '',"
            "updated_at TIMESTAMP NOT NULL DEFAULT NOW()"
            ")";
        PGresult* res1 = PQexec(conn_, sql1);
        PGresult* res2 = PQexec(conn_, sql2);
        PGresult* res3 = PQexec(conn_, sql3);
        PGresult* res4 = PQexec(conn_, sql4);
        bool ok = res1 && res2 &&
                  PQresultStatus(res1) == PGRES_COMMAND_OK &&
                  PQresultStatus(res2) == PGRES_COMMAND_OK &&
                  res3 && res4 &&
                  PQresultStatus(res3) == PGRES_COMMAND_OK &&
                  PQresultStatus(res4) == PGRES_COMMAND_OK;
        if (!ok) {
            std::cerr << "[PostgreSQL] 创建表失败: "
                      << (res1 ? PQresultErrorMessage(res1) : "")
                      << (res2 ? PQresultErrorMessage(res2) : "") << std::endl;
        }
        if (res1) PQclear(res1);
        if (res2) PQclear(res2);
        if (res3) PQclear(res3);
        if (res4) PQclear(res4);
        return ok;
    }
};

// RedisDatabase 实现
class RedisDatabase {
public:
    RedisDatabase() : redis_context_(nullptr) {
    }

    ~RedisDatabase() {
        if (redis_context_) {
            redisFree(redis_context_);
        }
    }

    bool connect(const std::string& host, int port, const std::string& password) {
        std::cout << "[Redis] 尝试连接: " << host << ":" << port << std::endl;
        
        // 连接到 Redis 服务器
        redis_context_ = redisConnect(host.c_str(), port);
        if (redis_context_ == nullptr || redis_context_->err) {
            if (redis_context_) {
                std::cerr << "[Redis] 连接失败: " << redis_context_->errstr << std::endl;
                redisFree(redis_context_);
                redis_context_ = nullptr;
            } else {
                std::cerr << "[Redis] 连接失败: 无法分配内存" << std::endl;
            }
            return false;
        }
        
        // 如果有密码，进行认证
        if (!password.empty()) {
            redisReply* reply = (redisReply*)redisCommand(redis_context_, "AUTH %s", password.c_str());
            if (reply == nullptr || reply->type == REDIS_REPLY_ERROR) {
                std::cerr << "[Redis] 认证失败: " << (reply ? reply->str : "未知错误") << std::endl;
                freeReplyObject(reply);
                redisFree(redis_context_);
                redis_context_ = nullptr;
                return false;
            }
            freeReplyObject(reply);
        }
        
        std::cout << "[Redis] 连接成功" << std::endl;
        return true;
    }

    bool isConnected() const {
        return redis_context_ != nullptr;
    }

    void cacheDroneState(DroneId drone_id, const ExtendedDroneState& state) {
        if (!isConnected()) {
            return;
        }
        
        // 构建缓存键
        std::string key = "drone:state:" + std::to_string(drone_id);
        
        // 构建缓存值（简化处理，实际应该序列化为 JSON）
        std::string value = "{\"drone_id\": " + std::to_string(state.drone_id) + ", \"latitude\": " + std::to_string(state.position.latitude) + ", \"longitude\": " + std::to_string(state.position.longitude) + "}";
        
        // 设置缓存，过期时间为 5 分钟
        setCache(key, value, 300);
    }

    void cacheDroneInPrivacyZone(DroneId drone_id, const ExtendedDroneState& state, const std::string& zone_id) {
        if (!isConnected()) {
            return;
        }
        
        // 构建缓存键
        std::string key = "drone:privacy:" + std::to_string(drone_id) + ":" + zone_id;
        
        // 构建缓存值（简化处理，实际应该序列化为 JSON）
        std::string value = "{\"drone_id\": " + std::to_string(state.drone_id) + ", \"latitude\": " + std::to_string(state.position.latitude) + ", \"longitude\": " + std::to_string(state.position.longitude) + ", \"altitude\": " + std::to_string(state.position.altitude) + ", \"zone_id\": \"" + zone_id + "\"}";
        
        // 设置缓存，过期时间为 1 小时（隐私数据需要保留更长时间）
        setCache(key, value, 3600);
    }

    void cacheFlightPathInPrivacyZone(DroneId drone_id, const std::vector<Waypoint>& path, const std::string& zone_id) {
        if (!isConnected()) {
            return;
        }
        
        // 构建缓存键
        std::string key = "drone:privacy:path:" + std::to_string(drone_id) + ":" + zone_id;
        
        // 构建缓存值（简化处理，实际应该序列化为 JSON）
        std::string value = "{\"drone_id\": " + std::to_string(drone_id) + ", \"zone_id\": \"" + zone_id + "\", \"waypoints\": [";
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) value += ", ";
            value += "{\"latitude\": " + std::to_string(path[i].position.latitude) + ", \"longitude\": " + std::to_string(path[i].position.longitude) + ", \"altitude\": " + std::to_string(path[i].position.altitude) + "}";
        }
        value += "]}";
        
        // 设置缓存，过期时间为 24 小时（飞行路径需要保留更长时间）
        setCache(key, value, 86400);
    }

    std::vector<std::string> getDronesInPrivacyZones() {
        if (!isConnected()) {
            return {};
        }
        
        // 执行 KEYS 命令，查找所有隐私区域内的无人机
        redisReply* reply = (redisReply*)redisCommand(redis_context_, "KEYS drone:privacy:*");
        if (reply == nullptr || reply->type != REDIS_REPLY_ARRAY) {
            freeReplyObject(reply);
            return {};
        }
        
        std::vector<std::string> keys;
        for (size_t i = 0; i < reply->elements; ++i) {
            keys.push_back(reply->element[i]->str);
        }
        
        freeReplyObject(reply);
        return keys;
    }

    std::optional<ExtendedDroneState> getCachedDroneState(DroneId drone_id) {
        if (!isConnected()) {
            return std::nullopt;
        }
        
        // 构建缓存键
        std::string key = "drone:state:" + std::to_string(drone_id);
        
        // 获取缓存
        auto value = getCache(key);
        if (!value) {
            return std::nullopt;
        }
        
        auto extract_number = [&](const std::string& src, const std::string& field) -> std::optional<double> {
            const std::regex pattern("\"" + field + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
            std::smatch match;
            if (!std::regex_search(src, match, pattern) || match.size() < 2) {
                return std::nullopt;
            }

            try {
                return std::stod(match[1].str());
            } catch (...) {
                return std::nullopt;
            }
        };

        ExtendedDroneState state{};
        state.drone_id = drone_id;

        auto latitude = extract_number(*value, "latitude");
        auto longitude = extract_number(*value, "longitude");
        auto altitude = extract_number(*value, "altitude");

        if (!latitude || !longitude) {
            std::cerr << "[Redis] getCachedDroneState 解析失败: 缺少经纬度字段, key=" << key << std::endl;
            return std::nullopt;
        }

        state.position.latitude = *latitude;
        state.position.longitude = *longitude;
        state.position.altitude = altitude.value_or(0.0);
        return state;
    }

    bool setCache(const std::string& key, const std::string& value, int expiration) {
        if (!isConnected()) {
            return false;
        }
        
        // 执行 SET 命令，设置过期时间
        redisReply* reply = (redisReply*)redisCommand(redis_context_, "SETEX %s %d %s", key.c_str(), expiration, value.c_str());
        if (reply == nullptr || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "[Redis] 设置缓存失败: " << (reply ? reply->str : "未知错误") << std::endl;
            freeReplyObject(reply);
            return false;
        }
        
        freeReplyObject(reply);
        return true;
    }

    std::optional<std::string> getCache(const std::string& key) {
        if (!isConnected()) {
            return std::nullopt;
        }
        
        // 执行 GET 命令
        redisReply* reply = (redisReply*)redisCommand(redis_context_, "GET %s", key.c_str());
        if (reply == nullptr) {
            std::cerr << "[Redis] 获取缓存失败: 命令执行错误" << std::endl;
            return std::nullopt;
        }
        
        if (reply->type == REDIS_REPLY_NIL) {
            // 键不存在
            freeReplyObject(reply);
            return std::nullopt;
        } else if (reply->type == REDIS_REPLY_STRING) {
            // 键存在，返回值
            std::string value = reply->str;
            freeReplyObject(reply);
            return value;
        } else {
            // 其他错误
            std::cerr << "[Redis] 获取缓存失败: 非字符串类型" << std::endl;
            freeReplyObject(reply);
            return std::nullopt;
        }
    }

    bool deleteCache(const std::string& key) {
        if (!isConnected()) {
            return false;
        }
        
        // 执行 DEL 命令
        redisReply* reply = (redisReply*)redisCommand(redis_context_, "DEL %s", key.c_str());
        if (reply == nullptr || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "[Redis] 删除缓存失败: " << (reply ? reply->str : "未知错误") << std::endl;
            freeReplyObject(reply);
            return false;
        }
        
        freeReplyObject(reply);
        return true;
    }

    bool hasPrivacySeedData() {
        if (!isConnected()) {
            return false;
        }
        redisReply* reply = (redisReply*)redisCommand(redis_context_, "KEYS privacy:location:name:*");
        if (reply == nullptr || reply->type != REDIS_REPLY_ARRAY) {
            if (reply) {
                freeReplyObject(reply);
            }
            return false;
        }
        bool has_data = reply->elements > 0;
        freeReplyObject(reply);
        return has_data;
    }

private:
    redisContext* redis_context_;
};

// DatabaseManager 实现
DatabaseManager::DatabaseManager() {
}

DatabaseManager::~DatabaseManager() {
}

bool DatabaseManager::initialize(const std::string& postgres_host, int postgres_port, const std::string& postgres_db, 
                               const std::string& postgres_user, const std::string& postgres_password,
                               const std::string& redis_host, int redis_port, const std::string& redis_password) {
    // 初始化PostgreSQL
    bool pg_connected = false;
    if (!postgres_host.empty() && postgres_port > 0) {
        postgres_db_ = std::make_unique<PostgreSQLDatabase>();
        pg_connected = postgres_db_->connect(postgres_host, postgres_port, postgres_db, postgres_user, postgres_password);
        if (!pg_connected) {
            std::cerr << "[数据库] PostgreSQL初始化失败，将继续初始化Redis" << std::endl;
            // 不返回false，继续初始化Redis
        } else {
            std::cout << "[数据库] PostgreSQL初始化成功" << std::endl;
        }
    } else {
        std::cout << "[数据库] 跳过PostgreSQL初始化（当前进程使用Redis模式）" << std::endl;
    }

    // 初始化Redis
    redis_db_ = std::make_unique<RedisDatabase>();
    if (!redis_db_->connect(redis_host, redis_port, redis_password)) {
        std::cerr << "[数据库] Redis初始化失败" << std::endl;
        return false;
    }

    std::cout << "[数据库] 初始化成功（PostgreSQL=" << (pg_connected ? "connected" : "disconnected")
              << ", Redis=connected）" << std::endl;

    // 每次启动自检隐私数据，不存在则从配置文件回填到数据库。
    bool has_privacy_data = false;
    if (postgres_db_ && postgres_db_->isConnected()) {
        has_privacy_data = postgres_db_->hasPrivacySeedData();
    }
    if (!has_privacy_data && redis_db_ && redis_db_->isConnected()) {
        has_privacy_data = redis_db_->hasPrivacySeedData();
    }
    if (!has_privacy_data) {
        resources::AirspaceResourceManager manager;
        const std::vector<std::string> config_paths = {
            "config/airspace_config.yaml",
            "./config/airspace_config.yaml",
            "../config/airspace_config.yaml",
            "../../config/airspace_config.yaml",
            "build/config/airspace_config.yaml",
            "../build/config/airspace_config.yaml"
        };
        bool loaded = false;
        for (const auto& path : config_paths) {
            if (manager.loadLocationsFromConfig(path)) {
                loaded = true;
                std::cout << "[数据库] 检测到隐私数据为空，已从配置加载: " << path << std::endl;
                break;
            }
        }
        if (loaded) {
            const auto locations = manager.getAllLocations();
            for (const auto& loc : locations) {
                if (postgres_db_ && postgres_db_->isConnected()) {
                    (void)postgres_db_->savePrivacyLocation(loc);
                }
                if (redis_db_ && redis_db_->isConnected() && !loc.name.empty()) {
                    std::ostringstream payload;
                    payload << "{"
                            << "\"name\":\"" << loc.name << "\","
                            << "\"code\":\"" << loc.code << "\","
                            << "\"latitude\":" << loc.latitude << ","
                            << "\"longitude\":" << loc.longitude << ","
                            << "\"altitude\":" << loc.altitude
                            << "}";
                    (void)redis_db_->setCache("privacy:location:name:" + loc.name, payload.str(), 24 * 3600);
                }
            }

            const auto& rules = manager.getAreaRestrictions();
            for (const auto& [zone_name, rule] : rules) {
                if (postgres_db_ && postgres_db_->isConnected()) {
                    (void)postgres_db_->savePrivacyZoneRule(zone_name, rule);
                }
                if (redis_db_ && redis_db_->isConnected() && rule.privacy_protection) {
                    std::ostringstream payload;
                    payload << "{"
                            << "\"zone_id\":\"" << zone_name << "\","
                            << "\"privacy_protection\":true,"
                            << "\"max_altitude\":" << rule.max_altitude
                            << "}";
                    (void)redis_db_->setCache("privacy:zone:rule:" + zone_name, payload.str(), 24 * 3600);
                }
            }
            std::cout << "[数据库] 隐私数据初始化完成: location=" << locations.size()
                      << ", rules=" << rules.size() << std::endl;
        } else {
            std::cerr << "[数据库] 隐私数据为空且配置文件未找到，跳过初始化" << std::endl;
        }
    } else {
        std::cout << "[数据库] 隐私数据已存在，跳过配置回填" << std::endl;
    }
    return true;
}

bool DatabaseManager::saveDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    bool postgres_ok = false;
    if (postgres_db_ && postgres_db_->isConnected()) {
        postgres_ok = postgres_db_->saveDroneState(drone_id, state);
    }

    bool redis_ok = false;
    if (redis_db_ && redis_db_->isConnected()) {
        redis_db_->cacheDroneState(drone_id, state);
        redis_ok = true;
    }

    return postgres_ok || redis_ok;
}

bool DatabaseManager::saveDroneInPrivacyZone(DroneId drone_id, const ExtendedDroneState& state, const std::string& zone_id) {
    // 缓存到Redis
    redis_db_->cacheDroneInPrivacyZone(drone_id, state, zone_id);
    return true;
}

bool DatabaseManager::saveFlightPathInPrivacyZone(DroneId drone_id, const std::vector<Waypoint>& path, const std::string& zone_id) {
    // 缓存到Redis
    redis_db_->cacheFlightPathInPrivacyZone(drone_id, path, zone_id);
    return true;
}

std::vector<std::string> DatabaseManager::getDronesInPrivacyZones() {
    // 从Redis获取隐私区域内的无人机
    return redis_db_->getDronesInPrivacyZones();
}

std::optional<ExtendedDroneState> DatabaseManager::getDroneState(DroneId drone_id) {
    // 先尝试从Redis缓存获取
    auto cached_state = redis_db_->getCachedDroneState(drone_id);
    if (cached_state) {
        return cached_state;
    }

    // 从PostgreSQL获取
    return (postgres_db_ && postgres_db_->isConnected()) ? postgres_db_->getDroneState(drone_id) : std::nullopt;
}

bool DatabaseManager::saveDroneAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) {
    if (!redis_db_ || !redis_db_->isConnected()) {
        return false;
    }

    std::string value = "{";
    size_t index = 0;
    for (const auto& [k, v] : attributes) {
        if (index++ > 0) {
            value += ",";
        }
        value += "\"" + k + "\":\"" + v + "\"";
    }
    value += "}";

    return redis_db_->setCache("drone:attributes:" + std::to_string(drone_id), value, 24 * 3600);
}

std::optional<std::map<std::string, std::string>> DatabaseManager::getDroneAttributes(DroneId drone_id) {
    if (redis_db_ && redis_db_->isConnected()) {
        auto cached = redis_db_->getCache("drone:attributes:" + std::to_string(drone_id));
        if (cached && !cached->empty()) {
            // 这里保留轻量实现：返回原始JSON载荷，调用方可自行解析。
            return std::map<std::string, std::string>{{"raw_json", *cached}};
        }
    }
    return std::nullopt;
}

bool DatabaseManager::saveEdgeDeviceInfo(const std::string& device_id, const std::string& device_name, 
                                       const std::string& location, bool online) {
    bool postgres_ok = false;
    if (postgres_db_ && postgres_db_->isConnected()) {
        postgres_ok = postgres_db_->saveEdgeDeviceInfo(device_id, device_name, location, online);
    }

    bool redis_ok = false;
    if (redis_db_ && redis_db_->isConnected()) {
        std::string value = "{\"device_id\":\"" + device_id + "\",\"device_name\":\"" + device_name +
                            "\",\"location\":\"" + location + "\",\"online\":" + (online ? "true" : "false") + "}";
        redis_ok = redis_db_->setCache("edge:device:" + device_id, value, 24 * 3600);
    }

    return postgres_ok || redis_ok;
}

std::optional<std::map<std::string, std::string>> DatabaseManager::getEdgeDeviceInfo(const std::string& device_id) {
    if (postgres_db_ && postgres_db_->isConnected()) {
        auto pg_data = postgres_db_->getEdgeDeviceInfo(device_id);
        if (pg_data) {
            return pg_data;
        }
    }
    if (redis_db_ && redis_db_->isConnected()) {
        auto cached = redis_db_->getCache("edge:device:" + device_id);
        if (cached && !cached->empty()) {
            return std::map<std::string, std::string>{{"raw_json", *cached}};
        }
    }
    return std::nullopt;
}

bool DatabaseManager::saveAccessDecision(DroneId drone_id, bool granted, const std::string& reason, const std::string& timestamp) {
    if (!redis_db_ || !redis_db_->isConnected()) {
        return false;
    }
    const std::string value = "{\"drone_id\":" + std::to_string(drone_id) +
                              ",\"granted\":" + (granted ? "true" : "false") +
                              ",\"reason\":\"" + reason +
                              "\",\"timestamp\":\"" + timestamp + "\"}";
    return redis_db_->setCache("access:decision:" + std::to_string(drone_id) + ":" + timestamp, value, 24 * 3600);
}

std::vector<std::map<std::string, std::string>> DatabaseManager::getAccessDecisions(DroneId drone_id, int limit) {
    // 这里可以实现从数据库获取访问决策的逻辑
    return {};
}

bool DatabaseManager::setCache(const std::string& key, const std::string& value, int expiration) {
    const bool ok = redis_db_->setCache(key, value, expiration);
    if (key.rfind("privacy:", 0) == 0) {
        std::cout << "[数据库][DEBUG] Redis写入隐私缓存: key=" << key
                  << ", value_len=" << value.size()
                  << ", ttl=" << expiration
                  << ", ok=" << (ok ? "true" : "false") << std::endl;
    }
    return ok;
}

std::optional<std::string> DatabaseManager::getCache(const std::string& key) {
    auto value = redis_db_->getCache(key);
    if (key.rfind("privacy:", 0) == 0) {
        std::cout << "[数据库][DEBUG] Redis读取隐私缓存: key=" << key
                  << ", hit=" << (value.has_value() ? "true" : "false");
        if (value.has_value()) {
            std::cout << ", value_len=" << value->size();
        }
        std::cout << std::endl;
    }
    return value;
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