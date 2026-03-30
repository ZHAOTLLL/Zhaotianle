/**
 * PostgreSQL数据库实现
 */
#include "database/database_manager.hpp"
#include <libpq-fe.h>
#include <iostream>
#include <stdexcept>

namespace drone_control {

class PostgreSQLDatabase {
public:
    PostgreSQLDatabase() : conn_(nullptr) {
    }

    ~PostgreSQLDatabase() {
        if (conn_) {
            PQfinish(conn_);
        }
    }

    bool connect(const std::string& host, int port, const std::string& dbname, 
                const std::string& user, const std::string& password) {
        std::string conninfo = "host='" + host + "' port='" + std::to_string(port) + "' dbname='" + dbname + "' user='" + user + "' password='" + password + "'";
        conn_ = PQconnectdb(conninfo.c_str());

        if (PQstatus(conn_) != CONNECTION_OK) {
            std::cerr << "[PostgreSQL] 连接失败: " << PQerrorMessage(conn_) << std::endl;
            PQfinish(conn_);
            conn_ = nullptr;
            return false;
        }

        std::cout << "[PostgreSQL] 连接成功" << std::endl;
        createTables();
        return true;
    }

    bool isConnected() const {
        return conn_ != nullptr && PQstatus(conn_) == CONNECTION_OK;
    }

    // 无人机状态管理
    bool saveDroneState(DroneId drone_id, const ExtendedDroneState& state) {
        if (!isConnected()) {
            return false;
        }

        const char* sql = R"(
            INSERT INTO drone_states (drone_id, latitude, longitude, altitude, speed, ground_speed, 
                                     battery_percentage, is_armed, flight_status, last_update)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, NOW())
            ON CONFLICT (drone_id) DO UPDATE SET
                latitude = $2,
                longitude = $3,
                altitude = $4,
                speed = $5,
                ground_speed = $6,
                battery_percentage = $7,
                is_armed = $8,
                flight_status = $9,
                last_update = NOW()
        )";

        PGresult* res = PQexecParams(conn_, sql, 9, nullptr,
            new const char*[9]{ 
                std::to_string(drone_id).c_str(),
                std::to_string(state.position.latitude).c_str(),
                std::to_string(state.position.longitude).c_str(),
                std::to_string(state.position.altitude).c_str(),
                std::to_string(state.speed).c_str(),
                std::to_string(state.ground_speed).c_str(),
                std::to_string(state.battery_percentage).c_str(),
                state.is_armed ? "true" : "false",
                std::to_string(static_cast<int>(state.flight_status)).c_str()
            },
            nullptr, nullptr, 0
        );

        if (PQresultStatus(res) != PGRES_COMMAND_OK) {
            std::cerr << "[PostgreSQL] 保存无人机状态失败: " << PQerrorMessage(conn_) << std::endl;
            PQclear(res);
            return false;
        }

        PQclear(res);
        return true;
    }

    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id) {
        if (!isConnected()) {
            return std::nullopt;
        }

        const char* sql = "SELECT latitude, longitude, altitude, speed, ground_speed, battery_percentage, is_armed, flight_status FROM drone_states WHERE drone_id = $1";
        PGresult* res = PQexecParams(conn_, sql, 1, nullptr,
            new const char*[1]{ std::to_string(drone_id).c_str() },
            nullptr, nullptr, 0
        );

        if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
            PQclear(res);
            return std::nullopt;
        }

        ExtendedDroneState state;
        state.drone_id = drone_id;
        state.position.latitude = std::stod(PQgetvalue(res, 0, 0));
        state.position.longitude = std::stod(PQgetvalue(res, 0, 1));
        state.position.altitude = std::stod(PQgetvalue(res, 0, 2));
        state.speed = std::stod(PQgetvalue(res, 0, 3));
        state.ground_speed = std::stod(PQgetvalue(res, 0, 4));
        state.battery_percentage = std::stod(PQgetvalue(res, 0, 5));
        state.is_armed = strcmp(PQgetvalue(res, 0, 6), "true") == 0;
        state.flight_status = static_cast<FlightStatus>(std::stoi(PQgetvalue(res, 0, 7)));

        PQclear(res);
        return state;
    }

    // 边缘设备管理
    bool saveEdgeDeviceInfo(const std::string& device_id, const std::string& device_name, 
                           const std::string& location, bool online) {
        if (!isConnected()) {
            return false;
        }

        const char* sql = R"(
            INSERT INTO edge_devices (device_id, device_name, location, online, last_heartbeat)
            VALUES ($1, $2, $3, $4, NOW())
            ON CONFLICT (device_id) DO UPDATE SET
                device_name = $2,
                location = $3,
                online = $4,
                last_heartbeat = NOW()
        )";

        PGresult* res = PQexecParams(conn_, sql, 4, nullptr,
            new const char*[4]{ 
                device_id.c_str(),
                device_name.c_str(),
                location.c_str(),
                online ? "true" : "false"
            },
            nullptr, nullptr, 0
        );

        if (PQresultStatus(res) != PGRES_COMMAND_OK) {
            std::cerr << "[PostgreSQL] 保存边缘设备信息失败: " << PQerrorMessage(conn_) << std::endl;
            PQclear(res);
            return false;
        }

        PQclear(res);
        return true;
    }

    std::optional<std::map<std::string, std::string>> getEdgeDeviceInfo(const std::string& device_id) {
        if (!isConnected()) {
            return std::nullopt;
        }

        const char* sql = "SELECT device_name, location, online, last_heartbeat FROM edge_devices WHERE device_id = $1";
        PGresult* res = PQexecParams(conn_, sql, 1, nullptr,
            new const char*[1]{ device_id.c_str() },
            nullptr, nullptr, 0
        );

        if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
            PQclear(res);
            return std::nullopt;
        }

        std::map<std::string, std::string> info;
        info["device_name"] = PQgetvalue(res, 0, 0);
        info["location"] = PQgetvalue(res, 0, 1);
        info["online"] = strcmp(PQgetvalue(res, 0, 2), "true") == 0 ? "true" : "false";
        info["last_heartbeat"] = PQgetvalue(res, 0, 3);

        PQclear(res);
        return info;
    }

private:
    PGconn* conn_;

    void createTables() {
        // 创建无人机状态表
        const char* create_drone_states = R"(
            CREATE TABLE IF NOT EXISTS drone_states (
                drone_id INTEGER PRIMARY KEY,
                latitude DOUBLE PRECISION,
                longitude DOUBLE PRECISION,
                altitude DOUBLE PRECISION,
                speed DOUBLE PRECISION,
                ground_speed DOUBLE PRECISION,
                battery_percentage DOUBLE PRECISION,
                is_armed BOOLEAN,
                flight_status INTEGER,
                last_update TIMESTAMP DEFAULT NOW()
            )
        )";

        // 创建边缘设备表
        const char* create_edge_devices = R"(
            CREATE TABLE IF NOT EXISTS edge_devices (
                device_id TEXT PRIMARY KEY,
                device_name TEXT,
                location TEXT,
                online BOOLEAN,
                last_heartbeat TIMESTAMP DEFAULT NOW()
            )
        )";

        // 创建访问决策表
        const char* create_access_decisions = R"(
            CREATE TABLE IF NOT EXISTS access_decisions (
                id SERIAL PRIMARY KEY,
                drone_id INTEGER,
                granted BOOLEAN,
                reason TEXT,
                timestamp TIMESTAMP DEFAULT NOW(),
                FOREIGN KEY (drone_id) REFERENCES drone_states(drone_id)
            )
        )";

        executeSql(create_drone_states);
        executeSql(create_edge_devices);
        executeSql(create_access_decisions);
    }

    void executeSql(const char* sql) {
        PGresult* res = PQexec(conn_, sql);
        if (PQresultStatus(res) != PGRES_COMMAND_OK) {
            std::cerr << "[PostgreSQL] 执行SQL失败: " << PQerrorMessage(conn_) << std::endl;
        }
        PQclear(res);
    }
};

} // namespace drone_control