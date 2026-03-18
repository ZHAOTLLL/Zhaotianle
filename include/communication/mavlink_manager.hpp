/**
 * MAVLink 连接管理
 * 负责与 PX4 的 MAVLink 连接、心跳、遥测与指令下发，以及访问控制相关的认证与任务交互。
 */
#pragma once

#include "common/types.hpp"
#include "common/drone_attributes.hpp"
#include "authentication/authentication_provider.hpp"
#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>

#ifdef HAVE_MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/ftp/ftp.h>
#endif

namespace drone_control {

class DroneStateManager;
class AccessControlEngine;
struct AccessRequest;
struct AccessDecision;
struct FlightPlan;

/** 管理与 PX4 的 MAVLink 连接、消息收发与状态同步 */
class MAVLinkManager {
public:
    // 连接状态枚举
    enum class ConnectionStatus {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        AUTHENTICATING,
        AUTHENTICATED,
        AUTHENTICATION_FAILED,
        FAILED
    };

    // 连接事件回调类型
    using ConnectionCallback = std::function<void(DroneId, ConnectionStatus)>;
    using TelemetryCallback = std::function<void(const ExtendedDroneState&)>;
    using AuthenticationCallback = std::function<void(DroneId, bool, const std::string&)>;

    MAVLinkManager();
    ~MAVLinkManager();

    // 连接管理
    /** 指定 URL 主动连接一架机（保留用于兼容） */
    bool connectToDrone(const std::string& connection_url);
    /** 局域网发现：监听 listen_url，自动发现并连接所有发送 MAVLink 心跳的无人机 */
    bool startDiscovery(const std::string& listen_url);
    void disconnectFromDrone(DroneId drone_id);
    void disconnectAll();
    
    // 状态查询
    std::vector<DroneId> getConnectedDrones() const;
    ConnectionStatus getConnectionStatus(DroneId drone_id) const;
    bool isDroneConnected(DroneId drone_id) const;
    
    // 回调注册
    void registerConnectionCallback(ConnectionCallback callback);
    void registerTelemetryCallback(TelemetryCallback callback);
    void registerAuthenticationCallback(AuthenticationCallback callback);
    
    // 命令发送
    bool sendCommand(DroneId drone_id, const std::string& command, 
                     const std::map<std::string, std::string>& parameters = {});
    
    // 获取无人机状态
    std::optional<ExtendedDroneState> getDroneState(DroneId drone_id) const;
    
    // 认证管理
    void setAuthenticationProvider(std::unique_ptr<AuthenticationProvider> provider);
    bool authenticateDrone(DroneId drone_id, const std::string& credential, const std::string& credential_type);
    bool isDroneAuthenticated(DroneId drone_id) const;
    
    // 状态管理器集成
    void setStateManager(std::shared_ptr<DroneStateManager> state_manager);
    
    // 访问控制引擎集成
    void setAccessControlEngine(std::shared_ptr<AccessControlEngine> engine);
    
    // 发送地理围栏解锁凭证和飞行计划
    void sendGeofenceSignature(uint32_t drone_id, const std::string& signature, const std::chrono::seconds& validity_duration);
    void sendFlightPlan(uint32_t drone_id, const FlightPlan& flight_plan);
    
    /** 手动确认任务并请求证书：发送 MAV_CMD_USER_1，等待后发 USER_2，触发访问控制流程。返回是否已发送。 */
    bool confirmMissionAndRequestCertificate(uint32_t drone_id);
    
    // 启动和停止服务
    bool start();
    /** 仅将 running_ 置为 true，不启动 5 条 worker 线程；与 MAVLinkManagerConcurrentRunner 配合使用时调用 */
    void startConcurrentMode();
    void stop();
    bool isRunning() const;

    /**
     * 单次轮询接口，供 MAVLinkManagerConcurrentRunner 等事件驱动调度使用。
     * 每个方法执行对应 worker 的一轮逻辑，不阻塞；与 start() 二选一使用。
     */
    void runConnectionCycleOnce();
    void runTelemetryCycleOnce();
    void runHeartbeatCycleOnce();
    void runAuthenticationCycleOnce();
    void runReconnectionCycleOnce();

private:
    struct DroneConnection {
        DroneId drone_id;
        std::string connection_url;
        ConnectionStatus status;
        std::chrono::steady_clock::time_point last_heartbeat;
        int retry_count;
        ExtendedDroneState state;
        
        // 认证相关
        bool is_authenticated;
        std::string authentication_credential;
        std::string credential_type;
        std::map<std::string, std::string> extracted_attributes;
        
#ifdef HAVE_MAVSDK
        std::shared_ptr<mavsdk::System> system;
        std::unique_ptr<mavsdk::Telemetry> telemetry;
        std::unique_ptr<mavsdk::Action> action;
        std::unique_ptr<mavsdk::Mission> mission;
        std::unique_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough;
        std::unique_ptr<mavsdk::Ftp> ftp;
        // 使用 void* 避免头文件依赖，在 cpp 文件中转换为具体类型
        std::unique_ptr<void, void(*)(void*)> offboard;
#endif
        
        DroneConnection() 
            : drone_id(0), status(ConnectionStatus::DISCONNECTED)
            , last_heartbeat(std::chrono::steady_clock::now())
            , retry_count(0), is_authenticated(false)
#ifdef HAVE_MAVSDK
            , offboard(nullptr, [](void*){})
#endif
            {}
    };

    // 内部方法
    void connectionWorker();
    void telemetryWorker();
    void heartbeatChecker();
    void authenticationWorker();
    void reconnectionWorker();
    
    bool establishConnection(DroneConnection& conn);
#ifdef HAVE_MAVSDK
    bool establishConnectionFromSystem(DroneConnection& conn, const std::shared_ptr<mavsdk::System>& system);
#endif
    void runDiscoveryCycleOnce();
    void handleConnectionLost(DroneId drone_id);
    void updateDroneState(DroneId drone_id, const ExtendedDroneState& state);
    
    bool performAuthentication(DroneConnection& conn);
    void handleAuthenticationResult(DroneId drone_id, bool success, const std::string& message);
    
    // 任务目的处理
    void handleMissionPurposeUpload(uint32_t drone_id, const std::string& mission_name, 
                                   const std::string& target_location, const std::string& operation_type,
                                   const std::string& certificate_hash, const std::string& mission_description,
                                   uint64_t timestamp);
    
    // 任务确认和证书请求
    void sendMissionConfirmation(uint32_t drone_id);
    void requestCertificateUpload(uint32_t drone_id);
    
    // 访问控制评估
    void performAccessControlEvaluation(uint32_t drone_id);
    void sendAccessControlResult(uint32_t drone_id, bool access_granted, const std::string& reason);
    
    // 证书上传处理
    void handleCertificateUpload(uint32_t drone_id, uint32_t certificate_size, uint16_t chunk_index,
                                uint16_t total_chunks, const std::string& certificate_data, uint64_t timestamp);
    std::string base64Decode(const std::string& encoded);
    
    // FTP证书文件下载
    void downloadCertificateViaFTP(DroneId drone_id);
    void downloadCertificateViaFTPWithRetry(DroneId drone_id, int max_retries = 3);
    void handleCertificateFileDownloaded(DroneId drone_id, const std::string& file_path, bool success);
    AccessRequest constructAccessRequest(uint32_t drone_id, const std::string& mission_name,
                                       const std::string& target_location, const std::string& operation_type,
                                       const std::string& certificate_hash, const std::string& mission_description);
    
    // MAVLink消息处理
    void setupMavlinkMessageHandlers(DroneConnection& conn);
#ifdef HAVE_MAVSDK
    void handleMavlinkMessage(DroneId drone_id, const mavlink_message_t& message);
#endif
    
    DroneId generateDroneId();
    
    // 成员变量
#ifdef HAVE_MAVSDK
    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
#endif
    
    mutable std::mutex connections_mutex_;
    std::map<DroneId, std::unique_ptr<DroneConnection>> connections_;
    std::map<std::string, DroneId> url_to_drone_id_;
    bool discovery_mode_{false};
    std::string discovery_listen_url_;
    std::map<uint64_t, DroneId> system_key_to_drone_id_;
    
    std::atomic<bool> running_;
    std::thread connection_thread_;
    std::thread telemetry_thread_;
    std::thread heartbeat_thread_;
    std::thread authentication_thread_;
    std::thread reconnection_thread_;
    
    std::vector<ConnectionCallback> connection_callbacks_;
    std::vector<TelemetryCallback> telemetry_callbacks_;
    std::vector<AuthenticationCallback> authentication_callbacks_;
    
    // 认证和状态管理
    std::unique_ptr<AuthenticationProvider> auth_provider_;
    std::shared_ptr<DroneStateManager> state_manager_;
    std::shared_ptr<AccessControlEngine> access_control_engine_;
    
    DroneId next_drone_id_;
    
    // 任务目的缓存，避免重复评估
    struct MissionPurposeCache {
        std::string mission_name;
        std::string target_location;
        std::string operation_type;
        std::string certificate_hash;
        std::string mission_description;
        std::string certificate_content;  // 存储下载的证书内容
        std::chrono::steady_clock::time_point last_evaluated;
        bool last_decision_granted;
        std::string last_decision_reason;
    };
    
    // 用于缓存分片的STATUSTEXT消息
    struct StatusTextBuffer {
        std::string buffer;
        std::chrono::steady_clock::time_point last_update;
    };
    std::unordered_map<DroneId, MissionPurposeCache> mission_purpose_cache_;
    std::unordered_map<DroneId, StatusTextBuffer> statustext_buffers_;
    static constexpr std::chrono::minutes CACHE_VALIDITY_DURATION{5};  // 缓存有效期5分钟
    
    // 证书上传缓存
    struct ValidatedCertificate {
        std::string certificate_data;
        std::chrono::system_clock::time_point validated_time;
        std::map<std::string, std::string> attributes;
    };
    std::unordered_map<DroneId, std::string> certificate_cache_;  // 证书分块缓存
    std::unordered_map<DroneId, ValidatedCertificate> validated_certificates_;  // 已验证的证书
    
    // 遥测单次轮询用状态（供 runTelemetryCycleOnce 与 telemetryWorker 共用）
    int telemetry_cycle_count_{0};
    std::chrono::steady_clock::time_point last_telemetry_status_report_;
    std::chrono::steady_clock::time_point last_telemetry_callback_time_;
    int total_telemetry_callbacks_processed_{0};
    
    // 配置参数
    static constexpr int MAX_RETRY_COUNT = 3;
    static constexpr std::chrono::seconds HEARTBEAT_TIMEOUT{60};  // 增加心跳超时时间到60秒
    static constexpr std::chrono::seconds RETRY_INTERVAL{5};
    static constexpr std::chrono::milliseconds TELEMETRY_INTERVAL{500};
    static constexpr std::chrono::seconds RECONNECT_INTERVAL{10};  // 重连间隔
    
};

} // namespace drone_control