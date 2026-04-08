/**
 * MAVLink 连接管理实现
 * 连接 PX4、心跳、遥测、指令下发及与访问控制相关的认证与任务交互。
 */
#include "communication/mavlink_manager.hpp"
#include "state/drone_state_manager.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/access_request.hpp"
#include "access_control/access_decision.hpp"
#include "flight_control/flight_plan.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <set>
#include <thread>
#include <atomic>
#include <cmath>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

#ifdef HAVE_MAVSDK
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavlink/mavlink_types.h>
#include <mavlink/common/mavlink.h>
#endif
#include <optional>


namespace drone_control {

MAVLinkManager::MAVLinkManager()
    : running_(false)
    , next_drone_id_(1) {
#ifdef HAVE_MAVSDK
    mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::GroundStation);
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(config);
#endif
}
MAVLinkManager::~MAVLinkManager() {
    stop();
}

bool MAVLinkManager::start() {
    if (running_.load()) {
        return true;
    }
    
    running_.store(true);
    
#ifdef USE_BOOST_ASIO
    // 初始化Boost.Asio事件循环
    io_context_ = std::make_unique<boost::asio::io_context>();
    work_ = std::make_unique<boost::asio::io_context::work>(*io_context_);
    
    // 启动线程池
    asio_threads_.resize(ASIO_THREAD_POOL_SIZE);
    for (std::size_t i = 0; i < ASIO_THREAD_POOL_SIZE; ++i) {
        asio_threads_[i] = std::thread([this]() {
            io_context_->run();
        });
    }
    
    // 发布初始任务
    boost::asio::post(*io_context_, [this]() {
        runConnectionCycleOnce();
    });
    boost::asio::post(*io_context_, [this]() {
        runTelemetryCycleOnce();
    });
    boost::asio::post(*io_context_, [this]() {
        runHeartbeatCycleOnce();
    });
    boost::asio::post(*io_context_, [this]() {
        runAuthenticationCycleOnce();
    });
    boost::asio::post(*io_context_, [this]() {
        runReconnectionCycleOnce();
    });
    
    std::cout << "MAVLinkManager started with Boost.Asio event loop" << std::endl;
#else
    // 兼容模式：使用原有轮询线程
    connection_thread_ = std::thread(&MAVLinkManager::connectionWorker, this);
    telemetry_thread_ = std::thread(&MAVLinkManager::telemetryWorker, this);
    heartbeat_thread_ = std::thread(&MAVLinkManager::heartbeatChecker, this);
    authentication_thread_ = std::thread(&MAVLinkManager::authenticationWorker, this);
    reconnection_thread_ = std::thread(&MAVLinkManager::reconnectionWorker, this);
#endif
    
    return true;
}

void MAVLinkManager::startConcurrentMode() {
    if (running_.load()) return;
    running_.store(true);
}

void MAVLinkManager::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    
#ifdef USE_BOOST_ASIO
    // 停止Boost.Asio事件循环
    if (work_) {
        work_.reset();
    }
    if (io_context_) {
        io_context_->stop();
    }
    
    // 等待线程池结束
    for (auto& thread : asio_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    asio_threads_.clear();
    
    if (io_context_) {
        io_context_->reset();
    }
    
    std::cout << "MAVLinkManager stopped with Boost.Asio event loop" << std::endl;
#else
    // 兼容模式：等待原有轮询线程结束
    if (connection_thread_.joinable()) {
        connection_thread_.join();
    }
    if (telemetry_thread_.joinable()) {
        telemetry_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    if (authentication_thread_.joinable()) {
        authentication_thread_.join();
    }
    if (reconnection_thread_.joinable()) {
        reconnection_thread_.join();
    }
#endif
    
    disconnectAll();
}


bool MAVLinkManager::isRunning() const {
    return running_.load();
}

bool MAVLinkManager::connectToDrone(const std::string& connection_url) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    // 检查是否已经连接到此URL
    auto it = url_to_drone_id_.find(connection_url);
    if (it != url_to_drone_id_.end()) {
        std::cout << "Already connected to " << connection_url << std::endl;
        return true;
    }
    
    // 创建新的连接
    auto conn = std::make_unique<DroneConnection>();
    conn->drone_id = generateDroneId();
    conn->connection_url = connection_url;
    conn->status = ConnectionStatus::CONNECTING;
    
    DroneId drone_id = conn->drone_id;
    
    // 存储连接信息
    connections_[drone_id] = std::move(conn);
    url_to_drone_id_[connection_url] = drone_id;
    
    std::cout << "Initiating connection to drone " << drone_id 
              << " at " << connection_url << std::endl;
    
    // 通知连接状态变化
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::CONNECTING);
    }
    
    return true;
}

bool MAVLinkManager::startDiscovery(const std::string& listen_url) {
#ifdef HAVE_MAVSDK
    if (!mavsdk_) return false;
    if (discovery_mode_) {
        std::cout << "Discovery already active on " << discovery_listen_url_ << std::endl;
        return true;
    }
    auto result = mavsdk_->add_any_connection(listen_url);
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Failed to start discovery listener: " << listen_url << std::endl;
        return false;
    }
    discovery_listen_url_ = listen_url;
    discovery_mode_ = true;
    std::cout << "LAN discovery started, listening on " << listen_url << std::endl;
    return true;
#else
    (void)listen_url;
    return false;
#endif
}

void MAVLinkManager::disconnectFromDrone(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return;
    }
    
    for (auto url_it = url_to_drone_id_.begin(); url_it != url_to_drone_id_.end(); ++url_it) {
        if (url_it->second == drone_id) { url_to_drone_id_.erase(url_it); break; }
    }
    for (auto it = system_key_to_drone_id_.begin(); it != system_key_to_drone_id_.end(); ++it) {
        if (it->second == drone_id) { system_key_to_drone_id_.erase(it); break; }
    }
    // 更新状态并通知
    it->second->status = ConnectionStatus::DISCONNECTED;
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::DISCONNECTED);
    }
    
    // 移除连接
    connections_.erase(it);
    
    std::cout << "Disconnected from drone " << drone_id << std::endl;
}

void MAVLinkManager::disconnectAll() {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto& [drone_id, conn] : connections_) {
        for (auto& callback : connection_callbacks_) callback(drone_id, ConnectionStatus::DISCONNECTED);
    }
    connections_.clear();
    url_to_drone_id_.clear();
    system_key_to_drone_id_.clear();
    std::cout << "Disconnected from all drones" << std::endl;
}

std::vector<DroneId> MAVLinkManager::getConnectedDrones() const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    std::vector<DroneId> connected_drones;
    for (const auto& [drone_id, conn] : connections_) {
        if (conn->status == ConnectionStatus::CONNECTED || conn->status == ConnectionStatus::AUTHENTICATED) {
            connected_drones.push_back(drone_id);
        }
    }
    
    return connected_drones;
}

MAVLinkManager::ConnectionStatus MAVLinkManager::getConnectionStatus(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return ConnectionStatus::DISCONNECTED;
    }
    
    return it->second->status;
}

bool MAVLinkManager::isDroneConnected(DroneId drone_id) const {
    auto status = getConnectionStatus(drone_id);
    return status == ConnectionStatus::CONNECTED || status == ConnectionStatus::AUTHENTICATED;
}

void MAVLinkManager::registerConnectionCallback(ConnectionCallback callback) {
    connection_callbacks_.push_back(callback);
}

void MAVLinkManager::registerTelemetryCallback(TelemetryCallback callback) {
    telemetry_callbacks_.push_back(callback);
}

void MAVLinkManager::registerAuthenticationCallback(AuthenticationCallback callback) {
    authentication_callbacks_.push_back(callback);
}

std::optional<ExtendedDroneState> MAVLinkManager::getDroneState(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return std::nullopt;
    }
    
    return it->second->state;
}

void MAVLinkManager::setAuthenticationProvider(std::unique_ptr<AuthenticationProvider> provider) {
    auth_provider_ = std::move(provider);
    std::cout << "Authentication provider set: " << (auth_provider_ ? auth_provider_->getProviderType() : "none") << std::endl;
}

bool MAVLinkManager::authenticateDrone(DroneId drone_id, const std::string& credential, const std::string& credential_type) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        std::cout << "Cannot authenticate unknown drone " << drone_id << std::endl;
        return false;
    }
    
    if (it->second->status != ConnectionStatus::CONNECTED) {
        std::cout << "Cannot authenticate disconnected drone " << drone_id << std::endl;
        return false;
    }
    
    // 存储认证凭据
    it->second->authentication_credential = credential;
    it->second->credential_type = credential_type;
    it->second->status = ConnectionStatus::AUTHENTICATING;
    
    std::cout << "Starting authentication for drone " << drone_id << " with " << credential_type << std::endl;
    
    // 通知状态变化
    for (auto& callback : connection_callbacks_) {
        callback(drone_id, ConnectionStatus::AUTHENTICATING);
    }
    
    return true;
}

bool MAVLinkManager::isDroneAuthenticated(DroneId drone_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it == connections_.end()) {
        return false;
    }
    
    return it->second->is_authenticated && it->second->status == ConnectionStatus::AUTHENTICATED;
}

void MAVLinkManager::setStateManager(std::shared_ptr<DroneStateManager> state_manager) {
    state_manager_ = state_manager;
    std::cout << "State manager set" << std::endl;
}

void MAVLinkManager::setAccessControlEngine(std::shared_ptr<AccessControlEngine> engine) {
    access_control_engine_ = engine;
    std::cout << "Access control engine set for MAVLink manager" << std::endl;
}

void MAVLinkManager::runConnectionCycleOnce() {//连接无人机
#ifdef HAVE_MAVSDK
    if (discovery_mode_) {
        runDiscoveryCycleOnce();
    }
#endif
    std::vector<DroneId> connecting_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTING) {
                connecting_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : connecting_drones) {
        std::unique_ptr<DroneConnection>* conn_ptr = nullptr;
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto it = connections_.find(drone_id);
            if (it == connections_.end()) continue;
            conn_ptr = &it->second;
        }
        bool connection_success = establishConnection(**conn_ptr);
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto it = connections_.find(drone_id);
            if (it != connections_.end()) {
                if (connection_success) {
                    it->second->status = ConnectionStatus::CONNECTED;
                    it->second->retry_count = 0;
                    for (auto& callback : connection_callbacks_) {
                        callback(drone_id, ConnectionStatus::CONNECTED);
                    }
                    if (state_manager_) {
                        state_manager_->setDroneOnline(drone_id);
                        state_manager_->updateDroneState(drone_id, it->second->state);
                    }
                    std::cout << "Successfully connected to drone " << drone_id << std::endl;
                } else {
                    it->second->retry_count++;
                    if (it->second->retry_count >= MAX_RETRY_COUNT) {
                        it->second->status = ConnectionStatus::FAILED;
                        for (auto& callback : connection_callbacks_) {
                            callback(drone_id, ConnectionStatus::FAILED);
                        }
                        std::cout << "Failed to connect to drone " << drone_id
                                  << " after " << MAX_RETRY_COUNT << " retries" << std::endl;
                    }
                }
            }
        }
    }
    
    // 在Boost.Asio模式下，重新调度任务
#ifdef USE_BOOST_ASIO
    if (running_.load() && io_context_) {
        boost::asio::post(*io_context_, [this]() {
            // 使用定时器延迟执行，避免占用过多CPU
            auto timer = std::make_shared<boost::asio::steady_timer>(*io_context_, RETRY_INTERVAL);
            timer->async_wait([this, timer](const boost::system::error_code&) {
                if (running_.load()) {
                    runConnectionCycleOnce();
                }
            });
        });
    }
#endif
}

#ifdef HAVE_MAVSDK
static uint64_t systemKey(const std::shared_ptr<mavsdk::System>& sys) {//获取无人机系统ID和组件ID
    uint8_t sysid = sys->get_system_id();
    auto comp_ids = sys->component_ids();
    uint8_t compid = comp_ids.empty() ? 0 : comp_ids[0];
    return (static_cast<uint64_t>(sysid) << 8) | compid;
}

void MAVLinkManager::runDiscoveryCycleOnce() {//发现无人机
    if (!mavsdk_ || !discovery_mode_) return;
    auto systems = mavsdk_->systems();
    std::set<uint64_t> current_keys;
    for (const auto& sys : systems) {
        if (!sys) continue;
        uint64_t key = systemKey(sys);
        current_keys.insert(key);
        std::lock_guard<std::mutex> lock(connections_mutex_);
        if (system_key_to_drone_id_.count(key)) continue;
        DroneId drone_id = generateDroneId();
        auto conn = std::make_unique<DroneConnection>();
        conn->drone_id = drone_id;
        conn->connection_url = discovery_listen_url_;
        conn->status = ConnectionStatus::CONNECTING;
        conn->system = sys;
        if (establishConnectionFromSystem(*conn, sys)) {
            conn->status = ConnectionStatus::CONNECTED;
            conn->last_heartbeat = std::chrono::steady_clock::now();
            system_key_to_drone_id_[key] = drone_id;
            connections_[drone_id] = std::move(conn);
            for (auto& cb : connection_callbacks_) cb(drone_id, ConnectionStatus::CONNECTED);
            if (state_manager_) {
                state_manager_->setDroneOnline(drone_id);
                state_manager_->updateDroneState(drone_id, connections_[drone_id]->state);
            }
            std::cout << "Discovered and connected drone " << drone_id
                      << " (sysid=" << (int)sys->get_system_id() << ")" << std::endl;
        }
    }
    std::vector<DroneId> to_remove;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto it = system_key_to_drone_id_.begin(); it != system_key_to_drone_id_.end(); ) {
            if (current_keys.count(it->first)) { ++it; continue; }
            to_remove.push_back(it->second);
            it = system_key_to_drone_id_.erase(it);
        }
    }
    for (DroneId did : to_remove) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto cit = connections_.find(did);
        if (cit != connections_.end()) {
            if (state_manager_) state_manager_->setDroneOffline(did);
            for (auto& cb : connection_callbacks_) cb(did, ConnectionStatus::DISCONNECTED);
            connections_.erase(cit);
        }
    }
}

bool MAVLinkManager::establishConnectionFromSystem(DroneConnection& conn, const std::shared_ptr<mavsdk::System>& system) {
    if (!system) return false;
    try {
        conn.system = system;
        conn.telemetry = std::make_unique<mavsdk::Telemetry>(system);
        conn.action = std::make_unique<mavsdk::Action>(system);
        conn.mission = std::make_unique<mavsdk::Mission>(system);
        conn.mavlink_passthrough = std::make_unique<mavsdk::MavlinkPassthrough>(system);
        conn.offboard = std::unique_ptr<void, void(*)(void*)>(
            new mavsdk::Offboard(system),
            [](void* p) { delete static_cast<mavsdk::Offboard*>(p); }
        );
        conn.ftp = std::make_unique<mavsdk::Ftp>(system);
        setupMavlinkMessageHandlers(conn);
        conn.state.drone_id = conn.drone_id;
        conn.state.drone_model = "PX4";
        conn.state.owner_organization = "discovered";
        conn.state.last_update = std::chrono::system_clock::now();
        return true;
    } catch (const std::exception& e) {
        std::cout << "establishConnectionFromSystem: " << e.what() << std::endl;
        return false;
    }
}
#endif

void MAVLinkManager::connectionWorker() {
    while (running_.load()) {
        runConnectionCycleOnce();
        std::this_thread::sleep_for(RETRY_INTERVAL);
    }
}

void MAVLinkManager::runTelemetryCycleOnce() {//处理无人机遥测数据
    if (telemetry_cycle_count_ == 0) {
        last_telemetry_status_report_ = std::chrono::steady_clock::now();
        last_telemetry_callback_time_ = std::chrono::steady_clock::now();
    }
    auto cycle_start = std::chrono::high_resolution_clock::now();
    
    // 收集需要处理的无人机连接
    std::vector<std::pair<DroneId, DroneConnection*>> drones_to_process;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (const auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTED || conn->status == ConnectionStatus::AUTHENTICATED) {
                drones_to_process.emplace_back(drone_id, conn.get());
            }
        }
    }
    
    // 异步处理遥测数据，避免阻塞事件循环
    for (const auto& [drone_id, conn] : drones_to_process) {
#ifdef USE_BOOST_ASIO
        if (io_context_) {
            boost::asio::post(*io_context_, [this, drone_id, conn]() {
                processDroneTelemetry(drone_id, conn);
            });
        }
#else
        // 非Boost.Asio模式：直接处理
        processDroneTelemetry(drone_id, conn);
#endif
    }
    
    telemetry_cycle_count_++;
    total_telemetry_callbacks_processed_ += drones_to_process.size();
    
    // 每100次循环打印一次遥测处理状态
    if (telemetry_cycle_count_ % 100 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_telemetry_status_report_).count();
        std::cout << "[遥测] 处理完成，周期=" << telemetry_cycle_count_ 
                  << ", 无人机数=" << drones_to_process.size() 
                  << ", 总回调数=" << total_telemetry_callbacks_processed_ 
                  << ", 耗时=" << elapsed << "ms" << std::endl;
        last_telemetry_status_report_ = now;
    }
    
    // 在Boost.Asio模式下，重新调度任务
#ifdef USE_BOOST_ASIO
    if (running_.load() && io_context_) {
        boost::asio::post(*io_context_, [this]() {
            // 使用定时器延迟执行，避免占用过多CPU
            auto timer = std::make_shared<boost::asio::steady_timer>(*io_context_, std::chrono::milliseconds(100));
            timer->async_wait([this, timer](const boost::system::error_code&) {
                if (running_.load()) {
                    runTelemetryCycleOnce();
                }
            });
        });
    }
#endif
}

void MAVLinkManager::processDroneTelemetry(DroneId drone_id, DroneConnection* conn) {//处理无人机遥测数据
    auto now = std::chrono::system_clock::now();
    conn->state.last_update = now;
    conn->last_heartbeat = std::chrono::steady_clock::now();
    conn->state.drone_id = drone_id;

    bool position_updated = false;
    double new_lat = conn->state.position.latitude;
    double new_lon = conn->state.position.longitude;

#ifdef HAVE_MAVSDK
    if (conn->telemetry) {
        try {
            // 位置数据 - 过滤更新频率
            if (telemetry_cycle_count_ % 5 == 0) {
                auto position = conn->telemetry->position();
                // 空间过滤：只有位置变化超过阈值才更新
                if (std::abs(position.latitude_deg - conn->state.position.latitude) > 0.0001 ||
                    std::abs(position.longitude_deg - conn->state.position.longitude) > 0.0001 ||
                    std::abs(position.relative_altitude_m - conn->state.position.altitude) > 0.5) {
                    new_lat = position.latitude_deg;
                    new_lon = position.longitude_deg;
                    conn->state.position.latitude = position.latitude_deg;
                    conn->state.position.longitude = position.longitude_deg;
                    conn->state.position.altitude = position.relative_altitude_m;
                    position_updated = true;
                    if (telemetry_cycle_count_ % 50 == 0) {
                        std::cout << "📊 无人机" << drone_id << " 位置更新: 纬度=" << position.latitude_deg
                                  << ", 经度=" << position.longitude_deg
                                  << ", 相对高度=" << position.relative_altitude_m << "米" << std::endl;
                    }
                }
            }
            
            // 速度数据 - 过滤更新频率
            if (telemetry_cycle_count_ % 3 == 0) {
                auto velocity_ned = conn->telemetry->velocity_ned();
                // 速度过滤：只有速度变化超过阈值才更新
                if (std::abs(velocity_ned.north_m_s - conn->state.velocity_north) > 0.1 ||
                    std::abs(velocity_ned.east_m_s - conn->state.velocity_east) > 0.1 ||
                    std::abs(velocity_ned.down_m_s - conn->state.velocity_down) > 0.1) {
                    conn->state.velocity_north = velocity_ned.north_m_s;
                    conn->state.velocity_east = velocity_ned.east_m_s;
                    conn->state.velocity_down = velocity_ned.down_m_s;
                    conn->state.speed = std::sqrt(
                        velocity_ned.north_m_s * velocity_ned.north_m_s +
                        velocity_ned.east_m_s * velocity_ned.east_m_s +
                        velocity_ned.down_m_s * velocity_ned.down_m_s);
                    conn->state.ground_speed = std::sqrt(
                        velocity_ned.north_m_s * velocity_ned.north_m_s +
                        velocity_ned.east_m_s * velocity_ned.east_m_s);
                }
            }
            
            // 电池数据 - 过滤更新频率
            if (telemetry_cycle_count_ % 10 == 0) {
                auto battery = conn->telemetry->battery();
                // 电池过滤：只有电量变化超过阈值才更新
                if (std::abs(battery.remaining_percent * 100.0 - conn->state.battery_percentage) > 1.0) {
                    conn->state.battery_percentage = battery.remaining_percent * 100.0;
                }
            }
            
            // 飞行模式 - 过滤更新频率
            if (telemetry_cycle_count_ % 5 == 0) {
                auto flight_mode = conn->telemetry->flight_mode();
                FlightStatus new_status = FlightStatus::UNKNOWN;
                switch (flight_mode) {
                    case mavsdk::Telemetry::FlightMode::Ready:
                        new_status = FlightStatus::LANDED;
                        break;
                    case mavsdk::Telemetry::FlightMode::Takeoff:
                        new_status = FlightStatus::TAKING_OFF;
                        break;
                    case mavsdk::Telemetry::FlightMode::Hold:
                    case mavsdk::Telemetry::FlightMode::Mission:
                    case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                    case mavsdk::Telemetry::FlightMode::Offboard:
                    case mavsdk::Telemetry::FlightMode::Manual:
                    case mavsdk::Telemetry::FlightMode::Altctl:
                    case mavsdk::Telemetry::FlightMode::Posctl:
                        new_status = FlightStatus::IN_AIR;
                        break;
                    case mavsdk::Telemetry::FlightMode::Land:
                        new_status = FlightStatus::LANDING;
                        break;
                    default:
                        new_status = FlightStatus::UNKNOWN;
                        break;
                }
                // 飞行状态过滤：只有状态变化才更新
                if (new_status != conn->state.flight_status) {
                    conn->state.flight_status = new_status;
                }
            }
            
            // 武装状态 - 过滤更新频率
            if (telemetry_cycle_count_ % 2 == 0) {
                bool is_armed = conn->telemetry->armed();
                // 武装状态过滤：只有状态变化才更新
                if (is_armed != conn->state.is_armed) {
                    conn->state.is_armed = is_armed;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[遥测] 处理无人机 " << drone_id << " 遥测数据时异常: " << e.what() << std::endl;
        }
    }
#endif
    
    // 更新无人机位置，用于动态连接管理
    if (position_updated) {
        updateDronePosition(drone_id, new_lat, new_lon);
    }
    
    // 通知状态管理器和回调
    if (state_manager_) {
        state_manager_->updateDroneState(drone_id, conn->state);
    }
    for (auto& callback : telemetry_callbacks_) {
        callback(conn->state);
    }
}



void MAVLinkManager::telemetryWorker() {//处理无人机遥测数据
    std::cout << "🔄 遥测数据工作线程已启动，更新间隔: " << TELEMETRY_INTERVAL.count() << "ms" << std::endl;
    while (running_.load()) {
        runTelemetryCycleOnce();
        std::this_thread::sleep_for(TELEMETRY_INTERVAL);
    }
    std::cout << "🛑 遥测数据工作线程已停止，总处理周期: " << telemetry_cycle_count_
              << ", 总回调数: " << total_telemetry_callbacks_processed_ << std::endl;
}

void MAVLinkManager::runHeartbeatCycleOnce() {
    auto now = std::chrono::steady_clock::now();
    std::vector<DroneId> lost_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::AUTHENTICATED) {
                auto elapsed = now - conn->last_heartbeat;
                if (elapsed > HEARTBEAT_TIMEOUT) {
                    lost_drones.push_back(drone_id);
                }
            }
        }
    }
    for (DroneId drone_id : lost_drones) {
        handleConnectionLost(drone_id);
    }
    
    // 在Boost.Asio模式下，重新调度任务
#ifdef USE_BOOST_ASIO
    if (running_.load() && io_context_) {
        boost::asio::post(*io_context_, [this]() {
            // 使用定时器延迟执行，避免占用过多CPU
            auto timer = std::make_shared<boost::asio::steady_timer>(*io_context_, std::chrono::seconds(5));
            timer->async_wait([this, timer](const boost::system::error_code&) {
                if (running_.load()) {
                    runHeartbeatCycleOnce();
                }
            });
        });
    }
#endif
}

void MAVLinkManager::heartbeatChecker() {
    while (running_.load()) {
        runHeartbeatCycleOnce();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void MAVLinkManager::runAuthenticationCycleOnce() {
    std::vector<DroneId> authenticating_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::AUTHENTICATING) {
                authenticating_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : authenticating_drones) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(drone_id);
        if (it != connections_.end()) {
            if (performAuthentication(*it->second)) {
                it->second->status = ConnectionStatus::AUTHENTICATED;
                it->second->is_authenticated = true;
                it->second->state.is_authenticated = true;
                it->second->state.authentication_type = it->second->credential_type.empty() ? "unknown" : it->second->credential_type;
                auto attr_org = it->second->extracted_attributes.find("organization");
                if (attr_org != it->second->extracted_attributes.end()) {
                    it->second->state.owner_organization = attr_org->second;
                }
                auto attr_cert = it->second->extracted_attributes.find("certificate_id");
                if (attr_cert != it->second->extracted_attributes.end()) {
                    it->second->state.certificate_id = attr_cert->second;
                }
                auto attr_trust = it->second->extracted_attributes.find("trust_level");
                if (attr_trust != it->second->extracted_attributes.end()) {
                    it->second->state.trust_level = attr_trust->second;
                }
                it->second->state.auth_timestamp = std::chrono::system_clock::now();
                it->second->state.auth_attributes = it->second->extracted_attributes;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::AUTHENTICATED);
                }
                if (state_manager_) {
                    state_manager_->updateDroneState(drone_id, it->second->state);
                }
                handleAuthenticationResult(drone_id, true, "Authentication successful");
                std::cout << "Successfully authenticated drone " << drone_id << std::endl;
            } else {
                it->second->status = ConnectionStatus::AUTHENTICATION_FAILED;
                it->second->is_authenticated = false;
                it->second->state.is_authenticated = false;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::AUTHENTICATION_FAILED);
                }
                handleAuthenticationResult(drone_id, false, "Authentication failed");
                std::cout << "Authentication failed for drone " << drone_id << std::endl;
            }
        }
    }
    
    // 在Boost.Asio模式下，重新调度任务
#ifdef USE_BOOST_ASIO
    if (running_.load() && io_context_) {
        boost::asio::post(*io_context_, [this]() {
            // 使用定时器延迟执行，避免占用过多CPU
            auto timer = std::make_shared<boost::asio::steady_timer>(*io_context_, std::chrono::seconds(1));
            timer->async_wait([this, timer](const boost::system::error_code&) {
                if (running_.load()) {
                    runAuthenticationCycleOnce();
                }
            });
        });
    }
#endif
}

void MAVLinkManager::authenticationWorker() {
    while (running_.load()) {
        runAuthenticationCycleOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MAVLinkManager::runReconnectionCycleOnce() {
    std::vector<DroneId> reconnecting_drones;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (auto& [drone_id, conn] : connections_) {
            if (conn->status == ConnectionStatus::CONNECTING && conn->retry_count <= MAX_RETRY_COUNT) {
                reconnecting_drones.push_back(drone_id);
            }
        }
    }
    for (DroneId drone_id : reconnecting_drones) {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(drone_id);
        if (it != connections_.end()) {
            std::cout << "🔄 正在重连无人机 " << drone_id << "..." << std::endl;
            if (establishConnection(*it->second)) {
                it->second->status = ConnectionStatus::CONNECTED;
                it->second->retry_count = 0;
                for (auto& callback : connection_callbacks_) {
                    callback(drone_id, ConnectionStatus::CONNECTED);
                }
                if (state_manager_) {
                    state_manager_->setDroneOnline(drone_id);
                    state_manager_->updateDroneState(drone_id, it->second->state);
                }
                std::cout << "无人机 " << drone_id << " 重连成功" << std::endl;
                
                // 重连成功后，重新分配边缘设备
                std::string best_edge_device = getBestEdgeDeviceForDrone(drone_id);
                if (!best_edge_device.empty()) {
                    assignDroneToEdgeDevice(drone_id, best_edge_device);
                }
            } else {
                std::cout << "[错误] 无人机 " << drone_id << " 重连失败，将在 " << RECONNECT_INTERVAL.count() << " 秒后重试" << std::endl;
            }
        }
    }
    
    // 在Boost.Asio模式下，重新调度任务
#ifdef USE_BOOST_ASIO
    if (running_.load() && io_context_) {
        boost::asio::post(*io_context_, [this]() {
            // 使用定时器延迟执行，避免占用过多CPU
            auto timer = std::make_shared<boost::asio::steady_timer>(*io_context_, RECONNECT_INTERVAL);
            timer->async_wait([this, timer](const boost::system::error_code&) {
                if (running_.load()) {
                    runReconnectionCycleOnce();
                }
            });
        });
    }
#endif
}

void MAVLinkManager::reconnectionWorker() {
    while (running_.load()) {
        runReconnectionCycleOnce();
        std::this_thread::sleep_for(RECONNECT_INTERVAL);
    }
}

bool MAVLinkManager::establishConnection(DroneConnection& conn) {
#ifdef HAVE_MAVSDK
    if (!mavsdk_) {
        return false;
    }
    
    try {
        // 添加连接
        auto connection_result = mavsdk_->add_any_connection(conn.connection_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            std::cout << "Failed to add connection: " << connection_result << std::endl;
            return false;
        }
        
        // 等待系统发现，最多等待10秒
        auto systems = mavsdk_->systems();
        int wait_count = 0;
        while (systems.empty() && wait_count < 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            systems = mavsdk_->systems();
            wait_count++;
        }
        
        if (systems.empty()) {
            std::cout << "No systems found after waiting" << std::endl;
            return false;
        }
        
        // 使用第一个发现的系统
        conn.system = systems[0];
        
        // 创建插件实例
        conn.telemetry = std::make_unique<mavsdk::Telemetry>(conn.system);
        conn.action = std::make_unique<mavsdk::Action>(conn.system);
        conn.mission = std::make_unique<mavsdk::Mission>(conn.system);
        conn.mavlink_passthrough = std::make_unique<mavsdk::MavlinkPassthrough>(conn.system);
        conn.offboard = std::unique_ptr<void, void(*)(void*)>(
            new mavsdk::Offboard(conn.system),
            [](void* ptr) { delete static_cast<mavsdk::Offboard*>(ptr); }
        );
        
        // 初始化FTP插件
        conn.ftp = std::make_unique<mavsdk::Ftp>(conn.system);
        
        // 设置MAVLink消息处理器
        setupMavlinkMessageHandlers(conn);
        
        // 完全避免MAVSDK回调机制，改用轮询方式获取所有数据
        // 这样可以完全避免回调队列溢出问题
        std::cout << "[警告]  使用轮询模式获取遥测数据，避免回调队列溢出" << std::endl;
        
        // 其他数据通过轮询获取，避免回调队列积压
        // 这些数据对访问控制不是关键的，可以降低更新频率
        
        // 初始化无人机状态
        conn.state.drone_id = conn.drone_id;
        conn.state.drone_model = "PX4_SITL";
        conn.state.owner_organization = "px4_simulation";
        conn.state.last_update = std::chrono::system_clock::now();
        
        std::cout << "Successfully established MAVLink connection to PX4 SITL" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cout << "Exception in establishConnection: " << e.what() << std::endl;
        return false;
    }
#else
    // 模拟连接建立
    std::cout << "Simulating connection to " << conn.connection_url << std::endl;
    conn.last_heartbeat = std::chrono::steady_clock::now();
    
    // 设置模拟状态
    conn.state.drone_id = conn.drone_id;
    conn.state.position = Position(39.9042, 116.4074, 50.0);  // 示例位置
    conn.state.flight_status = FlightStatus::LANDED;
    conn.state.battery_percentage = 95.0;
    conn.state.is_armed = false;
    conn.state.drone_model = "PX4_SITL";
    conn.state.owner_organization = "test_org";
    
    return true;
#endif
}

void MAVLinkManager::handleConnectionLost(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it != connections_.end()) {
        // 保存连接信息用于重连
        std::string connection_url = it->second->connection_url;
        
        it->second->status = ConnectionStatus::DISCONNECTED;
        it->second->is_authenticated = false;
        
        // 更新状态管理器
        if (state_manager_) {
            state_manager_->setDroneOffline(drone_id);
        }
        
        // 通知连接丢失
        for (auto& callback : connection_callbacks_) {
            callback(drone_id, ConnectionStatus::DISCONNECTED);
        }
        
        std::cout << "Connection lost to drone " << drone_id << std::endl;
        
        // 启动重连逻辑 - 将状态设置为CONNECTING以便重连
        if (it->second->retry_count < MAX_RETRY_COUNT) {
            it->second->status = ConnectionStatus::CONNECTING;
            it->second->retry_count++;
            std::cout << "🔄 尝试重连无人机 " << drone_id << " (第 " << it->second->retry_count << " 次)" << std::endl;
        } else {
            std::cout << "[错误] 无人机 " << drone_id << " 重连次数已达上限，停止重连" << std::endl;
        }
    }
}

void MAVLinkManager::updateDroneState(DroneId drone_id, const ExtendedDroneState& state) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    auto it = connections_.find(drone_id);
    if (it != connections_.end()) {
        it->second->state = state;
        it->second->last_heartbeat = std::chrono::steady_clock::now();
    }
}

bool MAVLinkManager::performAuthentication(DroneConnection& conn) {
    if (!auth_provider_) {
        std::cout << "No authentication provider configured, allowing connection" << std::endl;
        return true;  // 如果没有认证提供者，允许连接
    }
    
    if (conn.authentication_credential.empty()) {
        std::cout << "No authentication credential provided for drone " << conn.drone_id << std::endl;
        return false;
    }
    
    try {
        // 创建认证请求
        AuthenticationRequest request;
        request.credential = conn.authentication_credential;
        request.credential_type = conn.credential_type;
        request.drone_id = std::to_string(conn.drone_id);
        
        // 执行认证
        AuthenticationResult result = auth_provider_->authenticate(request);
        
        if (result.success) {
            // 存储提取的属性
            conn.extracted_attributes = result.attributes;
            
            // 更新无人机状态中的属性信息
            if (!result.attributes.empty()) {
                for (const auto& [key, value] : result.attributes) {
                    if (key == "organization") {
                        conn.state.owner_organization = value;
                    } else if (key == "role") {
                        // 可以根据角色设置特权状态
                        if (value == "government" || value == "police" || value == "emergency") {
                            conn.state.is_privileged = true;
                        }
                    }
                }
            }
            // 同步认证结果到状态
            conn.state.is_authenticated = true;
            conn.state.authentication_type = conn.credential_type.empty() ? "unknown" : conn.credential_type;
            // 优先使用返回字段的 trust_level
            if (!result.trust_level.empty()) {
                conn.state.trust_level = result.trust_level;
            } else {
                auto tl = conn.extracted_attributes.find("trust_level");
                if (tl != conn.extracted_attributes.end()) {
                    conn.state.trust_level = tl->second;
                }
            }
            auto cert_it = conn.extracted_attributes.find("certificate_id");
            if (cert_it != conn.extracted_attributes.end()) {
                conn.state.certificate_id = cert_it->second;
            }
            conn.state.auth_timestamp = std::chrono::system_clock::now();
            conn.state.auth_attributes = conn.extracted_attributes;
            
            // 即时写入状态管理器，便于前端展示
            if (state_manager_) {
                state_manager_->updateDroneState(conn.drone_id, conn.state);
            }
            
            std::cout << "Authentication successful for drone " << conn.drone_id 
                      << " (trust level: " << result.trust_level << ")" << std::endl;
            return true;
        } else {
            std::cout << "Authentication failed for drone " << conn.drone_id 
                      << ": " << result.error_message << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cout << "Exception during authentication for drone " << conn.drone_id 
                  << ": " << e.what() << std::endl;
        return false;
    }
}

void MAVLinkManager::handleAuthenticationResult(DroneId drone_id, bool success, const std::string& message) {
    // 通知认证回调
    for (auto& callback : authentication_callbacks_) {
        try {
            callback(drone_id, success, message);
        } catch (const std::exception& e) {
            std::cout << "Exception in authentication callback: " << e.what() << std::endl;
        }
    }
    
    // 如果认证失败，可以选择断开连接
    if (!success) {
        std::cout << "Considering disconnection for unauthenticated drone " << drone_id << std::endl;
        // 这里可以根据策略决定是否断开连接
        // disconnectFromDrone(drone_id);
    }
}

DroneId MAVLinkManager::generateDroneId() {
    return next_drone_id_++;
}

// 边缘设备管理方法
void MAVLinkManager::registerEdgeDevice(const std::string& device_id, const std::string& device_name, const std::string& location) {
    std::lock_guard<std::mutex> lock(edge_devices_mutex_);
    EdgeDeviceInfo device_info;
    device_info.device_id = device_id;
    device_info.device_name = device_name;
    device_info.location = location;
    device_info.online = true;
    device_info.last_heartbeat = std::chrono::system_clock::now();
    edge_devices_[device_id] = device_info;
    std::cout << "边缘设备注册成功: " << device_name << " (" << device_id << ")" << std::endl;
}

void MAVLinkManager::unregisterEdgeDevice(const std::string& device_id) {
    std::lock_guard<std::mutex> lock(edge_devices_mutex_);
    auto it = edge_devices_.find(device_id);
    if (it != edge_devices_.end()) {
        std::cout << "边缘设备注销: " << it->second.device_name << " (" << it->second.device_id << ")" << std::endl;
        edge_devices_.erase(it);
    }
}

void MAVLinkManager::updateEdgeDeviceStatus(const std::string& device_id, bool online) {
    std::lock_guard<std::mutex> lock(edge_devices_mutex_);
    auto it = edge_devices_.find(device_id);
    if (it != edge_devices_.end()) {
        it->second.online = online;
        it->second.last_heartbeat = std::chrono::system_clock::now();
        std::cout << "边缘设备状态更新: " << it->second.device_name << " (" << it->second.device_id << ") 在线状态: " << (online ? "在线" : "离线") << std::endl;
        
        // 如果边缘设备离线，将其管理的无人机重新分配到其他在线边缘设备
        if (!online) {
            auto drone_it = edge_device_to_drones_.find(device_id);
            if (drone_it != edge_device_to_drones_.end()) {
                for (DroneId drone_id : drone_it->second) {
                    std::string best_edge_device = getBestEdgeDeviceForDrone(drone_id);
                    if (!best_edge_device.empty()) {
                        switchDroneEdgeDevice(drone_id, best_edge_device);
                    }
                }
            }
        }
    }
}

// 边缘设备辅助方法
void MAVLinkManager::updateDronePosition(DroneId drone_id, double lat, double lon) {
    // 这里可以实现无人机位置更新逻辑
    // 例如，可以将位置信息存储到状态中
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(drone_id);
    if (it != connections_.end()) {
        it->second->state.position.latitude = lat;
        it->second->state.position.longitude = lon;
    }
}

std::string MAVLinkManager::getBestEdgeDeviceForDrone(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(edge_devices_mutex_);
    
    // 简单实现：返回第一个在线的边缘设备
    for (const auto& [device_id, device_info] : edge_devices_) {
        if (device_info.online) {
            return device_id;
        }
    }
    return "";
}

void MAVLinkManager::assignDroneToEdgeDevice(DroneId drone_id, const std::string& edge_device_id) {
    std::lock_guard<std::mutex> lock(edge_devices_mutex_);
    
    // 先从旧的边缘设备中移除
    auto old_it = drone_to_edge_device_.find(drone_id);
    if (old_it != drone_to_edge_device_.end()) {
        auto& old_drones = edge_device_to_drones_[old_it->second];
        old_drones.erase(std::remove(old_drones.begin(), old_drones.end(), drone_id), old_drones.end());
    }
    
    // 分配到新的边缘设备
    drone_to_edge_device_[drone_id] = edge_device_id;
    edge_device_to_drones_[edge_device_id].push_back(drone_id);
    
    std::cout << "无人机 " << drone_id << " 分配到边缘设备 " << edge_device_id << std::endl;
}

void MAVLinkManager::switchDroneEdgeDevice(DroneId drone_id, const std::string& new_edge_device_id) {
    assignDroneToEdgeDevice(drone_id, new_edge_device_id);
}

} // namespace drone_control