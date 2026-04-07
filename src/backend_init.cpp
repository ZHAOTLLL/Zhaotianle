/**
 * 后端统一初始化与关闭
 * 供 C++ gRPC 服务端使用：加载配置、建立与无人机的 MAVLink 直连、访问控制、飞行控制及各 API 模块。
 * 无人机与 C++ 的通信仅在此通过 MAVLink 直连，不经过 Go。
 */
#include "backend_init.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "api/drone_api.hpp"
#include "api/flight_api.hpp"
#include "api/access_api.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/abac_policy_engine.hpp"
#include "access_control/policy_config_loader.hpp"
#include "communication/mavlink_manager.hpp"
#include "state/drone_state_manager.hpp"
#include "flight_control/flight_control_module.hpp"
#include "authentication/authentication_provider_factory.hpp"
#include "signature/geofence_signature_service.hpp"
#include "signature/mission_signature_verifier.hpp"
#include "database/database_manager.hpp"
#include "resources/airspace_resource_manager.hpp"

#ifdef USE_CONCURRENT_MAVLINK
#include "communication/mavlink_manager_concurrent_runner.hpp"
#include <boost/asio.hpp>
#endif

namespace backend {

using namespace drone_control;

static std::string s_mavlink_connection_url = "udp://0.0.0.0:14550";
static bool s_core_direct_mavlink_enabled = false;
static std::unique_ptr<MAVLinkManager> s_mavlink_mgr;
static std::shared_ptr<DroneStateManager> s_state_mgr;
static std::shared_ptr<DatabaseManager> s_database_mgr;
static std::shared_ptr<resources::AirspaceResourceManager> s_airspace_mgr;
static std::unique_ptr<AccessControlEngine> s_access_engine;
static std::unique_ptr<FlightControlModule> s_flight_control;
static std::unique_ptr<api::DroneAPI> s_drone_api;
static std::unique_ptr<api::FlightAPI> s_flight_api;
static std::unique_ptr<api::AccessAPI> s_access_api;

#ifdef USE_CONCURRENT_MAVLINK
static std::unique_ptr<boost::asio::io_context> s_io_context;
static std::unique_ptr<MAVLinkManagerConcurrentRunner> s_runner;
static std::thread s_io_thread;
#endif

static bool loadConfig(const std::string& config_file) {
  try {
    if (!std::ifstream(config_file).good()) {
      std::cerr << "配置文件未找到: " << config_file << std::endl;
      return false;
    }
    YAML::Node config = YAML::LoadFile(config_file);
    std::cout << "Configuration loaded from: " << config_file << std::endl;
    try {
      if (config["network"] && config["network"]["mavlink"] && config["network"]["mavlink"]["connection_url"]) {
        s_mavlink_connection_url = config["network"]["mavlink"]["connection_url"].as<std::string>();
        std::cout << "MAVLink connection_url from config: " << s_mavlink_connection_url << std::endl;
      }
      if (config["features"] && config["features"]["core_direct_mavlink_enabled"]) {
        s_core_direct_mavlink_enabled = config["features"]["core_direct_mavlink_enabled"].as<bool>();
      }
      std::cout << "Core direct MAVLink mode: "
                << (s_core_direct_mavlink_enabled ? "ENABLED" : "DISABLED(edge bridge mode)") << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Failed to parse MAVLink connection_url: " << e.what() << std::endl;
    }
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    return false;
  }
}

static void setupMavLink() {
  s_mavlink_mgr = std::make_unique<MAVLinkManager>();
  s_state_mgr = std::make_shared<DroneStateManager>();
  s_database_mgr = std::make_shared<DatabaseManager>();
  (void)s_database_mgr->initialize("", 0, "drone_control", "postgres", "postgres",
                                   "localhost", 6379, "");
  s_state_mgr->setDatabaseManager(s_database_mgr);

  s_airspace_mgr = std::make_shared<resources::AirspaceResourceManager>();
  const std::vector<std::string> airspace_paths = {
      "config/airspace_config.yaml",
      "./config/airspace_config.yaml",
      "../config/airspace_config.yaml",
      "../../config/airspace_config.yaml",
      "build/config/airspace_config.yaml",
      "../build/config/airspace_config.yaml"};
  for (const auto& p : airspace_paths) {
    if (s_airspace_mgr->loadLocationsFromConfig(p)) {
      s_state_mgr->setAirspaceResourceManager(s_airspace_mgr);
      std::cout << "Airspace config loaded for privacy DB sync: " << p << std::endl;
      break;
    }
  }

  s_mavlink_mgr->setStateManager(s_state_mgr);
  if (!s_core_direct_mavlink_enabled) {
    std::cout << "[核心服务][连接] 核心直连无人机已禁用，当前由edge负责发现与MAVLink连接" << std::endl;
    return;
  }
#ifdef USE_CONCURRENT_MAVLINK
  s_mavlink_mgr->startConcurrentMode();
  s_io_context = std::make_unique<boost::asio::io_context>();
  s_runner = std::make_unique<MAVLinkManagerConcurrentRunner>(*s_io_context, 4);
  s_runner->addPeriodicTask(std::chrono::seconds(5), []() { if (s_mavlink_mgr) s_mavlink_mgr->runConnectionCycleOnce(); });
  s_runner->addPeriodicTask(std::chrono::milliseconds(500), []() { if (s_mavlink_mgr) s_mavlink_mgr->runTelemetryCycleOnce(); });
  s_runner->addPeriodicTask(std::chrono::seconds(5), []() { if (s_mavlink_mgr) s_mavlink_mgr->runHeartbeatCycleOnce(); });
  s_runner->addPeriodicTask(std::chrono::seconds(1), []() { if (s_mavlink_mgr) s_mavlink_mgr->runAuthenticationCycleOnce(); });
  s_runner->addPeriodicTask(std::chrono::seconds(10), []() { if (s_mavlink_mgr) s_mavlink_mgr->runReconnectionCycleOnce(); });
  s_runner->start();
  s_io_thread = std::thread([]() { s_io_context->run(); });
  std::cout << "MAVLink 并发运行器已启动" << std::endl;
#else
  s_mavlink_mgr->start();
#endif
  if (!s_mavlink_mgr->startDiscovery(s_mavlink_connection_url)) {
    std::cerr << "Failed to start LAN discovery on: " << s_mavlink_connection_url << std::endl;
  } else {
    std::cout << "MAVLink 局域网发现已开启，监听: " << s_mavlink_connection_url << std::endl;
  }
}

static void setupAccessControl() {
  s_access_engine = std::make_unique<AccessControlEngine>();
  try {
    auto policy_engine = std::make_unique<access_control::ABACPolicyEngine>();
    const std::string policy_config = "../config/access_control_policies.yaml";
    std::cout << " 正在加载策略文件: " << policy_config << std::endl;
    if (policy_engine->loadPoliciesFromFile(policy_config)) {
      s_access_engine->setAccessControlPolicy(std::move(policy_engine));
      std::cout << " 访问控制策略已加载" << std::endl;
    } else {
      std::cerr << " 策略文件加载失败" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "  无法加载访问控制策略: " << e.what() << std::endl;
  }
  auto& auth_factory = AuthenticationProviderFactory::getInstance();
  auto auth_provider = auth_factory.createProvider("x509_certificate");
  if (auth_provider) {
    s_access_engine->setAuthenticationProvider(std::move(auth_provider));
    std::cout << " X.509证书认证提供者已设置" << std::endl;
  }
  if (s_mavlink_mgr) s_access_engine->setMAVLinkManager(std::shared_ptr<MAVLinkManager>(s_mavlink_mgr.get(), [](MAVLinkManager*){}));
  if (s_state_mgr) s_access_engine->setStateManager(s_state_mgr);
}

bool init(const std::string& config_file) {
  if (!loadConfig(config_file)) {
    std::cerr << "Backend init: load config failed, using defaults" << std::endl;
  }
  setupMavLink();
  setupAccessControl();

  std::cout << " 正在初始化地理围栏签名服务..." << std::endl;
  auto geofence_service = std::make_unique<GeofenceSignatureService>();
  std::vector<std::string> possible_paths = {
    "config/keys/geofence_private.pem",
    "../config/keys/geofence_private.pem",
    "../../config/keys/geofence_private.pem",
    "./config/keys/geofence_private.pem"
  };
  std::string private_key_file, public_key_file;
  bool found_keys = false;
  for (const auto& priv_path : possible_paths) {
    std::string pub_path = priv_path;
    size_t pos = pub_path.find("private");
    if (pos != std::string::npos) pub_path.replace(pos, 7, "public");
    std::ifstream priv_check(priv_path);
    std::ifstream pub_check(pub_path);
    if (priv_check.good() && pub_check.good()) {
      private_key_file = priv_path;
      public_key_file = pub_path;
      found_keys = true;
      break;
    }
  }
  if (found_keys && geofence_service->initialize(private_key_file, public_key_file)) {
    std::vector<std::string> sig_paths = { "geofence_signed.json", "../build/geofence_signed.json", "../../build/geofence_signed.json" };
    for (const auto& sig_path : sig_paths) {
      std::ifstream f(sig_path);
      if (f.good() && geofence_service->loadDSSSignedGeofence(sig_path)) {
        std::cout << " 地理围栏签名已加载: " << sig_path << std::endl;
        break;
      }
    }
    s_access_engine->setGeofenceSignatureService(std::move(geofence_service));
  }

  std::cout << " 正在初始化任务签名验证器（DSS）..." << std::endl;
  auto mission_verifier = std::make_unique<MissionSignatureVerifier>();
  if (mission_verifier->initialize()) {
    s_access_engine->setMissionSignatureVerifier(std::move(mission_verifier));
    std::cout << " 任务签名验证器已设置" << std::endl;
  }
  if (s_mavlink_mgr) {
    s_mavlink_mgr->setAccessControlEngine(std::shared_ptr<AccessControlEngine>(s_access_engine.get(), [](AccessControlEngine*){}));
  }

  s_flight_control = std::make_unique<FlightControlModule>();
  if (s_flight_control->initialize(std::shared_ptr<MAVLinkManager>(s_mavlink_mgr.get(), [](MAVLinkManager*){}))) {
    std::cout << "Flight Control Module initialized" << std::endl;
  }

  s_drone_api = std::make_unique<api::DroneAPI>(
    std::shared_ptr<MAVLinkManager>(s_mavlink_mgr.get(), [](MAVLinkManager*){}),
    s_state_mgr
  );
  s_flight_api = std::make_unique<api::FlightAPI>(
    std::shared_ptr<MAVLinkManager>(s_mavlink_mgr.get(), [](MAVLinkManager*){}),
    std::shared_ptr<FlightControlModule>(s_flight_control.get(), [](FlightControlModule*){})
  );
  s_access_api = std::make_unique<api::AccessAPI>(
    std::shared_ptr<AccessControlEngine>(s_access_engine.get(), [](AccessControlEngine*){})
  );
  std::cout << "Core API modules initialized (drone, flight, access)" << std::endl;
  return true;
}

void shutdown() {
  s_access_api.reset();
  s_flight_api.reset();
  s_drone_api.reset();
  s_flight_control.reset();
  if (s_mavlink_mgr) s_mavlink_mgr->setAccessControlEngine(nullptr);
#ifdef USE_CONCURRENT_MAVLINK
  if (s_runner) { s_runner->stop(); s_runner.reset(); }
  if (s_io_context) s_io_context->stop();
  if (s_io_thread.joinable()) s_io_thread.join();
  s_io_context.reset();
#endif
  s_mavlink_mgr.reset();
  s_state_mgr.reset();
  s_airspace_mgr.reset();
  s_database_mgr.reset();
  s_access_engine.reset();
  std::cout << "Backend shutdown done" << std::endl;
}

std::shared_ptr<MAVLinkManager> getMavLinkManager() {
  return s_mavlink_mgr ? std::shared_ptr<MAVLinkManager>(s_mavlink_mgr.get(), [](MAVLinkManager*){}) : nullptr;
}
std::shared_ptr<DroneStateManager> getStateManager() { return s_state_mgr; }
std::shared_ptr<FlightControlModule> getFlightControl() {
  return s_flight_control ? std::shared_ptr<FlightControlModule>(s_flight_control.get(), [](FlightControlModule*){}) : nullptr;
}
AccessControlEngine* getAccessControlEngine() { return s_access_engine.get(); }
api::DroneAPI* getDroneAPI() { return s_drone_api.get(); }
api::FlightAPI* getFlightAPI() { return s_flight_api.get(); }
api::AccessAPI* getAccessAPI() { return s_access_api.get(); }

bool confirmMission(uint32_t drone_id) {
  auto m = getMavLinkManager();
  return m && m->confirmMissionAndRequestCertificate(drone_id);
}

}  // namespace backend
