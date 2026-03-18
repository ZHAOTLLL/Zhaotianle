/**
 * 无人机访问控制后端主程序
 * 启动 HTTP 服务与 MAVLink 连接，加载访问控制策略，注册健康检查、无人机、飞行控制、访问评估、展示等 API 路由。
 */
#include <iostream>
#include <memory>
#include <signal.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <fstream>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "src/api/http_server.hpp"
#include "src/api/status_api.hpp"
#include "src/api/drone_api.hpp"
#include "src/api/flight_api.hpp"
#include "src/api/access_api.hpp"
#include "src/api/access_control_display_api.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/abac_policy_engine.hpp"
#include "access_control/policy_config_loader.hpp"
#include "common/types.hpp"
#include "communication/mavlink_manager.hpp"
#ifdef USE_BOOST_ASIO
#include "communication/mavlink_manager_concurrent_runner.hpp"
#include <boost/asio.hpp>
#endif
#include "state/drone_state_manager.hpp"
#include "flight_control/flight_control_module.hpp"
#include "authentication/authentication_provider_factory.hpp"
#include "signature/geofence_signature_service.hpp"
#include "signature/mission_signature_verifier.hpp"

using namespace drone_control;

// ---------------------------------------------------------------------------
// 进程内全局组件（HTTP 服务、访问控制引擎、MAVLink、状态、飞行控制、各 API）
// ---------------------------------------------------------------------------
static bool should_exit = false;
static std::atomic<bool> shutdown_in_progress{false};
static std::unique_ptr<HttpServer> http_server;
static std::unique_ptr<AccessControlEngine> access_engine;

static std::unique_ptr<drone_control::MAVLinkManager> g_mavlink_mgr;
static std::string g_mavlink_connection_url = "udp://0.0.0.0:14550";

#ifdef USE_CONCURRENT_MAVLINK
static std::unique_ptr<boost::asio::io_context> g_mavlink_io_context;
static std::unique_ptr<drone_control::MAVLinkManagerConcurrentRunner> g_mavlink_runner;
static std::thread g_mavlink_io_thread;
#endif

static std::shared_ptr<drone_control::DroneStateManager> g_state_mgr;
static std::unique_ptr<drone_control::FlightControlModule> g_flight_control;

static std::unique_ptr<api::StatusAPI> g_status_api;
static std::unique_ptr<api::DroneAPI> g_drone_api;
static std::unique_ptr<api::FlightAPI> g_flight_api;
static std::unique_ptr<api::AccessAPI> g_access_api;
static std::unique_ptr<api::AccessControlDisplayAPI> g_display_api;

/** 信号处理：只接受一次关闭，依次停 HTTP、MAVLink 并发、MAVLink、飞行控制 */
void signal_handler(int signal) {
    if (shutdown_in_progress.exchange(true)) {
        std::cout << "\n 系统正在关闭中，请稍候..." << std::endl;
        return;
    }
    std::cout << "\n 收到信号 " << signal << "，正在关闭系统..." << std::endl;
    should_exit = true;

    if (http_server) {
        std::cout << " 正在停止HTTP服务器..." << std::endl;
        http_server->stop();
    }

#ifdef USE_CONCURRENT_MAVLINK
    if (g_mavlink_runner) {
        g_mavlink_runner->stop();
        g_mavlink_runner.reset();
    }
    if (g_mavlink_io_context) {
        g_mavlink_io_context->stop();
    }
    if (g_mavlink_io_thread.joinable()) {
        g_mavlink_io_thread.join();
    }
    g_mavlink_io_context.reset();
#endif
    if (g_mavlink_mgr) {
        std::cout << "正在断开MAVLink连接..." << std::endl;
        g_mavlink_mgr->stop();
    }
    if (g_flight_control) {
        std::cout << "正在关闭飞行控制模块..." << std::endl;
    }
    std::cout << " 系统关闭完成" << std::endl;
}

/** 从 YAML 加载主配置，主要读取 MAVLink 连接地址 */
bool loadConfig(const std::string& config_file) {
    try {
        if (!std::ifstream(config_file).good()) {
            std::cerr << "配置文件未找到: " << config_file << std::endl;
            return false;
        }
        
        YAML::Node config = YAML::LoadFile(config_file);
        std::cout << "Configuration loaded from: " << config_file << std::endl;

        try {
            if (config["network"] && config["network"]["mavlink"] && config["network"]["mavlink"]["connection_url"]) {
                g_mavlink_connection_url = config["network"]["mavlink"]["connection_url"].as<std::string>();
                std::cout << "MAVLink connection_url from config: " << g_mavlink_connection_url << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse MAVLink connection_url, using default: " << g_mavlink_connection_url << ", err=" << e.what() << std::endl;
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return false;
    }
}

namespace {

constexpr int kDroneIdMin = 1;
constexpr int kDroneIdMax = 3;

/** 无请求体的飞行命令 */
const char* kFlightCommandsNoBody[] = {
    "arm", "disarm", "land", "emergency_land", "return_to_launch", "hold", "takeoff"
};
constexpr size_t kFlightCommandsNoBodyCount = sizeof(kFlightCommandsNoBody) / sizeof(kFlightCommandsNoBody[0]);

/** 需要 JSON 请求体的飞行命令 */
const char* kFlightCommandsWithBody[] = {
    "set_mode", "set_altitude", "move_to"
};
constexpr size_t kFlightCommandsWithBodyCount = sizeof(kFlightCommandsWithBody) / sizeof(kFlightCommandsWithBody[0]);

/** 展示用测试用例名称（与 display/test/{case} 路径对应） */
const char* kDisplayTestCases[] = {
    "government-emergency", "university-access", "residential-privacy",
    "behavior-control", "emergency-rescue", "unauthorized-photography"
};
constexpr size_t kDisplayTestCasesCount = sizeof(kDisplayTestCases) / sizeof(kDisplayTestCases[0]);

void addFlightCommandRoutes() {
    auto addCmd = [](const std::string& suffix, bool with_body) {
        for (int id = kDroneIdMin; id <= kDroneIdMax; id++) {
            std::string path = "/api/v1/drones/" + std::to_string(id) + "/command/" + suffix;
            if (with_body) {
                http_server->addRouteWithBody(path, "POST", [](const std::string& p, const std::string& m, const std::string& b) {
                    return g_flight_api->handleFlightCommandWithParams(p, m, b);
                });
            } else {
                http_server->addRoute(path, "POST", [](const std::string& p, const std::string& m) {
                    return g_flight_api->handleFlightCommand(p, m);
                });
            }
        }
    };
    for (size_t i = 0; i < kFlightCommandsNoBodyCount; i++) {
        addCmd(kFlightCommandsNoBody[i], false);
    }
    for (size_t i = 0; i < kFlightCommandsWithBodyCount; i++) {
        addCmd(kFlightCommandsWithBody[i], true);
    }
}

void addDisplayTestRoutes() {
    const std::string prefix = "/api/v1/display/test/";
    for (size_t i = 0; i < kDisplayTestCasesCount; i++) {
        http_server->addRoute(prefix + kDisplayTestCases[i], "GET", [](const std::string& path, const std::string& method) {
            return g_display_api->handleRequest(method, path, "");
        });
    }
}

}  // namespace

/** 初始化 MAVLink 管理器与状态管理器，并按配置连接飞控 */
static void setupMavLink() {
    g_mavlink_mgr = std::make_unique<drone_control::MAVLinkManager>();
    g_state_mgr = std::make_shared<drone_control::DroneStateManager>();
    g_mavlink_mgr->setStateManager(g_state_mgr);
#ifdef USE_CONCURRENT_MAVLINK
    g_mavlink_mgr->startConcurrentMode();
    g_mavlink_io_context = std::make_unique<boost::asio::io_context>();
    g_mavlink_runner = std::make_unique<drone_control::MAVLinkManagerConcurrentRunner>(*g_mavlink_io_context, 4);
    g_mavlink_runner->addPeriodicTask(std::chrono::seconds(5), []() { if (g_mavlink_mgr) g_mavlink_mgr->runConnectionCycleOnce(); });
    g_mavlink_runner->addPeriodicTask(std::chrono::milliseconds(500), []() { if (g_mavlink_mgr) g_mavlink_mgr->runTelemetryCycleOnce(); });
    g_mavlink_runner->addPeriodicTask(std::chrono::seconds(5), []() { if (g_mavlink_mgr) g_mavlink_mgr->runHeartbeatCycleOnce(); });
    g_mavlink_runner->addPeriodicTask(std::chrono::seconds(1), []() { if (g_mavlink_mgr) g_mavlink_mgr->runAuthenticationCycleOnce(); });
    g_mavlink_runner->addPeriodicTask(std::chrono::seconds(10), []() { if (g_mavlink_mgr) g_mavlink_mgr->runReconnectionCycleOnce(); });
    g_mavlink_runner->start();
    g_mavlink_io_thread = std::thread([]() { g_mavlink_io_context->run(); });
    std::cout << "MAVLink 并发运行器已启动（事件循环 + 线程池）" << std::endl;
#else
    g_mavlink_mgr->start();
#endif
    if (!g_mavlink_mgr->connectToDrone(g_mavlink_connection_url)) {
        std::cerr << "Failed to initiate MAVLink connection to: " << g_mavlink_connection_url << std::endl;
    } else {
        std::cout << "Connecting to MAVLink at: " << g_mavlink_connection_url << std::endl;
    }
}

/** 创建访问控制引擎，加载策略文件，挂接认证提供者与 MAVLink/状态管理器 */
static void setupAccessControl() {
    access_engine = std::make_unique<AccessControlEngine>();
    try {
        auto policy_engine = std::make_unique<drone_control::access_control::ABACPolicyEngine>();
        const std::string policy_config = "../config/access_control_policies.yaml";
        std::cout << " 正在加载策略文件: " << policy_config << std::endl;
        if (policy_engine->loadPoliciesFromFile(policy_config)) {
            access_engine->setAccessControlPolicy(std::move(policy_engine));
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
        access_engine->setAuthenticationProvider(std::move(auth_provider));
        std::cout << " X.509证书认证提供者已设置" << std::endl;
    } else {
        std::cout << " 无法创建X.509证书认证提供者" << std::endl;
    }
    if (g_mavlink_mgr) access_engine->setMAVLinkManager(std::shared_ptr<MAVLinkManager>(g_mavlink_mgr.get(), [](MAVLinkManager*){}));
    if (g_state_mgr) access_engine->setStateManager(std::shared_ptr<DroneStateManager>(g_state_mgr.get(), [](DroneStateManager*){}));
}

/** 注册所有 HTTP 路由，将路径与对应 API 处理函数绑定 */
static void setupRoutes() {
    http_server->addRoute("/health", "GET", [](const std::string& path, const std::string& method) {
        return g_status_api->handleHealth(path, method);
    });
    http_server->addRoute("/api/v1/drones", "GET", [](const std::string& path, const std::string& method) {
        return g_drone_api->handleDrones(path, method);
    });
    http_server->addRoute("/api/v1/drones/states", "GET", [](const std::string& path, const std::string& method) {
        return g_drone_api->handleDroneStates(path, method);
    });
    for (int id = kDroneIdMin; id <= kDroneIdMax; id++) {
        std::string p = "/api/v1/drones/" + std::to_string(id);
        http_server->addRoute(p, "GET", [](const std::string& path, const std::string& method) {
            return g_drone_api->handleDroneDetail(path, method);
        });
    }

    addFlightCommandRoutes();

    http_server->addRoute("/api/v1/flight/auto-request", "POST", [](const std::string& path, const std::string& method) {
        return g_flight_api->handleAutoFlightRequest(path, method);
    });
    http_server->addRoute("/api/v1/flight/status", "GET", [](const std::string& path, const std::string& method) {
        return g_flight_api->handleFlightStatus(path, method);
    });
    http_server->addRouteWithBody("/api/v1/access/evaluate", "POST", [](const std::string& path, const std::string& method, const std::string& body) {
        return g_access_api->handleAccessEvaluateWithBody(path, method, body);
    });

    http_server->addRoute("/api/v1/display/policies", "GET", [](const std::string& path, const std::string& method) {
        return g_display_api->handleRequest(method, path, "");
    });
    http_server->addRoute("/api/v1/display/test-cases", "GET", [](const std::string& path, const std::string& method) {
        return g_display_api->handleRequest(method, path, "");
    });
    addDisplayTestRoutes();

    http_server->addRoute("/api/v1/resources", "GET", [](const std::string& path, const std::string& method) {
        return g_status_api->handleResources(path, method);
    });
    http_server->addRoute("/api/v1/policies", "GET", [](const std::string& path, const std::string& method) {
        return g_status_api->handlePolicies(path, method);
    });

    std::cout << "API routes configured: /health, /api/v1/drones*, /api/v1/flight*, /api/v1/access/evaluate, /api/v1/display/*, /api/v1/resources, /api/v1/policies" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "无人机访问控制 v1.0.0" << std::endl;
    std::cout << "正在初始化系统组件..." << std::endl;
    
    // 解析命令行参数
    std::string config_file = "../config/main_config.yaml";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            config_file = argv[i + 1];
            i++;
        } else if (std::string(argv[i]) == "--help") {
            std::cout << "Usage: " << argv[0] << " [--config <config_file>] [--help]" << std::endl;
            return 0;
        }
    }
    
    // 设置信号处理
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // kill命令
    signal(SIGQUIT, signal_handler);  // Ctrl+\
    signal(SIGHUP, signal_handler);   // 终端关闭
    
    try {
        // 加载配置
        if (!loadConfig(config_file)) {
            std::cerr << "Failed to load configuration, using defaults" << std::endl;
        }
        
        setupMavLink();
        setupAccessControl();

        // 初始化地理围栏签名服务
        std::cout << " 正在初始化地理围栏签名服务..." << std::endl;
        auto geofence_service = std::make_unique<GeofenceSignatureService>();
        
        // 尝试多个可能的密钥文件路径（支持不同的运行目录）
        std::vector<std::string> possible_paths = {
            "config/keys/geofence_private.pem",
            "../config/keys/geofence_private.pem",
            "../../config/keys/geofence_private.pem",
            "./config/keys/geofence_private.pem"
        };
        
        std::string private_key_file;
        std::string public_key_file;
        bool found_keys = false;
        
        for (const auto& priv_path : possible_paths) {
            std::string pub_path = priv_path;
            // 将private替换为public
            size_t pos = pub_path.find("private");
            if (pos != std::string::npos) {
                pub_path.replace(pos, 7, "public");
            }
            
            std::ifstream priv_check(priv_path);
            std::ifstream pub_check(pub_path);
            if (priv_check.good() && pub_check.good()) {
                private_key_file = priv_path;
                public_key_file = pub_path;
                found_keys = true;
                priv_check.close();
                pub_check.close();
                break;
            }
            priv_check.close();
            pub_check.close();
        }
        
        if (!found_keys) {
            std::cerr << "  错误：无法找到地理围栏密钥文件" << std::endl;
            std::cerr << "   已尝试的路径:" << std::endl;
            for (const auto& path : possible_paths) {
                std::cerr << "     - " << path << std::endl;
            }
            std::cerr << "   请确保密钥文件存在于上述路径之一" << std::endl;
            std::cerr << "   系统将继续运行，但地理围栏解锁凭证将无法生成" << std::endl;
        } else {
            if (geofence_service->initialize(private_key_file, public_key_file)) {
                // 尝试加载地理围栏签名文件（DSS方案需要）
                std::vector<std::string> geofence_sig_paths = {
                    "geofence_signed.json",
                    "../build/geofence_signed.json",
                    "../../build/geofence_signed.json",
                    "./build/geofence_signed.json"
                };
                
                bool geofence_loaded = false;
                for (const auto& sig_path : geofence_sig_paths) {
                    std::ifstream sig_file(sig_path);
                    if (sig_file.good()) {
                        sig_file.close();
                        std::string sig_content;
                        std::ifstream in(sig_path);
                        std::stringstream buffer;
                        buffer << in.rdbuf();
                        sig_content = buffer.str();
                        in.close();
                        
                        if (geofence_service->loadDSSSignedGeofence(sig_path)) {
                            std::cout << " 地理围栏签名已加载: " << sig_path << std::endl;
                            geofence_loaded = true;
                            break;
                        }
                    }
                }
                
                if (!geofence_loaded) {
                    std::cerr << "  警告：未找到地理围栏签名文件，DSS方案可能无法正常工作" << std::endl;
                    std::cerr << "   已尝试的路径:" << std::endl;
                    for (const auto& path : geofence_sig_paths) {
                        std::cerr << "     - " << path << std::endl;
                    }
                }
                
                access_engine->setGeofenceSignatureService(std::move(geofence_service));
                std::cout << " 地理围栏签名服务已初始化并设置" << std::endl;
                std::cout << "    私钥文件: " << private_key_file << std::endl;
                std::cout << "    公钥文件: " << public_key_file << std::endl;
            } else {
                std::cerr << " 地理围栏签名服务初始化失败" << std::endl;
                std::cerr << "   密钥文件路径: " << private_key_file << ", " << public_key_file << std::endl;
                std::cerr << "   系统将继续运行，但地理围栏解锁凭证将无法生成" << std::endl;
            }
        }
        
        // 初始化任务签名验证器（用于DSS方案）
        std::cout << " 正在初始化任务签名验证器（DSS方案）..." << std::endl;
        auto mission_verifier = std::make_unique<MissionSignatureVerifier>();
        if (mission_verifier->initialize()) {
            access_engine->setMissionSignatureVerifier(std::move(mission_verifier));
            std::cout << " 任务签名验证器已初始化并设置（DSS方案）" << std::endl;
        } else {
            std::cerr << " 任务签名验证器初始化失败" << std::endl;
            std::cerr << "   系统将继续运行，但DSS方案将无法使用" << std::endl;
        }
        
        // 将访问控制引擎设置到MAVLink管理器中，用于处理任务目的
        if (g_mavlink_mgr) {
            g_mavlink_mgr->setAccessControlEngine(std::shared_ptr<AccessControlEngine>(access_engine.get(), [](AccessControlEngine*){}));
        }
        
        std::cout << "Access Control Engine initialized with real-time drone data integration" << std::endl;
        
        // 创建飞行控制模块
        g_flight_control = std::make_unique<drone_control::FlightControlModule>();
        if (g_flight_control->initialize(std::shared_ptr<MAVLinkManager>(g_mavlink_mgr.get(), [](MAVLinkManager*){}))) {
            std::cout << "Flight Control Module initialized successfully" << std::endl;
        } else {
            std::cerr << "Failed to initialize Flight Control Module" << std::endl;
        }
        
        // 初始化API模块
        g_status_api = std::make_unique<api::StatusAPI>();
        g_drone_api = std::make_unique<api::DroneAPI>(
            std::shared_ptr<MAVLinkManager>(g_mavlink_mgr.get(), [](MAVLinkManager*){}),
            g_state_mgr
        );
        g_flight_api = std::make_unique<api::FlightAPI>(
            std::shared_ptr<MAVLinkManager>(g_mavlink_mgr.get(), [](MAVLinkManager*){}),
            std::shared_ptr<FlightControlModule>(g_flight_control.get(), [](FlightControlModule*){})
        );
        g_access_api = std::make_unique<api::AccessAPI>(
            std::shared_ptr<AccessControlEngine>(access_engine.get(), [](AccessControlEngine*){})
        );
        g_display_api = std::make_unique<api::AccessControlDisplayAPI>(
            std::shared_ptr<AccessControlEngine>(access_engine.get(), [](AccessControlEngine*){})
        );
        std::cout << "API modules initialized successfully" << std::endl;

        // 创建HTTP服务器
        http_server = std::make_unique<HttpServer>("0.0.0.0", 8080);
        
        // 设置静态文件根目录
        http_server->setStaticRoot("../web");
        std::cout << "Static file root set to: ../web" << std::endl;
        
        setupRoutes();
        
        // 启动HTTP服务器
        if (!http_server->start()) {
            std::cerr << "Failed to start HTTP server" << std::endl;
            return 1;
        }
        
        std::cout << "System ready. API available at http://localhost:8080" << std::endl;
        std::cout << "Try: curl http://localhost:8080/health" << std::endl;
        std::cout << "Press Ctrl+C to exit." << std::endl;
        
        // 主循环
        while (!should_exit) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 确保所有组件都已关闭
        std::cout << "\n 正在清理系统资源..." << std::endl;
        
        // 关闭性能记录器
        // if (g_performance_logger) {
        //     g_performance_logger->close();
        //     std::cout << " 性能数据已保存" << std::endl;
        // }
        
        // 清理全局资源
        if (g_flight_control) {
            g_flight_control.reset();
        }
#ifdef USE_CONCURRENT_MAVLINK
        if (g_mavlink_runner) {
            g_mavlink_runner->stop();
            g_mavlink_runner.reset();
        }
        if (g_mavlink_io_context) {
            g_mavlink_io_context->stop();
        }
        if (g_mavlink_io_thread.joinable()) {
            g_mavlink_io_thread.join();
        }
        g_mavlink_io_context.reset();
#endif
        if (g_mavlink_mgr) {
            g_mavlink_mgr.reset();
        }
        
        if (g_state_mgr) {
            g_state_mgr.reset();
        }
        
        if (access_engine) {
            access_engine.reset();
        }
        
        if (http_server) {
            http_server.reset();
        }
        
        std::cout << " 系统关闭完成，所有资源已清理" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}