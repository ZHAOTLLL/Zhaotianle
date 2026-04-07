/**
 * 数据库服务主文件
 */
#ifdef HAVE_GRPC

#include <grpcpp/grpcpp.h>
#include <atomic>
#include <csignal>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

#include "drone_control.grpc.pb.h"
#include "drone_control.pb.h"
#include "database/database_manager.hpp"

using drone_control::DatabaseService;
using drone_control::SaveEdgeDeviceInfoRequest;
using drone_control::SaveEdgeDeviceInfoResponse;
using drone_control::SaveDroneStateRequest;
using drone_control::SaveDroneStateResponse;

static grpc::Server* g_server = nullptr;

static void signal_handler(int) {
  if (g_server) g_server->Shutdown();
}

// 数据库服务实现
class DatabaseServiceImpl final : public DatabaseService::Service {
 public:
  DatabaseServiceImpl() {
    // 初始化数据库管理器
    database_manager_ = std::make_unique<drone_control::DatabaseManager>();
    
    // 尝试连接到数据库
    std::cout << "[PostgreSQL] 连接到: localhost:5432/drone_control" << std::endl;
    std::cout << "[Redis] 连接到: localhost:6379" << std::endl;
    
    // 实际连接代码
    bool initialized = database_manager_->initialize(
        "localhost", 5432, "drone_control", "postgres", "postgres",
        "localhost", 6379, ""
    );
    if (!initialized) {
        std::cerr << "[数据库] 初始化失败，将在无数据库模式下运行" << std::endl;
        // 继续运行，不退出
    } else {
        std::cout << "[数据库] 初始化成功" << std::endl;
        std::cout << "[数据库服务][连接] PostgreSQL连接"
                  << (database_manager_->isPostgresConnected() ? "已建立" : "未建立")
                  << "，Redis连接"
                  << (database_manager_->isRedisConnected() ? "已建立" : "未建立")
                  << std::endl;
    }

    std::thread([this]() {
      while (stats_running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "[数据库服务][DEBUG] 运行统计: save_edge_device_info="
                  << save_edge_device_info_count_.load()
                  << ", save_drone_state=" << save_drone_state_count_.load()
                  << ", total_rpc=" << (save_edge_device_info_count_.load() + save_drone_state_count_.load())
                  << std::endl;
      }
    }).detach();
  }

  ~DatabaseServiceImpl() override {
    stats_running_.store(false);
  }

  grpc::Status SaveEdgeDeviceInfo(grpc::ServerContext* ctx, const SaveEdgeDeviceInfoRequest* req, SaveEdgeDeviceInfoResponse* rsp) override {
    (void)ctx;
    save_edge_device_info_count_.fetch_add(1);
    std::cout << "[数据库服务] 收到保存边缘设备信息请求: " << req->device_id() << std::endl;
    std::cout << "[数据库服务][DEBUG] 设备信息: name=" << req->device_name()
              << ", location=" << req->location()
              << ", online=" << (req->online() ? "true" : "false") << std::endl;
    
    const bool success = database_manager_->saveEdgeDeviceInfo(
        req->device_id(), req->device_name(), req->location(), req->online());
    
    if (success) {
      std::cout << "[数据库服务] 边缘设备信息保存成功" << std::endl;
      rsp->set_success(true);
      rsp->set_message("边缘设备信息保存成功");
    } else {
      std::cerr << "[数据库服务] 边缘设备信息保存失败" << std::endl;
      rsp->set_success(false);
      rsp->set_message("边缘设备信息保存失败");
    }
    std::cout << "[数据库服务][DEBUG] 保存结果: success=" << (rsp->success() ? "true" : "false")
              << ", message=" << rsp->message() << std::endl;
    
    return grpc::Status::OK;
  }

  grpc::Status SaveDroneState(grpc::ServerContext* ctx, const SaveDroneStateRequest* req, SaveDroneStateResponse* rsp) override {
    (void)ctx;
    save_drone_state_count_.fetch_add(1);
    std::cout << "[数据库服务] 收到保存无人机状态请求: 无人机=" << req->drone_id() << std::endl;
    std::cout << "[数据库服务][DEBUG] 状态请求摘要: device=" << req->device_id()
              << ", state_json_len=" << req->state_json().size() << std::endl;
    
    drone_control::ExtendedDroneState state{};
    state.drone_id = req->drone_id();

#ifdef HAVE_NLOHMANN_JSON
    if (!req->state_json().empty()) {
      try {
        const json state_j = json::parse(req->state_json());
        if (state_j.contains("position")) {
          const auto& pos = state_j["position"];
          if (pos.contains("latitude")) state.position.latitude = pos["latitude"].get<double>();
          if (pos.contains("longitude")) state.position.longitude = pos["longitude"].get<double>();
          if (pos.contains("altitude")) state.position.altitude = pos["altitude"].get<double>();
        }
        if (state_j.contains("velocity")) {
          const auto& vel = state_j["velocity"];
          if (vel.contains("north")) state.velocity_north = vel["north"].get<double>();
          if (vel.contains("east")) state.velocity_east = vel["east"].get<double>();
          if (vel.contains("down")) state.velocity_down = vel["down"].get<double>();
        }
        if (state_j.contains("battery_percentage")) {
          state.battery_percentage = state_j["battery_percentage"].get<double>();
        }
        if (state_j.contains("is_armed")) {
          state.is_armed = state_j["is_armed"].get<bool>();
        }
      } catch (const std::exception& e) {
        std::cerr << "[数据库服务] state_json 解析失败，将按最小字段继续保存: " << e.what() << std::endl;
      }
    }
#endif

    const bool success = database_manager_->saveDroneState(req->drone_id(), state);
    
    if (success) {
      std::cout << "[数据库服务] 无人机状态保存成功" << std::endl;
      rsp->set_success(true);
      rsp->set_message("无人机状态保存成功");
    } else {
      std::cerr << "[数据库服务] 无人机状态保存失败" << std::endl;
      rsp->set_success(false);
      rsp->set_message("无人机状态保存失败");
    }
    std::cout << "[数据库服务][DEBUG] 状态入库结果: success=" << (rsp->success() ? "true" : "false")
              << ", lat=" << state.position.latitude
              << ", lon=" << state.position.longitude
              << ", alt=" << state.position.altitude << std::endl;
    
    return grpc::Status::OK;
  }

private:
  std::unique_ptr<drone_control::DatabaseManager> database_manager_;
  std::atomic<uint64_t> save_edge_device_info_count_{0};
  std::atomic<uint64_t> save_drone_state_count_{0};
  std::atomic<bool> stats_running_{true};
};

int main(int argc, char* argv[]) {
  std::string listen = "0.0.0.0:50052";
  DatabaseServiceImpl service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  g_server = server.get();
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::cout << "💾 数据库服务启动..." << std::endl;
  std::cout << "✅ 数据库服务启动成功" << std::endl;
  std::cout << "📡 监听地址: " << listen << std::endl;
  std::cout << "📋 PostgreSQL: localhost:5432/drone_control" << std::endl;
  std::cout << "📋 Redis: localhost:6379" << std::endl;
  server->Wait();
  return 0;
}

#else

#include <iostream>
int main() {
  std::cout << "Build without gRPC (HAVE_GRPC not defined). 数据库服务需启用 gRPC 后构建 database_service_app。" << std::endl;
  return 0;
}

#endif
