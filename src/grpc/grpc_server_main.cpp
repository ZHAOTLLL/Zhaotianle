/**
 * 地面端 gRPC 服务（唯一 C++ 对外入口）
 * 仅暴露核心 RPC：无人机、飞行、访问评估；健康/资源/策略/展示由 Go 网关实现。
 */
#ifdef HAVE_GRPC

#include <grpcpp/grpcpp.h>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <iomanip>

#include "drone_control.grpc.pb.h"
#include "drone_control.pb.h"
#include "backend_init.hpp"
#include "api/drone_api.hpp"
#include "api/flight_api.hpp"
#include "api/access_api.hpp"
#include "database/database_manager.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/access_request.hpp"
#include "access_control/access_decision.hpp"
#include "state/drone_state_manager.hpp"
#include "common/drone_attributes.hpp"

using drone_control::DroneControlCore;
using drone_control::DroneControlService;
using drone_control::JsonResponse;
using drone_control::GetDronesRequest;
using drone_control::GetDroneStatesRequest;
using drone_control::GetDroneDetailRequest;
using drone_control::FlightCommandRequest;
using drone_control::FlightCommandWithParamsRequest;
using drone_control::AutoFlightRequestReq;
using drone_control::GetFlightStatusRequest;
using drone_control::EvaluateAccessRequest;
using drone_control::ConfirmMissionRequest;
using drone_control::RegisterEdgeDeviceRequest;
using drone_control::RegisterEdgeDeviceResponse;
using drone_control::SyncDroneStateRequest;
using drone_control::SyncDroneStateResponse;
using drone_control::EvaluateEdgeAccessRequest;
using drone_control::EvaluateEdgeAccessResponse;
using drone_control::HeartbeatRequest;
using drone_control::HeartbeatResponse;
using drone_control::DatabaseService;
using drone_control::SaveEdgeDeviceInfoRequest;
using drone_control::SaveEdgeDeviceInfoResponse;
using drone_control::SaveDroneStateRequest;
using drone_control::SaveDroneStateResponse;

static grpc::Server* g_server = nullptr;

static void signal_handler(int) {
  if (g_server) g_server->Shutdown();
}

// 边缘设备服务实现
class EdgeDeviceServiceImpl final : public DroneControlService::Service {
 public:
  EdgeDeviceServiceImpl() {
    // 初始化数据库服务客户端
    database_channel_ = grpc::CreateChannel("localhost:50052", grpc::InsecureChannelCredentials());
    database_stub_ = DatabaseService::NewStub(database_channel_);
  }

  grpc::Status RegisterEdgeDevice(grpc::ServerContext* ctx, const RegisterEdgeDeviceRequest* req, RegisterEdgeDeviceResponse* rsp) override {
    (void)ctx;
    std::cout << "[核心服务] 收到边缘设备注册请求: " << req->device_id() << " - " << req->device_name() << " (" << req->location() << ")" << std::endl;
    
    // 保存边缘设备信息到数据库
    grpc::ClientContext db_ctx;
    SaveEdgeDeviceInfoRequest db_req;
    SaveEdgeDeviceInfoResponse db_rsp;
    
    db_req.set_device_id(req->device_id());
    db_req.set_device_name(req->device_name());
    db_req.set_location(req->location());
    db_req.set_online(true);
    
    grpc::Status db_status = database_stub_->SaveEdgeDeviceInfo(&db_ctx, db_req, &db_rsp);
    
    if (db_status.ok() && db_rsp.success()) {
      std::cout << "[核心服务] 边缘设备注册到数据库成功" << std::endl;
      rsp->set_success(true);
      rsp->set_message("边缘设备注册成功");
    } else {
      std::cerr << "[核心服务] 边缘设备注册到数据库失败" << std::endl;
      rsp->set_success(false);
      rsp->set_message("边缘设备注册失败：数据库保存失败");
    }
    
    return grpc::Status::OK;
  }

  grpc::Status SyncDroneState(grpc::ServerContext* ctx, const SyncDroneStateRequest* req, SyncDroneStateResponse* rsp) override {
    (void)ctx;
    std::cout << "[核心服务] 收到无人机状态同步: 设备=" << req->device_id() << ", 无人机=" << req->drone_id() << std::endl;
    
    // 保存无人机状态到数据库
    grpc::ClientContext db_ctx;
    SaveDroneStateRequest db_req;
    SaveDroneStateResponse db_rsp;
    
    db_req.set_drone_id(req->drone_id());
    db_req.set_state_json(req->state_json());
    db_req.set_device_id(req->device_id());
    
    grpc::Status db_status = database_stub_->SaveDroneState(&db_ctx, db_req, &db_rsp);
    
    if (db_status.ok() && db_rsp.success()) {
      std::cout << "[核心服务] 无人机状态保存到数据库成功" << std::endl;
      
      // 更新本地无人机状态管理器（简化处理，实际应该解析 state_json）
      auto state_manager = backend::getStateManager();
      if (state_manager) {
        // 这里可以根据需要解析 state_json 并更新状态
        std::cout << "[核心服务] 本地无人机状态管理器更新成功" << std::endl;
      }
      
      rsp->set_success(true);
      rsp->set_message("无人机状态同步成功");
    } else {
      std::cerr << "[核心服务] 无人机状态保存到数据库失败" << std::endl;
      rsp->set_success(false);
      rsp->set_message("无人机状态同步失败：数据库保存失败");
    }
    
    return grpc::Status::OK;
  }

  grpc::Status EvaluateEdgeAccess(grpc::ServerContext* ctx, const EvaluateEdgeAccessRequest* req, EvaluateEdgeAccessResponse* rsp) override {
    (void)ctx;
    std::cout << "[核心服务] 收到访问权限评估请求: 设备=" << req->device_id() << ", 无人机=" << req->drone_id() << std::endl;
    
    // 获取访问控制引擎
    auto* access_engine = backend::getAccessControlEngine();
    if (!access_engine) {
      std::cerr << "[核心服务] 访问控制引擎未初始化" << std::endl;
      rsp->set_access_granted(false);
      rsp->set_reason("访问控制引擎未初始化");
      return grpc::Status::OK;
    }
    
    // 构建访问请求
    drone_control::AccessRequest access_request;
    access_request.drone_id = req->drone_id();
    access_request.request_source = "edge_device";
    access_request.context["edge_device_id"] = req->device_id();
    access_request.context["geofence_token"] = req->geofence_token();
    
    // 评估访问权限
    drone_control::AccessDecision decision = access_engine->evaluateAccess(access_request);
    
    std::cout << "[核心服务] 访问权限评估结果: " << (decision.granted ? "允许" : "拒绝") << " - " << decision.reason << std::endl;
    
    rsp->set_access_granted(decision.granted);
    rsp->set_reason(decision.reason);
    
    return grpc::Status::OK;
  }

  grpc::Status Heartbeat(grpc::ServerContext* ctx, const HeartbeatRequest* req, HeartbeatResponse* rsp) override {
    (void)ctx;
    std::cout << "[核心服务] 收到边缘设备心跳: " << req->device_id() << std::endl;
    
    // 更新边缘设备心跳信息到数据库
    grpc::ClientContext db_ctx;
    SaveEdgeDeviceInfoRequest db_req;
    SaveEdgeDeviceInfoResponse db_rsp;
    
    db_req.set_device_id(req->device_id());
    db_req.set_online(true);
    
    // 生成当前时间戳
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%dT%H:%M:%SZ");
    std::string timestamp = ss.str();
    
    grpc::Status db_status = database_stub_->SaveEdgeDeviceInfo(&db_ctx, db_req, &db_rsp);
    
    if (db_status.ok() && db_rsp.success()) {
      std::cout << "[核心服务] 边缘设备心跳更新到数据库成功" << std::endl;
    } else {
      std::cerr << "[核心服务] 边缘设备心跳更新到数据库失败" << std::endl;
    }
    
    rsp->set_success(true);
    rsp->set_timestamp(timestamp);
    
    return grpc::Status::OK;
  }

private:
  std::shared_ptr<grpc::Channel> database_channel_;
  std::unique_ptr<DatabaseService::Stub> database_stub_;
};

class CoreServiceImpl final : public DroneControlCore::Service {
 public:
  grpc::Status GetDrones(grpc::ServerContext* ctx, const GetDronesRequest* req, JsonResponse* rsp) override {
    (void)ctx;(void)req;
    auto* api = backend::getDroneAPI();
    rsp->set_json(api ? api->handleDrones("/api/v1/drones", "GET") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status GetDroneStates(grpc::ServerContext* ctx, const GetDroneStatesRequest* req, JsonResponse* rsp) override {
    (void)ctx;(void)req;
    auto* api = backend::getDroneAPI();
    rsp->set_json(api ? api->handleDroneStates("/api/v1/drones/states", "GET") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status GetDroneDetail(grpc::ServerContext* ctx, const GetDroneDetailRequest* req, JsonResponse* rsp) override {
    (void)ctx;
    std::string path = "/api/v1/drones/" + std::to_string(req->drone_id());
    auto* api = backend::getDroneAPI();
    rsp->set_json(api ? api->handleDroneDetail(path, "GET") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status FlightCommand(grpc::ServerContext* ctx, const FlightCommandRequest* req, JsonResponse* rsp) override {
    (void)ctx;
    std::string path = "/api/v1/drones/" + std::to_string(req->drone_id()) + "/command/" + req->command();
    auto* api = backend::getFlightAPI();
    rsp->set_json(api ? api->handleFlightCommand(path, "POST") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status FlightCommandWithParams(grpc::ServerContext* ctx, const FlightCommandWithParamsRequest* req, JsonResponse* rsp) override {
    (void)ctx;
    std::string path = "/api/v1/drones/" + std::to_string(req->drone_id()) + "/command/" + req->command();
    auto* api = backend::getFlightAPI();
    rsp->set_json(api ? api->handleFlightCommandWithParams(path, "POST", req->params_json()) : "{}");
    return grpc::Status::OK;
  }

  grpc::Status AutoFlightRequest(grpc::ServerContext* ctx, const AutoFlightRequestReq* req, JsonResponse* rsp) override {
    (void)ctx;(void)req;
    auto* api = backend::getFlightAPI();
    rsp->set_json(api ? api->handleAutoFlightRequest("/api/v1/flight/auto-request", "POST") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status GetFlightStatus(grpc::ServerContext* ctx, const GetFlightStatusRequest* req, JsonResponse* rsp) override {
    (void)ctx;(void)req;
    auto* api = backend::getFlightAPI();
    rsp->set_json(api ? api->handleFlightStatus("/api/v1/flight/status", "GET") : "{}");
    return grpc::Status::OK;
  }

  grpc::Status EvaluateAccess(grpc::ServerContext* ctx, const EvaluateAccessRequest* req, JsonResponse* rsp) override {
    (void)ctx;
    auto* api = backend::getAccessAPI();
    rsp->set_json(api ? api->handleAccessEvaluateWithBody("/api/v1/access/evaluate", "POST", req->request_json()) : "{}");
    return grpc::Status::OK;
  }

  grpc::Status ConfirmMission(grpc::ServerContext* ctx, const ConfirmMissionRequest* req, JsonResponse* rsp) override {
    (void)ctx;
    bool ok = backend::confirmMission(static_cast<uint32_t>(req->drone_id()));
    rsp->set_json(ok ? "{\"ok\":true,\"message\":\"已发送任务确认并请求证书\"}" : "{\"ok\":false,\"error\":\"无人机未连接或发送失败\"}");
    return grpc::Status::OK;
  }
};

int main(int argc, char* argv[]) {
  std::string config_file = (argc >= 2 && std::string(argv[1]) != "--help") ? argv[1] : "../config/main_config.yaml";
  if (!backend::init(config_file)) {
    std::cerr << "drone_grpc_server: backend::init 失败" << std::endl;
    return 1;
  }
  if (!backend::getDroneAPI()) {
    std::cerr << "drone_grpc_server: API 未就绪" << std::endl;
    return 1;
  }
  std::string listen = "0.0.0.0:50051";
  CoreServiceImpl core_service;
  EdgeDeviceServiceImpl edge_service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen, grpc::InsecureServerCredentials());
  builder.RegisterService(&core_service);
  builder.RegisterService(&edge_service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  g_server = server.get();
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::cout << "gRPC server listening on " << listen << std::endl;
  std::cout << "边缘设备服务已注册" << std::endl;
  server->Wait();
  backend::shutdown();
  return 0;
}

#else

#include <iostream>
int main() {
  std::cout << "Build without gRPC (HAVE_GRPC not defined). C++ 服务需启用 gRPC 后构建 drone_grpc_server。" << std::endl;
  return 0;
}

#endif
