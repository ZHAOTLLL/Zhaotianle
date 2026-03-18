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

#include "drone_control.grpc.pb.h"
#include "drone_control.pb.h"
#include "backend_init.hpp"
#include "api/drone_api.hpp"
#include "api/flight_api.hpp"
#include "api/access_api.hpp"

using drone_control::DroneControlCore;
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

static grpc::Server* g_server = nullptr;

static void signal_handler(int) {
  if (g_server) g_server->Shutdown();
}

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
  CoreServiceImpl service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  g_server = server.get();
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::cout << "gRPC server listening on " << listen << std::endl;
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
