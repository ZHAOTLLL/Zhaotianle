# 个人项目

基于属性的访问控制 (ABAC)，支持：身份 → 区域 → 行为 三级控制，与 PX4/MAVLink 集成。
本项目旨在由隐私空域内的地面端与无人机飞控进行通信，验证外来无人机任务，并自动根据无人机上报的任务获取对应的飞行计划或飞行指令，由地面端向无人机飞控传递指令引导无人机完成任务。使得空域内隐私信息对无人机透明。
系统主要创新点是设计了一个基于ABAC的三层访问控制模型，包括身份认证，区域访问控制，行为控制。

---

## 依赖

- CMake 3.16+
- C++17
- yaml-cpp, OpenSSL, nlohmann_json, GMP (libgmp-dev)
- MAVSDK（可选，用于 PX4/MAVLink）
- Miracl Core（需先构建）
- TBB / OpenMP / Boost.Asio（可选）

## 功能演示图


### 多无人机状态监测

![多无人机状态监测](docs/images/image-20260318131515939.png)

![多无人机状态监测](docs/images/image-20260318131523861.png)

### 高级任务上传

![高级任务上传](docs/images/image-20260318131539726.png)

### 后台

![后台](docs/images/image-20260318131617893.png)

![后台](docs/images/image-20260318131627722.png)

### 访问控制策略展示

![访问控制策略展示](docs/images/image-20260318131653636.png)

### API 接口测试

![API接口测试](docs/images/image-20260318131730203.png)

### 单无人机飞行仿真

![单无人机飞行仿真](docs/images/image-20260318131824815.png)

![单无人机飞行仿真](docs/images/image-20260318131852181.png)







## 构建

```bash
# 1. 构建 Miracl Core
./build_miracl_ed25519.sh

# 2. 构建项目（需安装 gRPC 与 Protobuf，如 vcpkg install grpc）
mkdir build && cd build
cmake ..
make drone_grpc_server
```



对外仅使用 RPC，不再提供 C++ HTTP API。无人机与 C++ 访问控制**直连**（C++ 通过 MAVLink 直连飞控），不经过 Go。

## 运行后端

**方式一：仅 C++ gRPC 服务**

```bash
./build/drone_grpc_server [config.yaml]
# 默认监听 0.0.0.0:50051；MAVLink 在 config 中 connection_url 监听（如 udp://0.0.0.0:14550），局域网内自动发现并连接所有发送心跳的无人机
```

**方式二：Go 网关 + C++（对外只暴露 Go REST）**

```bash
# 终端 1：C++ gRPC 服务（唯一 C++ 入口，直连无人机）
./build/drone_grpc_server

# 终端 2：Go 网关（对外 REST，通过 gRPC 调用 C++）
cd api-go && go build -o gateway . && GRPC_TARGET=localhost:50051 ./gateway
# 默认 http://localhost:8081
```

- Proto 与代码生成：`./scripts/gen_proto.sh`
- API 概览

| 类型     | 路径 / 说明 |
|----------|--------------|
| 健康     | `GET /health` |
| 无人机   | `GET /api/v1/drones`、`GET /api/v1/drones/states`、`GET /api/v1/drones/1`～`3` |
| 飞行控制 | `POST /api/v1/drones/{1..3}/command/arm|disarm|land|takeoff|...`（7 种无参 + 3 种带参） |
| 任务     | `POST /api/v1/flight/auto-request`，`GET /api/v1/flight/status` |
| 访问评估 | `POST /api/v1/access/evaluate`（JSON body） |
| 展示     | `GET /api/v1/display/policies`，`GET /api/v1/display/test-cases`，`GET /api/v1/display/test/{case}`（6 个用例） |
| 资源策略 | `GET /api/v1/resources`，`GET /api/v1/policies` |



## 配置

- `config/` - 主配置 (YAML、证书等)
- `config/main_config.yaml` - MAVLink 连接等

## 目录说明

- `proto/` - gRPC 服务与消息定义（C++ 与 Go 共用）
- `api-go/` - Go 网关：对外 REST，对内仅通过 gRPC 调用 C++
- `src/grpc/`、`src/backend_init.cpp` - C++ 唯一服务入口：gRPC 服务端，MAVLink 直连无人机

