# gRPC 生成代码目录

本目录应由 `scripts/gen_proto.sh` 生成，包含：

- `drone_control.pb.go`：Protobuf 消息
- `drone_control_grpc.pb.go`：gRPC 客户端与服务端接口

生成前请安装：

```bash
go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest
```

在项目根目录执行：

```bash
./scripts/gen_proto.sh
```

生成后，Go 网关可通过 `GRPC_TARGET=localhost:50051` 使用 gRPC 调用 C++ 核心。

一键启动（先编译 C++ 与 Go）：

```bash
./scripts/run_grpc_gateway.sh [config.yaml]
```

或手动：先启动 `build/drone_grpc_server`，再 `GRPC_TARGET=localhost:50051 ./api-gateway`。
