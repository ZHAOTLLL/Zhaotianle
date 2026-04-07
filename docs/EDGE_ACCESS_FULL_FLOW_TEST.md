# 边缘访问控制全流程测试指南

本文档用于验证如下完整链路：

1. 边缘设备收到无人机任务并发起访问控制请求  
2. 边缘侧调用核心 `EvaluateEdgeAccess`  
3. 核心从隐私数据库（Redis，数据库优先）读取目标隐私位置  
4. 核心执行访问控制策略匹配并生成飞行路径 + 令牌  
5. 核心将路径与令牌返回给边缘

## 0. 前置条件

- 已安装并启动 PostgreSQL、Redis
- `access_control_core` 已成功编译
- 当前目录：`/home/zhao/px4/access_control_core`

## 1. 编译测试所需目标

```bash
cd /home/zhao/px4/access_control_core
cmake --build build --target database_service_app drone_grpc_server test_edge_access_flow -j4
```

## 2. 启动数据库服务（后台）

```bash
cd /home/zhao/px4/access_control_core/build
./database_service_app > /tmp/flow_db.log 2>&1 &
echo $!
```

检查日志：

```bash
tail -n 30 /tmp/flow_db.log
```

预期关键字：

- `PostgreSQL 连接成功`
- `PostgreSQL初始化成功`
- `监听地址: 0.0.0.0:50052`

## 3. 写入隐私位置（模拟隐私数据库数据）

```bash
redis-cli SET "privacy:location:name:Building_A" '{"name":"Building_A","code":"DB_ONLY","latitude":31.7754,"longitude":117.182,"altitude":12}' EX 3600
```

预期输出：

- `OK`

## 4. 启动核心服务（后台）

```bash
cd /home/zhao/px4/access_control_core/build
./drone_grpc_server ../config/main_config.yaml > /tmp/flow_core.log 2>&1 &
echo $!
sleep 6
```

## 5. 发起边缘侧访问控制请求

```bash
cd /home/zhao/px4/access_control_core/build
./test_edge_access_flow
```

预期输出示例：

```text
access_granted=true
decision=PERMIT
validity_duration=45
token_length=1528
flight_plan_json_length=590
```

说明：

- `token_length > 0` 表示令牌已返回  
- `flight_plan_json_length > 0` 表示路径已返回

## 6. 校验核心侧是否命中“数据库优先”

```bash
rg "从数据库缓存加载目标位置|飞行计划生成成功|访问权限评估结果" /tmp/flow_core.log
```

预期关键字：

- `从数据库缓存加载目标位置: Building_A`
- `飞行计划生成成功`
- `访问权限评估结果: 允许`

## 7. 停止后台服务

可用如下方式停止：

```bash
pkill -f database_service_app
pkill -f drone_grpc_server
```

## 8. 一键串行执行（可选）

```bash
cd /home/zhao/px4/access_control_core/build
./database_service_app > /tmp/flow_db.log 2>&1 & DBPID=$!; \
sleep 2; \
redis-cli SET "privacy:location:name:Building_A" '{"name":"Building_A","code":"DB_ONLY","latitude":31.7754,"longitude":117.182,"altitude":12}' EX 3600; \
./drone_grpc_server ../config/main_config.yaml > /tmp/flow_core.log 2>&1 & COREPID=$!; \
sleep 6; \
./test_edge_access_flow; \
kill $COREPID $DBPID
```
