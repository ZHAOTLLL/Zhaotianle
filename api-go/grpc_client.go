// gRPC 客户端：连接 C++ DroneControlCore，将 REST 请求转为 RPC 调用。
package main

import (
	"context"
	"strings"

	"github.com/access_control_core/api-go/pkg/grpcpc"
	"google.golang.org/grpc"
)

// DialCore 连接 C++ gRPC 服务，返回连接与客户端。调用方负责 conn.Close()。
func DialCore(addr string) (conn *grpc.ClientConn, client grpcpc.DroneControlCoreClient, err error) {
	conn, err = grpc.Dial(addr, grpc.WithInsecure())
	if err != nil {
		return nil, nil, err
	}
	client = grpcpc.NewDroneControlCoreClient(conn)
	return conn, client, nil
}

// EvaluateAccessViaRPC 调用 C++ EvaluateAccess，返回 json 字符串；若 rsp.error 非空则视为错误。
func EvaluateAccessViaRPC(client grpcpc.DroneControlCoreClient, requestJSON string) (string, error) {
	ctx := context.Background()
	rsp, err := client.EvaluateAccess(ctx, &grpcpc.EvaluateAccessRequest{RequestJson: requestJSON})
	if err != nil {
		return "", err
	}
	if rsp.Error != "" {
		return rsp.Json, &rpcError{msg: rsp.Error}
	}
	return rsp.Json, nil
}

type rpcError struct{ msg string }

func (e *rpcError) Error() string { return e.msg }

// CoreRPC 将 REST 路径与方法映射到 gRPC 调用。
type CoreRPC struct {
	Client grpcpc.DroneControlCoreClient
}

// GetJSON 根据 method、path、body 调用对应 RPC，返回响应 JSON 和错误。未匹配时返回 ("", nil)。
func (r *CoreRPC) GetJSON(ctx context.Context, method, path string, body []byte) (jsonStr string, err error) {
	if r.Client == nil {
		return "", nil // 调用方返回 503
	}
	switch {
	case method == "GET" && path == "/api/v1/drones":
		return r.getDrones(ctx)
	case method == "GET" && path == "/api/v1/drones/states":
		return r.getDroneStates(ctx)
	case method == "GET" && path == "/api/v1/flight/status":
		return r.getFlightStatus(ctx)
	case method == "POST" && path == "/api/v1/flight/auto-request":
		return r.autoFlightRequest(ctx)
	case method == "POST" && path == "/api/v1/access/evaluate":
		return r.evaluateAccess(ctx, string(body))
	}
	// GET /api/v1/drones/:id
	if method == "GET" && len(path) > len("/api/v1/drones/") && path != "/api/v1/drones/states" {
		if p := "/api/v1/drones/"; len(path) > len(p) && path[:len(p)] == p {
			return r.getDroneDetail(ctx, path[len(p):])
		}
	}
	// POST /api/v1/drones/:id/confirm-mission
	if method == "POST" && len(path) > len("/api/v1/drones/") {
		if idStr, ok := parseConfirmMissionPath(path); ok {
			return r.confirmMission(ctx, idStr)
		}
	}
	// POST /api/v1/drones/:id/command/:cmd [body optional]
	if method == "POST" && len(path) > len("/api/v1/drones/") {
		return r.serveFlightCommand(ctx, path, body)
	}
	return "", nil // 未匹配，调用方走 proxy 或 404
}

func (r *CoreRPC) getDrones(ctx context.Context) (string, error) {
	rsp, err := r.Client.GetDrones(ctx, &grpcpc.GetDronesRequest{})
	return jsonOrErr(rsp, err)
}

func (r *CoreRPC) getDroneStates(ctx context.Context) (string, error) {
	rsp, err := r.Client.GetDroneStates(ctx, &grpcpc.GetDroneStatesRequest{})
	return jsonOrErr(rsp, err)
}

func (r *CoreRPC) getDroneDetail(ctx context.Context, idStr string) (string, error) {
	id := parseInt32(idStr)
	rsp, err := r.Client.GetDroneDetail(ctx, &grpcpc.GetDroneDetailRequest{DroneId: id})
	return jsonOrErr(rsp, err)
}

func (r *CoreRPC) getFlightStatus(ctx context.Context) (string, error) {
	rsp, err := r.Client.GetFlightStatus(ctx, &grpcpc.GetFlightStatusRequest{})
	return jsonOrErr(rsp, err)
}

func (r *CoreRPC) autoFlightRequest(ctx context.Context) (string, error) {
	rsp, err := r.Client.AutoFlightRequest(ctx, &grpcpc.AutoFlightRequestReq{})
	return jsonOrErr(rsp, err)
}

func (r *CoreRPC) evaluateAccess(ctx context.Context, body string) (string, error) {
	rsp, err := r.Client.EvaluateAccess(ctx, &grpcpc.EvaluateAccessRequest{RequestJson: body})
	if err != nil {
		return "", err
	}
	if rsp.Error != "" {
		return rsp.Json, &rpcError{msg: rsp.Error}
	}
	return rsp.Json, nil
}

func (r *CoreRPC) confirmMission(ctx context.Context, idStr string) (string, error) {
	id := parseInt32(idStr)
	rsp, err := r.Client.ConfirmMission(ctx, &grpcpc.GetDroneDetailRequest{DroneId: id})
	return jsonOrErr(rsp, err)
}

// parseConfirmMissionPath 从 /api/v1/drones/1/confirm-mission 解析出 id "1"。
func parseConfirmMissionPath(path string) (idStr string, ok bool) {
	const prefix = "/api/v1/drones/"
	const suffix = "/confirm-mission"
	if len(path) <= len(prefix)+len(suffix) || path[:len(prefix)] != prefix {
		return "", false
	}
	rest := path[len(prefix):]
	i := strings.Index(rest, suffix)
	if i < 0 || rest[i:] != suffix {
		return "", false
	}
	idStr = rest[:i]
	for _, c := range idStr {
		if c < '0' || c > '9' {
			return "", false
		}
	}
	return idStr, idStr != ""
}

func (r *CoreRPC) serveFlightCommand(ctx context.Context, path string, body []byte) (string, error) {
	droneID, command, ok := parseCommandPath(path)
	if !ok {
		return "", nil
	}
	paramsJSON := string(body)
	if paramsJSON != "" && paramsJSON != "{}" {
		rsp, err := r.Client.FlightCommandWithParams(ctx, &grpcpc.FlightCommandWithParamsRequest{
			DroneId:    droneID,
			Command:    command,
			ParamsJson: paramsJSON,
		})
		return jsonOrErr(rsp, err)
	}
	rsp, err := r.Client.FlightCommand(ctx, &grpcpc.FlightCommandRequest{
		DroneId: droneID,
		Command: command,
	})
	return jsonOrErr(rsp, err)
}

func jsonOrErr(rsp *grpcpc.JsonResponse, err error) (string, error) {
	if err != nil {
		return "", err
	}
	if rsp != nil && rsp.Error != "" {
		return rsp.Json, &rpcError{msg: rsp.Error}
	}
	if rsp != nil {
		return rsp.Json, nil
	}
	return "{}", nil
}

func parseInt32(s string) int32 {
	var n int32
	for _, c := range s {
		if c < '0' || c > '9' {
			break
		}
		n = n*10 + int32(c-'0')
	}
	return n
}

// parseCommandPath 从 /api/v1/drones/1/command/arm 解析出 drone_id=1, command=arm。
func parseCommandPath(path string) (droneID int32, command string, ok bool) {
	const prefix = "/api/v1/drones/"
	if len(path) <= len(prefix) || path[:len(prefix)] != prefix {
		return 0, "", false
	}
	rest := path[len(prefix):]
	i := 0
	for i < len(rest) && rest[i] >= '0' && rest[i] <= '9' {
		i++
	}
	if i == 0 || i >= len(rest) {
		return 0, "", false
	}
	if rest[i] != '/' {
		return 0, "", false
	}
	rest = rest[i+1:]
	const cmdPrefix = "command/"
	if len(rest) < len(cmdPrefix) || rest[:len(cmdPrefix)] != cmdPrefix {
		return 0, "", false
	}
	command = rest[len(cmdPrefix):]
	droneID = parseInt32(path[len(prefix):])
	return droneID, command, true
}

