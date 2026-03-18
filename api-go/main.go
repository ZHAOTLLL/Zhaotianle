// 网关主程序：对外提供 REST API。
// 健康、资源、策略、展示由 Go 本地实现；核心接口（无人机/飞行/访问评估）通过 gRPC 调 C++ 或 HTTP 代理。
package main

import (
	"fmt"
	"io"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"path/filepath"
	"strings"

	"github.com/gin-gonic/gin"
)

const (
	defaultBackendURL = "http://localhost:8080"
	defaultListen    = ":8081"
)

func main() {
	var proxy *httputil.ReverseProxy
	var coreRPC *CoreRPC
	grpcTarget := os.Getenv("GRPC_TARGET")

	if grpcTarget != "" {
		conn, client, err := DialCore(grpcTarget)
		if err != nil {
			panic("GRPC_TARGET 连接失败: " + err.Error())
		}
		defer conn.Close()
		coreRPC = &CoreRPC{Client: client}
		_ = coreRPC
	}

	if coreRPC == nil {
		backendURL := os.Getenv("BACKEND_URL")
		if backendURL == "" {
			backendURL = defaultBackendURL
		}
		target, err := url.Parse(backendURL)
		if err != nil {
			panic("invalid BACKEND_URL: " + err.Error())
		}
		proxy = httputil.NewSingleHostReverseProxy(target)
		proxy.Director = func(req *http.Request) {
			req.URL.Scheme = target.Scheme
			req.URL.Host = target.Host
			req.Host = target.Host
		}
	}

	gin.SetMode(gin.ReleaseMode)
	r := gin.Default()

	// 前端静态文件：优先 WEB_DIR；否则相对可执行文件 ../web（api-gateway 在 api-go/ 下时）
	webDir := os.Getenv("WEB_DIR")
	if webDir == "" {
		if exe, err := os.Executable(); err == nil {
			webDir = filepath.Clean(filepath.Join(filepath.Dir(exe), "..", "web"))
		}
	}
	if webDir == "" {
		webDir = "web"
	}
	if _, err := os.Stat(webDir); err == nil {
		r.Static("/web", webDir)
	}

	// 非核心接口：Go 本地实现
	r.GET("/health", Health)
	r.GET("/api/v1/resources", Resources)
	r.GET("/api/v1/policies", Policies)
	var evaluateAccess func(string) (string, error)
	if coreRPC != nil {
		evaluateAccess = func(req string) (string, error) { return EvaluateAccessViaRPC(coreRPC.Client, req) }
	}
	RouteDisplay(r, evaluateAccess)

	// 核心接口：gRPC 或 HTTP 代理（仅通配，避免与 *path 冲突）
	if coreRPC != nil {
		r.Any("/api/v1/drones/*path", coreHandler(coreRPC))
		r.Any("/api/v1/flight/*path", coreHandler(coreRPC))
		r.Any("/api/v1/access/*path", coreHandler(coreRPC))
	} else {
		r.Any("/api/v1/drones/*path", func(c *gin.Context) { proxy.ServeHTTP(c.Writer, c.Request) })
		r.Any("/api/v1/flight/*path", func(c *gin.Context) { proxy.ServeHTTP(c.Writer, c.Request) })
		r.Any("/api/v1/access/*path", func(c *gin.Context) { proxy.ServeHTTP(c.Writer, c.Request) })
		r.Any("/api/*path", func(c *gin.Context) { proxy.ServeHTTP(c.Writer, c.Request) })
	}

	r.GET("/", func(c *gin.Context) {
		mode := "http-proxy"
		backend := defaultBackendURL
		if coreRPC != nil {
			mode = "grpc"
			backend = grpcTarget
		}
		c.JSON(http.StatusOK, gin.H{
			"service": "access-control-api",
			"layer":   "go",
			"backend": backend,
			"mode":    mode,
		})
	})
	if proxy != nil {
		r.NoRoute(func(c *gin.Context) { proxy.ServeHTTP(c.Writer, c.Request) })
	}

	listen := os.Getenv("LISTEN")
	if listen == "" {
		listen = defaultListen
	}
	if !strings.HasPrefix(listen, ":") {
		listen = ":" + listen
	}
	fmt.Println("Gateway listening on", listen)
	if coreRPC != nil {
		fmt.Println("mode: grpc, target:", grpcTarget)
	} else {
		fmt.Println("mode: http-proxy")
	}
	if _, err := os.Stat(webDir); err == nil {
		fmt.Println("web UI: http://localhost" + listen + "/web/index.html")
	}
	_ = r.Run(listen)
}

// coreHandler 将当前请求转发到 CoreRPC.GetJSON 并写回 JSON。
func coreHandler(rpc *CoreRPC) gin.HandlerFunc {
	return func(c *gin.Context) {
		path := c.Request.URL.Path
		body, _ := io.ReadAll(c.Request.Body)
		jsonStr, err := rpc.GetJSON(c.Request.Context(), c.Request.Method, path, body)
		if jsonStr == "" && err == nil {
			c.JSON(http.StatusNotFound, gin.H{"error": "no matching RPC"})
			return
		}
		if err != nil {
			// RPC 错误仍可能返回了 json（如 C++ 的 error 字段）
			c.Data(http.StatusOK, "application/json", []byte(jsonStr))
			return
		}
		c.Data(http.StatusOK, "application/json", []byte(jsonStr))
	}
}
