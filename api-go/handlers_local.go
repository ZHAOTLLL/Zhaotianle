// 非核心 API：健康、资源、策略、展示由 Go 网关直接实现，不经过 C++。
package main

import (
	"encoding/json"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

// configDir 配置目录，优先环境变量 CONFIG_DIR，否则 config 或 ../config
func configDir() string {
	if d := os.Getenv("CONFIG_DIR"); d != "" {
		return d
	}
	if _, err := os.Stat("config"); err == nil {
		return "config"
	}
	return "../config"
}

// Health 健康检查：Go 本地返回
func Health(c *gin.Context) {
	uptime := int64(0)
	if start != nil {
		uptime = int64(time.Since(*start).Seconds())
	}
	c.JSON(http.StatusOK, gin.H{
		"status":         "healthy",
		"uptime_seconds": uptime,
		"timestamp":      time.Now().Format("2006-01-02 15:04:05"),
	})
}

var start *time.Time

func init() {
	t := time.Now()
	start = &t
}

// Resources 空域资源列表：从 config/airspace_config.yaml 读取，格式与 C++ 兼容
func Resources(c *gin.Context) {
	page, _ := strconv.Atoi(c.DefaultQuery("page", "1"))
	pageSize, _ := strconv.Atoi(c.DefaultQuery("pageSize", "20"))
	if page < 1 {
		page = 1
	}
	if pageSize < 1 || pageSize > 100 {
		pageSize = 20
	}

	path := configDir() + "/airspace_config.yaml"
	data, err := os.ReadFile(path)
	if err != nil {
		c.JSON(http.StatusOK, gin.H{
			"resources": defaultResources(page, pageSize),
			"pagination": gin.H{
				"page": page, "page_size": pageSize,
				"total_count": 1, "total_pages": 1,
				"has_next": false, "has_prev": false,
			},
			"airspace_name": "示例空域", "airspace_code": "SAMPLE",
			"last_updated": time.Now().Format("2006-01-02 15:04:05"),
		})
		return
	}

	var cfg struct {
		Locations []struct {
			Name        string            `yaml:"name"`
			Code        string            `yaml:"code"`
			Description string            `yaml:"description"`
			Latitude    float64           `yaml:"latitude"`
			Longitude   float64           `yaml:"longitude"`
			Altitude    float64           `yaml:"altitude"`
			Attributes  map[string]string `yaml:"attributes"`
		} `yaml:"locations"`
	}
	_ = yaml.Unmarshal(data, &cfg)

	res := make([]gin.H, 0)
	total := len(cfg.Locations)
	if total == 0 {
		res = defaultResources(page, pageSize)
		total = 1
	} else {
		start := (page - 1) * pageSize
		end := start + pageSize
		if start >= total {
			start = total
		}
		if end > total {
			end = total
		}
		for i := start; i < end; i++ {
			loc := cfg.Locations[i]
			res = append(res, gin.H{
				"name": loc.Name, "code": loc.Code, "description": loc.Description,
				"position": gin.H{"lat": loc.Latitude, "lon": loc.Longitude, "alt": loc.Altitude},
				"attributes": loc.Attributes,
			})
		}
	}
	totalPages := (total + pageSize - 1) / pageSize
	if totalPages < 1 {
		totalPages = 1
	}
	c.JSON(http.StatusOK, gin.H{
		"resources": res,
		"pagination": gin.H{
			"page": page, "page_size": pageSize,
			"total_count": total, "total_pages": totalPages,
			"has_next": page < totalPages, "has_prev": page > 1,
		},
		"airspace_name": "合肥低空示例空域", "airspace_code": "HEFEI_SAMPLE",
		"last_updated": time.Now().Format("2006-01-02 15:04:05"),
	})
}

func defaultResources(page, pageSize int) []gin.H {
	return []gin.H{{
		"name": "合肥高新区航线", "code": "HEF-HIGH-001",
		"description": "连接高新区与政务区的低空示例航线",
		"position": gin.H{"lat": 31.8206, "lon": 117.2272, "alt": 80.0},
		"attributes": gin.H{"type": "corridor", "category": "commercial", "status": "available"},
	}}
}

// Policies 访问控制策略列表：从 config/access_control_policies.yaml 读取
func Policies(c *gin.Context) {
	page, _ := strconv.Atoi(c.DefaultQuery("page", "1"))
	pageSize, _ := strconv.Atoi(c.DefaultQuery("pageSize", "10"))
	if page < 1 {
		page = 1
	}
	if pageSize < 1 || pageSize > 50 {
		pageSize = 10
	}

	path := configDir() + "/access_control_policies.yaml"
	data, err := os.ReadFile(path)
	if err != nil {
		c.JSON(http.StatusOK, gin.H{
			"policies": []gin.H{},
			"pagination": gin.H{
				"current_page": page, "page_size": pageSize,
				"total_count": 0, "total_pages": 0,
				"has_next": false, "has_prev": false,
			},
		})
		return
	}

	var cfg struct {
		AccessControl struct {
			Policies []struct {
				RuleID      string `yaml:"rule_id"`
				Name       string `yaml:"name"`
				Description string `yaml:"description"`
				Effect     string `yaml:"effect"`
				Priority   int    `yaml:"priority"`
				IsActive   bool   `yaml:"is_active"`
				SubjectCondition  string `yaml:"subject_condition"`
				ResourceCondition string `yaml:"resource_condition"`
				ActionCondition   string `yaml:"action_condition"`
				EnvironmentCondition string `yaml:"environment_condition"`
			} `yaml:"policies"`
		} `yaml:"access_control"`
	}
	_ = yaml.Unmarshal(data, &cfg)

	total := len(cfg.AccessControl.Policies)
	start := (page - 1) * pageSize
	end := start + pageSize
	if start > total {
		start = total
	}
	if end > total {
		end = total
	}
	policies := make([]gin.H, 0)
	for i := start; i < end; i++ {
		p := cfg.AccessControl.Policies[i]
		conditions := gin.H{}
		if p.SubjectCondition != "" {
			conditions["subject"] = p.SubjectCondition
		}
		if p.ResourceCondition != "" {
			conditions["resource"] = p.ResourceCondition
		}
		if p.ActionCondition != "" {
			conditions["action"] = p.ActionCondition
		}
		if p.EnvironmentCondition != "" {
			conditions["environment"] = p.EnvironmentCondition
		}
		policies = append(policies, gin.H{
			"rule_id": p.RuleID, "name": p.Name, "description": p.Description,
			"effect": p.Effect, "priority": p.Priority, "is_active": p.IsActive,
			"conditions": conditions,
		})
	}
	totalPages := (total + pageSize - 1) / pageSize
	if totalPages < 1 {
		totalPages = 1
	}
	c.JSON(http.StatusOK, gin.H{
		"policies": policies,
		"pagination": gin.H{
			"current_page": page, "page_size": pageSize,
			"total_count": total, "total_pages": totalPages,
			"has_next": page < totalPages, "has_prev": page > 1,
		},
	})
}

// DisplayTestCases 展示用测试用例名列表（与 C++ 原 display 一致）
var displayTestCaseNames = []string{
	"government-emergency", "university-access", "residential-privacy",
	"behavior-control", "emergency-rescue", "unauthorized-photography",
}

func DisplayTestCases(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"test_cases": displayTestCaseNames})
}

// DisplayPolicies 展示用策略列表（静态，与 C++ 原逻辑一致）
func DisplayPolicies(c *gin.Context) {
	// 精简版策略展示 JSON，与前端展示兼容
	out := map[string]interface{}{
		"policies": []gin.H{
			{"rule_id": "government_emergency_full_access", "name": "政府紧急响应完全权限", "effect": "PERMIT", "priority": 100, "category": "认证策略"},
			{"rule_id": "university_educational_access", "name": "大学区域教育研究权限", "effect": "PERMIT", "priority": 80, "category": "区域访问控制"},
			{"rule_id": "residential_privacy_protection", "name": "住宅区域隐私保护", "effect": "DENY", "priority": 50, "category": "区域访问控制"},
			{"rule_id": "altitude_restriction", "name": "无人机高度限制", "effect": "PERMIT", "priority": 75, "category": "行为控制策略"},
		},
		"summary": gin.H{
			"total_policies": 20, "active_policies": 20,
			"categories":    []string{"认证策略", "区域访问控制", "行为控制策略"},
			"conflict_resolution": "priority_based",
		},
		"timestamp": time.Now().Format("2006-01-02 15:04:05"),
	}
	c.JSON(http.StatusOK, out)
}

// EvaluateAccessRequest 访问评估请求体（与 C++ /api/v1/access/evaluate 一致）
type EvaluateAccessRequest struct {
	DroneID         int                    `json:"drone_id"`
	TargetLocation  string                 `json:"target_location"`
	OperationType   string                 `json:"operation_type"`
	Attributes      map[string]interface{} `json:"attributes"`
	Context         map[string]string      `json:"context"`
}

// ExecuteDisplayTestCase 执行展示用例：将用例名映射为请求 JSON，调用 C++ EvaluateAccess RPC；若未配置 gRPC 则返回说明
func ExecuteDisplayTestCase(c *gin.Context, name string, evaluateAccess func(requestJSON string) (json string, err error)) {
	requestJSON := getDisplayTestCaseRequest(name)
	if requestJSON == "" {
		c.JSON(http.StatusOK, gin.H{
			"error":     "unknown test case: " + name,
			"timestamp": time.Now().Format("2006-01-02 15:04:05"),
		})
		return
	}
	if evaluateAccess == nil {
		c.JSON(http.StatusOK, gin.H{
			"error":     "display test execution requires GRPC_TARGET (C++ core)",
			"timestamp": time.Now().Format("2006-01-02 15:04:05"),
		})
		return
	}
	resp, err := evaluateAccess(requestJSON)
	if err != nil {
		c.JSON(http.StatusOK, gin.H{
			"error":     err.Error(),
			"timestamp": time.Now().Format("2006-01-02 15:04:05"),
		})
		return
	}
	var out interface{}
	_ = json.Unmarshal([]byte(resp), &out)
	c.JSON(http.StatusOK, out)
}

func getDisplayTestCaseRequest(name string) string {
	// 每个展示用例对应的访问评估请求 JSON（与 C++ display 用例一致）
	requests := map[string]string{
		"government-emergency": `{"drone_id":12345,"target_location":"合肥市政府","operation_type":"emergency","attributes":{"organization":"government","role":"emergency","trust_level":"high"}}`,
		"university-access":    `{"drone_id":12346,"target_location":"合肥工业大学","operation_type":"research","attributes":{"organization":"university","role":"researcher","trust_level":"medium"}}`,
		"residential-privacy":  `{"drone_id":12347,"target_location":"合肥市住宅区","operation_type":"surveillance","attributes":{"organization":"commercial","role":"operator","trust_level":"medium"}}`,
		"behavior-control":    `{"drone_id":12349,"target_location":"合肥市商业中心","operation_type":"delivery","attributes":{"organization":"commercial","role":"delivery","trust_level":"medium"}}`,
		"emergency-rescue":     `{"drone_id":12345,"target_location":"合肥市政府","operation_type":"emergency","attributes":{"organization":"government","role":"emergency","trust_level":"high"}}`,
		"unauthorized-photography": `{"drone_id":12350,"target_location":"合肥市住宅区","operation_type":"surveillance","attributes":{"organization":"unknown","role":"photographer","trust_level":"low"}}`,
	}
	return requests[name]
}

// RouteDisplay 注册展示相关路由；evaluateAccess 为 nil 时 ExecuteDisplayTestCase 返回需配置 GRPC_TARGET
func RouteDisplay(r *gin.Engine, evaluateAccess func(string) (string, error)) {
	r.GET("/api/v1/display/policies", DisplayPolicies)
	r.GET("/api/v1/display/test-cases", DisplayTestCases)
	r.GET("/api/v1/display/test/:name", func(c *gin.Context) {
		ExecuteDisplayTestCase(c, c.Param("name"), evaluateAccess)
	})
	r.POST("/api/v1/display/test/:name", func(c *gin.Context) {
		ExecuteDisplayTestCase(c, c.Param("name"), evaluateAccess)
	})
}

// Core 路由前缀：这些由 gRPC 转 C++
var corePrefixes = []string{"/api/v1/drones", "/api/v1/flight", "/api/v1/access"}

func isCoreRoute(path string) bool {
	for _, p := range corePrefixes {
		if strings.HasPrefix(path, p) {
			return true
		}
	}
	return false
}
