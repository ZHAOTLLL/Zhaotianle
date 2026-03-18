/**
 * 访问控制引擎
 * 串联认证、策略与属性，完成「身份 → 区域 → 行为」三级评估并产出访问决策。
 */
#pragma once

#include "access_request.hpp"
#include "access_decision.hpp"
#include "authentication/authentication_provider.hpp"
#include "access_control/access_control_policy.hpp"
#include "attributes/attribute_provider.hpp"
#include "signature/geofence_signature_service.hpp"
#include "signature/mission_signature_verifier.hpp"
#include "flight_control/flight_plan.hpp"
#include "resources/airspace_resource_manager.hpp"
#include <memory>
#include <vector>
#include <map>

namespace drone_control {

class MAVLinkManager;
class DroneStateManager;

namespace experiments {
    class PerformanceLogger;
}

/** 访问控制引擎：协调认证、策略与属性，输出允许/拒绝及约束 */
class AccessControlEngine {
public:
    AccessControlEngine();
    ~AccessControlEngine();
    
    /**
     * 评估访问请求
     * @param request 访问请求
     * @return 访问决策
     */
    AccessDecision evaluateAccess(const AccessRequest& request);

    /**
     * 设置认证提供者
     * @param provider 认证提供者
     */
    void setAuthenticationProvider(std::unique_ptr<AuthenticationProvider> provider);
    
    /**
     * 设置访问控制策略
     * @param policy 访问控制策略
     */
    void setAccessControlPolicy(std::unique_ptr<AccessControlPolicy> policy);
    
    /**
     * 添加属性提供者
     * @param provider 属性提供者
     */
    void addAttributeProvider(std::unique_ptr<AttributeProvider> provider);
    
    /**
     * 设置地理围栏签名服务
     * @param signature_service 签名服务
     */
    void setGeofenceSignatureService(std::unique_ptr<GeofenceSignatureService> signature_service);
    
    /**
     * 设置任务签名验证器（用于DSS方案）
     * @param verifier 任务签名验证器
     */
    void setMissionSignatureVerifier(std::unique_ptr<MissionSignatureVerifier> verifier);
    
    /**
     * 设置MAVLink管理器
     * @param mavlink_manager MAVLink管理器
     */
    void setMAVLinkManager(std::shared_ptr<MAVLinkManager> mavlink_manager);
    
    /**
     * 设置状态管理器
     * @param state_manager 状态管理器
     */
    void setStateManager(std::shared_ptr<DroneStateManager> state_manager);
    
    /**
     * 设置性能记录器（用于实验数据收集）
     * @param logger 性能记录器
     */
    void setPerformanceLogger(std::shared_ptr<experiments::PerformanceLogger> logger);
    
    /**
     * 聚合所有属性提供者的属性
     * @param drone_id 无人机ID
     * @return 聚合后的属性映射
     */
    std::map<std::string, std::string> aggregateAttributes(DroneId drone_id);
    
private:
    struct AuthenticationExecution {
        AccessDecision decision;
        std::map<std::string, std::string> attributes;
    };

    std::unique_ptr<AuthenticationProvider> auth_provider_;
    std::unique_ptr<AccessControlPolicy> access_policy_;
    std::vector<std::unique_ptr<AttributeProvider>> attribute_providers_;
    std::unique_ptr<class AccessDecisionGenerator> decision_generator_;
    std::unique_ptr<GeofenceSignatureService> geofence_service_;
    std::unique_ptr<MissionSignatureVerifier> mission_signature_verifier_;  // DSS任务签名验证器
    std::unique_ptr<PathPlanner> path_planner_;  // 路径规划器
    std::unique_ptr<resources::AirspaceResourceManager> resource_manager_;  // 空域资源管理器（避免重复加载）
    
    // 新增：MAVLink管理器和状态管理器引用
    std::shared_ptr<MAVLinkManager> mavlink_manager_;
    std::shared_ptr<DroneStateManager> state_manager_;
    
    // 性能记录器（用于实验数据收集）
    std::shared_ptr<experiments::PerformanceLogger> performance_logger_;
    
    /**
     * 执行身份认证级别检查
     */
    AccessDecision evaluateAuthentication(const AccessRequest& request);
    AuthenticationExecution performAuthentication(const AccessRequest& request);
    
    /**
     * 执行区域访问级别检查
     */
    AccessDecision evaluateRegionAccess(const AccessRequest& request);
    
    /**
     * 执行行为控制级别检查
     * @param request 访问请求
     * @param auth_attributes 认证阶段的属性（如target_alias等从任务签名中提取的属性）
     */
    AccessDecision evaluateBehaviorControl(const AccessRequest& request,
                                          const std::map<std::string, std::string>& auth_attributes);
    
    /**
     * 执行特权UAV的简化行为控制（应急路径）
     * 仅生成地理围栏解锁凭证和最小约束，不生成完整飞行计划
     */
    AccessDecision evaluateEmergencyBehaviorControl(const AccessRequest& request,
                                                    const std::map<std::string, std::string>& auth_attributes);
    
    /**
     * 生成决策ID
     */
    std::string generateDecisionId(const AccessRequest& request);
    
    /**
     * 生成飞行计划
     */
    std::optional<FlightPlan> generateFlightPlan(const AccessRequest& request, 
                                                 const std::map<std::string, std::string>& attributes);
    
    /**
     * 获取位置敏感级别
     */
    std::string getLocationSensitivity(const std::string& location);

    bool shouldBypassPolicyEvaluation(const std::map<std::string, std::string>& auth_attributes,
                                      const AccessRequest& request) const;

    void enrichRequestWithAuthMetadata(AccessRequest& request,
                                       const std::map<std::string, std::string>& auth_attributes) const;
    
public:
    /**
     * 验证地理围栏签名
     * @param signature 签名字符串
     * @param drone_id 无人机ID
     * @param region_id 区域ID
     * @param current_attributes 当前属性
     * @return 是否有效
     */
    bool verifyGeofenceSignature(const std::string& signature, 
                                DroneId drone_id, 
                                const std::string& region_id,
                                const std::map<std::string, std::string>& current_attributes);
    
    /**
     * 撤销无人机的地理围栏签名
     * @param drone_id 无人机ID
     * @return 撤销的签名数量
     */
    int revokeGeofenceSignatures(DroneId drone_id);
    
    /**
     * 检查并清理过期签名
     * @return 清理的签名数量
     */
    int cleanupExpiredSignatures();

private:
};

} // namespace drone_control