/**
 * @file access_control_policy.hpp
 * @brief 抽象访问控制策略接口，规范策略加载、评估与版本管理。
 */
#pragma once

#include "access_request.hpp"
#include "access_decision.hpp"

namespace drone_control {

/**
 * 访问控制策略接口
 * 定义访问控制策略的基本接口
 */
class AccessControlPolicy {
public:
    virtual ~AccessControlPolicy() = default;
    
    /**
     * 评估访问请求
     * @param request 访问请求
     * @return 访问决策
     */
    virtual AccessDecision evaluate(const AccessRequest& request) = 0;
    
    /**
     * 加载策略配置
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    virtual bool loadConfiguration(const std::string& config_file) = 0;
    
    /**
     * 重新加载策略
     * @return 是否重新加载成功
     */
    virtual bool reloadPolicies() = 0;
    
    /**
     * 获取策略版本
     * @return 策略版本
     */
    virtual std::string getPolicyVersion() const = 0;
};

} // namespace drone_control