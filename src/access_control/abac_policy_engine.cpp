/**
 * ABAC 策略引擎
 * 加载 YAML 策略、匹配主体/资源/动作/环境条件并返回允许/拒绝及约束。
 */
#include "access_control/abac_policy_engine.hpp"
#include "access_control/policy_config_loader.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <memory>

namespace drone_control {
namespace access_control {

ABACPolicyEngine::ABACPolicyEngine() 
    : evaluator_(std::make_unique<ExpressionEvaluator>())
    , default_decision_(false)
    , conflict_resolution_strategy_("priority_based")
    , policy_version_("1.0.0")
    , stats_{}
{
    // 初始化统计信息
    stats_.total_evaluations = 0;
    stats_.permit_decisions = 0;
    stats_.deny_decisions = 0;
    stats_.indeterminate_decisions = 0;
    stats_.average_evaluation_time_ms = 0.0;
}

AccessDecision ABACPolicyEngine::evaluate(const AccessRequest& request) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        PolicyEvaluationResult result = evaluateInternal(request);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double evaluation_time_ms = duration.count() / 1000.0;
        
        updateStatistics(result, evaluation_time_ms);
        
        return createAccessDecision(result);
    } catch (const std::exception& e) {
        // 记录错误并返回默认决策
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double evaluation_time_ms = duration.count() / 1000.0;
        
        PolicyEvaluationResult error_result;
        error_result.granted = default_decision_;
        error_result.reason = "Policy evaluation error: " + std::string(e.what());
        error_result.confidence_score = 0;
        
        updateStatistics(error_result, evaluation_time_ms);
        
        return createAccessDecision(error_result);
    }
}

void ABACPolicyEngine::addPolicy(const PolicyRule& rule) {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    
    // 检查是否已存在相同ID的规则
    auto it = std::find_if(policies_.begin(), policies_.end(),
                          [&rule](const PolicyRule& existing) {
                              return existing.rule_id == rule.rule_id;
                          });
    
    if (it != policies_.end()) {
        throw std::invalid_argument("Policy rule with ID '" + rule.rule_id + "' already exists");
    }
    
    if (!validatePolicy(rule)) {
        throw std::invalid_argument("Invalid policy rule: " + rule.rule_id);
    }
    
    policies_.push_back(rule);
    
    // 按优先级排序策略（高优先级在前）
    std::sort(policies_.begin(), policies_.end(),
              [](const PolicyRule& a, const PolicyRule& b) {
                  return a.priority > b.priority;
              });
}

void ABACPolicyEngine::removePolicy(const std::string& rule_id) {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    
    auto it = std::find_if(policies_.begin(), policies_.end(),
                          [&rule_id](const PolicyRule& rule) {
                              return rule.rule_id == rule_id;
                          });
    
    if (it != policies_.end()) {
        policies_.erase(it);
    }
}

void ABACPolicyEngine::updatePolicy(const std::string& rule_id, const PolicyRule& rule) {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    
    auto it = std::find_if(policies_.begin(), policies_.end(),
                          [&rule_id](const PolicyRule& existing) {
                              return existing.rule_id == rule_id;
                          });
    
    if (it == policies_.end()) {
        throw std::invalid_argument("Policy rule with ID '" + rule_id + "' not found");
    }
    
    if (!validatePolicy(rule)) {
        throw std::invalid_argument("Invalid policy rule: " + rule.rule_id);
    }
    
    *it = rule;
    
    // 重新按优先级排序策略
    std::sort(policies_.begin(), policies_.end(),
              [](const PolicyRule& a, const PolicyRule& b) {
                  return a.priority > b.priority;
              });
}

ABACPolicyEngine::PolicyRule ABACPolicyEngine::getPolicy(const std::string& rule_id) const {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    
    auto it = std::find_if(policies_.begin(), policies_.end(),
                          [&rule_id](const PolicyRule& rule) {
                              return rule.rule_id == rule_id;
                          });
    
    if (it == policies_.end()) {
        throw std::invalid_argument("Policy rule with ID '" + rule_id + "' not found");
    }
    
    return *it;
}

std::vector<ABACPolicyEngine::PolicyRule> ABACPolicyEngine::getAllPolicies() const {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    return policies_;
}

void ABACPolicyEngine::clearAllPolicies() {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    policies_.clear();
}

bool ABACPolicyEngine::validatePolicy(const PolicyRule& rule) const {
    // 基本验证
    if (rule.rule_id.empty()) {
        return false;
    }
    
    if (rule.effect != "PERMIT" && rule.effect != "DENY" && rule.effect != "INDETERMINATE") {
        return false;
    }
    
    // 如果条件存在则验证条件
    if (!rule.subject_condition.empty() && 
        !evaluator_->validateExpression(rule.subject_condition)) {
        return false;
    }
    
    if (!rule.resource_condition.empty() && 
        !evaluator_->validateExpression(rule.resource_condition)) {
        return false;
    }
    
    if (!rule.action_condition.empty() && 
        !evaluator_->validateExpression(rule.action_condition)) {
        return false;
    }
    
    if (!rule.environment_condition.empty() && 
        !evaluator_->validateExpression(rule.environment_condition)) {
        return false;
    }
    
    return true;
}

bool ABACPolicyEngine::loadConfiguration(const std::string& config_file) {
    if (!loader_) {
        loader_ = createLoader();
    }
    return loader_->loadFromFile(config_file, *this);
}

bool ABACPolicyEngine::reloadPolicies() {
    return true;
}

std::string ABACPolicyEngine::getPolicyVersion() const {
    return policy_version_;
}

void ABACPolicyEngine::setDefaultDecision(bool granted) {
    default_decision_ = granted;
}

void ABACPolicyEngine::setConflictResolutionStrategy(const std::string& strategy) {
    if (strategy == "deny_overrides" || strategy == "permit_overrides" || 
        strategy == "first_applicable" || strategy == "priority_based") {
        conflict_resolution_strategy_ = strategy;
    } else {
        throw std::invalid_argument("Unknown conflict resolution strategy: " + strategy);
    }
}

bool ABACPolicyEngine::loadPoliciesFromFile(const std::string& policy_file) {
    if (!loader_) {
        loader_ = createLoader();
    }
    return loader_->loadFromFile(policy_file, *this);
}

void ABACPolicyEngine::savePoliciesToFile(const std::string& policy_file) const {
    if (!loader_) {
        loader_ = createLoader();
    }
    loader_->saveToFile(policy_file, *this);
}

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::evaluateInternal(const AccessRequest& request) const {
    std::map<std::string, std::string> context = buildEvaluationContext(request);
    std::vector<PolicyRule> applicable_policies = findApplicablePolicies(request);
    
    if (applicable_policies.empty()) {
        PolicyEvaluationResult result;
        result.granted = default_decision_;
        result.reason = "No applicable policies found";
        result.confidence_score = 50;
        return result;
    }
    
    // 解决适用策略之间的冲突
    return resolveConflicts(applicable_policies, context);
}

bool ABACPolicyEngine::evaluateCondition(const std::string& condition, 
                                       const std::map<std::string, std::string>& context) const {
    if (condition.empty() || condition == "true") {
        return true;
    }
    
    if (condition == "false") {
        return false;
    }
    
    return evaluator_->evaluate(condition, context);
}

std::map<std::string, std::string> ABACPolicyEngine::buildEvaluationContext(const AccessRequest& request) const {
    std::map<std::string, std::string> context;
    
    // 添加来自DroneAttributes的属性
    context["organization"] = request.attributes.organization;
    context["role"] = request.attributes.role;
    context["certificate_id"] = request.attributes.certificate_id;
    context["trust_level"] = request.attributes.trust_level;
    context["mission_type"] = request.attributes.mission_type;
    context["payload_type"] = request.attributes.payload_type;
    context["flight_mode"] = request.attributes.flight_mode;
    context["battery_level"] = std::to_string(request.attributes.battery_level);
    context["emergency_status"] = request.attributes.emergency_status;
    context["weather_condition"] = request.attributes.weather_condition;
    
    // 添加上下文信息
    for (const auto& [key, value] : request.context) {
        context[key] = value;
    }
    
    // 添加请求特定的属性
    context["drone_id"] = std::to_string(request.drone_id);
    context["target_location"] = request.target_location;
    context["operation_type"] = request.operation_type;
    
    // 添加当前时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    context["current_timestamp"] = std::to_string(time_t);
    
    return context;
}

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::resolveConflicts(
    const std::vector<PolicyRule>& applicable_rules,
    const std::map<std::string, std::string>& context) const {
    
    if (conflict_resolution_strategy_ == "deny_overrides") {
        return resolveDenyOverrides(applicable_rules, context);
    } else if (conflict_resolution_strategy_ == "permit_overrides") {
        return resolvePermitOverrides(applicable_rules, context);
    } else if (conflict_resolution_strategy_ == "first_applicable") {
        return resolveFirstApplicable(applicable_rules, context);
    } else if (conflict_resolution_strategy_ == "priority_based") {
        return resolvePriorityBased(applicable_rules, context);
    }
    
    // 默认为deny_overrides
    return resolveDenyOverrides(applicable_rules, context);
}

std::vector<ABACPolicyEngine::PolicyRule> ABACPolicyEngine::findApplicablePolicies(const AccessRequest& request) const {
    std::lock_guard<std::mutex> lock(policies_mutex_);
    std::vector<PolicyRule> applicable;
    
    std::map<std::string, std::string> context = buildEvaluationContext(request);
    
    for (const auto& policy : policies_) {
        if (policy.is_active && isPolicyApplicable(policy, context)) {
            applicable.push_back(policy);
        }
    }
    return applicable;
}

bool ABACPolicyEngine::isPolicyApplicable(const PolicyRule& rule, 
                                        const std::map<std::string, std::string>& context) const {
    // 策略要适用，所有条件都必须满足
    return evaluateCondition(rule.subject_condition, context) &&
           evaluateCondition(rule.resource_condition, context) &&
           evaluateCondition(rule.action_condition, context) &&
           evaluateCondition(rule.environment_condition, context);
}

void ABACPolicyEngine::updateStatistics(const PolicyEvaluationResult& result, 
                                       double evaluation_time_ms) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.total_evaluations++;
    
    if (result.granted) {
        stats_.permit_decisions++;
    } else {
        stats_.deny_decisions++;
    }
    
    // 更新平均评估时间
    double total_time = stats_.average_evaluation_time_ms * (stats_.total_evaluations - 1);
    stats_.average_evaluation_time_ms = (total_time + evaluation_time_ms) / stats_.total_evaluations;
    
    // 更新规则使用统计
    for (const auto& rule_id : result.applied_rules) {
        stats_.rule_usage_count[rule_id]++;
    }
}

AccessDecision ABACPolicyEngine::createAccessDecision(const PolicyEvaluationResult& result) const {
    AccessDecision decision;
    decision.granted = result.granted;
    decision.reason = result.reason;
    decision.restricted_capabilities = result.constraints;
    decision.required_actions = result.obligations;
    decision.applied_policies = result.applied_rules;
    decision.policy_version = policy_version_;
    
    return decision;
}

// Conflict resolution strategy implementations

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::resolveDenyOverrides(
    const std::vector<PolicyRule>& rules,
    const std::map<std::string, std::string>& context) const {
    
    PolicyEvaluationResult result;
    result.granted = false;
    result.confidence_score = 0;
    
    // If any rule denies, return DENY
    for (const auto& rule : rules) {
        if (rule.effect == "DENY") {
            result.granted = false;
            result.reason = "Access denied by rule: " + rule.name;
            result.applied_rules.push_back(rule.rule_id);
            result.confidence_score = 90;
            return result;
        }
    }
    
    // If any rule permits and no rule denies, return PERMIT
    for (const auto& rule : rules) {
        if (rule.effect == "PERMIT") {
            result.granted = true;
            result.reason = "Access permitted by rule: " + rule.name;
            result.applied_rules.push_back(rule.rule_id);
            result.constraints.insert(result.constraints.end(), 
                                    rule.constraints.begin(), rule.constraints.end());
            result.obligations.insert(result.obligations.end(), 
                                    rule.obligations.begin(), rule.obligations.end());
            result.confidence_score = 80;
            return result;
        }
    }
    
    // No applicable rules found
    result.granted = default_decision_;
    result.reason = "No decisive rules found, using default decision";
    result.confidence_score = 30;
    
    return result;
}

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::resolvePermitOverrides(
    const std::vector<PolicyRule>& rules,
    const std::map<std::string, std::string>& context) const {
    
    PolicyEvaluationResult result;
    result.granted = false;
    result.confidence_score = 0;
    
    // If any rule permits, return PERMIT
    for (const auto& rule : rules) {
        if (rule.effect == "PERMIT") {
            result.granted = true;
            result.reason = "Access permitted by rule: " + rule.name;
            result.applied_rules.push_back(rule.rule_id);
            result.constraints.insert(result.constraints.end(), 
                                    rule.constraints.begin(), rule.constraints.end());
            result.obligations.insert(result.obligations.end(), 
                                    rule.obligations.begin(), rule.obligations.end());
            result.confidence_score = 90;
            return result;
        }
    }
    
    // If any rule denies and no rule permits, return DENY
    for (const auto& rule : rules) {
        if (rule.effect == "DENY") {
            result.granted = false;
            result.reason = "Access denied by rule: " + rule.name;
            result.applied_rules.push_back(rule.rule_id);
            result.confidence_score = 80;
            return result;
        }
    }
    
    // No applicable rules found
    result.granted = default_decision_;
    result.reason = "No decisive rules found, using default decision";
    result.confidence_score = 30;
    
    return result;
}

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::resolveFirstApplicable(
    const std::vector<PolicyRule>& rules,
    const std::map<std::string, std::string>& context) const {
    
    PolicyEvaluationResult result;
    
    if (!rules.empty()) {
        const auto& first_rule = rules[0];
        
        if (first_rule.effect == "PERMIT") {
            result.granted = true;
            result.constraints.insert(result.constraints.end(), 
                                    first_rule.constraints.begin(), first_rule.constraints.end());
            result.obligations.insert(result.obligations.end(), 
                                    first_rule.obligations.begin(), first_rule.obligations.end());
        } else if (first_rule.effect == "DENY") {
            result.granted = false;
        } else {
            result.granted = false; // Default to deny for indeterminate
        }
        
        result.reason = "Decision based on first applicable rule: " + first_rule.name;
        result.applied_rules.push_back(first_rule.rule_id);
        result.confidence_score = 85;
    } else {
        result.granted = default_decision_;
        result.reason = "No applicable rules found, using default decision";
        result.confidence_score = 30;
    }
    
    return result;
}

ABACPolicyEngine::PolicyEvaluationResult ABACPolicyEngine::resolvePriorityBased(
    const std::vector<PolicyRule>& rules,
    const std::map<std::string, std::string>& context) const {
    
    PolicyEvaluationResult result;
    
    if (!rules.empty()) {
        // Rules are already sorted by priority (highest first)
        const auto& highest_priority_rule = rules[0];
        
        if (highest_priority_rule.effect == "PERMIT") {
            result.granted = true;
            result.constraints.insert(result.constraints.end(), 
                                    highest_priority_rule.constraints.begin(), 
                                    highest_priority_rule.constraints.end());
            result.obligations.insert(result.obligations.end(), 
                                    highest_priority_rule.obligations.begin(), 
                                    highest_priority_rule.obligations.end());
        } else if (highest_priority_rule.effect == "DENY") {
            result.granted = false;
        } else {
            result.granted = false; // Default to deny for indeterminate
        }
        
        result.reason = "Decision based on highest priority rule: " + highest_priority_rule.name;
        result.applied_rules.push_back(highest_priority_rule.rule_id);
        result.confidence_score = 95;
    } else {
        result.granted = default_decision_;
        result.reason = "No applicable rules found, using default decision";
        result.confidence_score = 30;
    }
    
    return result;
}

std::unique_ptr<PolicyConfigLoader> ABACPolicyEngine::createLoader() const {
    return std::make_unique<PolicyConfigLoader>();
}

} // namespace access_control
} // namespace drone_control