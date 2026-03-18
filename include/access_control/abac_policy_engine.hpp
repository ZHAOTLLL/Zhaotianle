/**
 * ABAC 策略引擎
 * 从 YAML 加载策略规则，根据主体/资源/动作/环境属性进行匹配并返回允许/拒绝及约束。
 */
#pragma once

#include "access_control_policy.hpp"
#include "expression_evaluator.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

namespace drone_control {
namespace access_control {

class ABACPolicyEngine : public AccessControlPolicy {
public:
    /** 策略规则结构 */
    struct PolicyRule {
        std::string rule_id;
        std::string name;
        std::string description;
        
        // ABAC conditions
        std::string subject_condition;      // Subject attribute conditions
        std::string resource_condition;     // Resource attribute conditions  
        std::string action_condition;       // Action attribute conditions
        std::string environment_condition;  // Environment attribute conditions
        
        // Rule effect and metadata
        std::string effect;                 // PERMIT, DENY, or INDETERMINATE
        int priority;                       // Higher number = higher priority
        bool is_active;                     // Whether rule is currently active
        
        // Optional constraints and obligations
        std::vector<std::string> constraints;    // Additional constraints if permitted
        std::vector<std::string> obligations;    // Actions that must be performed
        
        PolicyRule() : priority(0), is_active(true) {}
    };
    
    /** 策略评估结果 */
    struct PolicyEvaluationResult {
        bool granted;
        std::string reason;
        std::vector<std::string> applied_rules;
        std::vector<std::string> constraints;
        std::vector<std::string> obligations;
        int confidence_score;  // 0-100, higher means more confident
        
        PolicyEvaluationResult() : granted(false), confidence_score(0) {}
    };
    
    ABACPolicyEngine();
    ~ABACPolicyEngine() override = default;
    
    // AccessControlPolicy interface implementation
    AccessDecision evaluate(const AccessRequest& request) override;
    
    // Policy management
    void addPolicy(const PolicyRule& rule);
    void removePolicy(const std::string& rule_id);
    void updatePolicy(const std::string& rule_id, const PolicyRule& rule);
    PolicyRule getPolicy(const std::string& rule_id) const;
    std::vector<PolicyRule> getAllPolicies() const;
    
    // Policy loading and persistence
    bool loadPoliciesFromFile(const std::string& policy_file);
    void savePoliciesToFile(const std::string& policy_file) const;
    void clearAllPolicies();
    
    // Policy validation
    bool validatePolicy(const PolicyRule& rule) const;

    bool loadConfiguration(const std::string& config_file) override;
    bool reloadPolicies() override;
    std::string getPolicyVersion() const override;
    void setDefaultDecision(bool granted);
    void setConflictResolutionStrategy(const std::string& strategy);

private:
    struct PolicyStats {
        int total_evaluations;
        int permit_decisions;
        int deny_decisions;
        int indeterminate_decisions;
        std::map<std::string, int> rule_usage_count;
        double average_evaluation_time_ms;
    };
    std::vector<PolicyRule> policies_;
    std::unique_ptr<ExpressionEvaluator> evaluator_;
    mutable std::unique_ptr<class PolicyConfigLoader> loader_;
    mutable std::mutex policies_mutex_;
    bool default_decision_;
    std::string conflict_resolution_strategy_;
    std::string policy_version_;
    mutable PolicyStats stats_;
    mutable std::mutex stats_mutex_;
    
    // Policy evaluation helpers
    PolicyEvaluationResult evaluateInternal(const AccessRequest& request) const;
    bool evaluateCondition(const std::string& condition, 
                          const std::map<std::string, std::string>& context) const;
    std::map<std::string, std::string> buildEvaluationContext(const AccessRequest& request) const;
    std::unique_ptr<class PolicyConfigLoader> createLoader() const;
    
    // Conflict resolution
    PolicyEvaluationResult resolveConflicts(const std::vector<PolicyRule>& applicable_rules,
                                          const std::map<std::string, std::string>& context) const;
    
    // Policy matching
    std::vector<PolicyRule> findApplicablePolicies(const AccessRequest& request) const;
    bool isPolicyApplicable(const PolicyRule& rule, 
                           const std::map<std::string, std::string>& context) const;
    
    // Utility methods
    void updateStatistics(const PolicyEvaluationResult& result, 
                         double evaluation_time_ms) const;
    AccessDecision createAccessDecision(const PolicyEvaluationResult& result) const;
    
    // Built-in conflict resolution strategies
    PolicyEvaluationResult resolveDenyOverrides(const std::vector<PolicyRule>& rules,
                                              const std::map<std::string, std::string>& context) const;
    PolicyEvaluationResult resolvePermitOverrides(const std::vector<PolicyRule>& rules,
                                                const std::map<std::string, std::string>& context) const;
    PolicyEvaluationResult resolveFirstApplicable(const std::vector<PolicyRule>& rules,
                                                 const std::map<std::string, std::string>& context) const;
    PolicyEvaluationResult resolvePriorityBased(const std::vector<PolicyRule>& rules,
                                              const std::map<std::string, std::string>& context) const;
};

} // namespace access_control
} // namespace drone_control