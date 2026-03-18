/**
 * access_decision_generator.hpp - 访问决策生成器
 */
#pragma once

#include "access_request.hpp"
#include "access_decision.hpp"
#include "common/drone_attributes.hpp"
#include <string>
#include <vector>
#include <map>

namespace drone_control {

/**
 * 访问决策生成器
 * 负责基于策略评估结果生成详细的访问决策
 */
class AccessDecisionGenerator {
public:
    AccessDecisionGenerator();
    ~AccessDecisionGenerator();
    
    /**
     * 生成访问决策
     * @param request 访问请求
     * @param policy_result 策略评估结果
     * @param attributes 聚合属性
     * @param level 访问级别
     * @return 完整的访问决策
     */
    AccessDecision generateDecision(const AccessRequest& request,
                                   bool policy_granted,
                                   const std::string& policy_reason,
                                   const std::map<std::string, std::string>& attributes,
                                   AccessLevel level);

private:
    std::vector<std::string> generateBehaviorRestrictions(const std::map<std::string, std::string>& attributes,
                                                          const std::string& target_location);
    std::vector<std::string> generateRequiredActions(const std::map<std::string, std::string>& attributes,
                                                     const std::string& operation_type);
    std::string generateDetailedReason(bool granted,
                                       const std::map<std::string, std::string>& attributes,
                                       const std::string& policy_reason);
    Duration calculateValidityDuration(const std::map<std::string, std::string>& attributes,
                                       const std::string& operation_type);
    std::vector<std::string> generateImprovementSuggestions(const std::map<std::string, std::string>& attributes,
                                                            const std::string& policy_reason);
};

} // namespace drone_control