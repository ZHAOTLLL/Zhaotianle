/**
 * @file policy_config_loader.hpp
 * @brief 声明策略配置加载器，负责解析 YAML 策略、执行校验并驱动 ABACPolicyEngine 更新规则。
 */
#pragma once

#include "abac_policy_engine.hpp"
#include <string>
#include <vector>
#include <map>

namespace drone_control {
namespace access_control {

/**
 * @brief Policy configuration loader for YAML files
 * 
 * Loads ABAC policy rules from YAML configuration files and validates them.
 */
class PolicyConfigLoader {
public:
    PolicyConfigLoader() = default;
    ~PolicyConfigLoader() = default;
    
    /**
     * @brief Load policies from YAML file
     * @param config_file Path to YAML configuration file
     * @param engine ABAC policy engine to load policies into
     * @return true if successful, false otherwise
     */
    bool loadFromFile(const std::string& config_file, ABACPolicyEngine& engine);
    
    /**
     * @brief Save policies to YAML file
     * @param config_file Path to YAML configuration file
     * @param engine ABAC policy engine to save policies from
     * @return true if successful, false otherwise
     */
    bool saveToFile(const std::string& config_file, const ABACPolicyEngine& engine);
    
    /**
     * @brief Validate YAML configuration file
     * @param config_file Path to YAML configuration file
     * @return Vector of validation errors (empty if valid)
     */
    std::vector<std::string> validateConfigFile(const std::string& config_file);
    
    /**
     * @brief Parse policy rule from YAML-like string format
     * @param policy_str Policy string in YAML format
     * @return Parsed policy rule
     */
    ABACPolicyEngine::PolicyRule parsePolicy(const std::string& policy_str);
    
    /**
     * @brief Convert policy rule to YAML string
     * @param rule Policy rule to convert
     * @return YAML string representation
     */
    std::string policyToYaml(const ABACPolicyEngine::PolicyRule& rule);

private:
    // Simple YAML parsing helpers (without external dependencies)
    std::map<std::string, std::string> parseYamlSection(const std::string& content, const std::string& section);
    std::vector<std::string> parseYamlList(const std::string& list_str);
    std::vector<std::string> splitPolicyBlocks(const std::string& policies_section);
    std::string trim(const std::string& str);
    std::string unquote(const std::string& str);
    bool isValidYamlFile(const std::string& file_path);
    
    // Configuration validation
    bool validatePolicySection(const std::map<std::string, std::string>& policy_data);
    
public:
    std::string getLastError() const { return last_error_; }
    
    mutable std::string last_error_;
};

} // namespace access_control
} // namespace drone_control