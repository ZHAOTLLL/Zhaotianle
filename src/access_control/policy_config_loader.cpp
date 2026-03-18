/**
 * 策略配置加载
 * 从 YAML 文件加载策略规则并写入 ABAC 策略引擎，支持保存。
 */
#include "access_control/policy_config_loader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <regex>

namespace drone_control {
namespace access_control {

bool PolicyConfigLoader::loadFromFile(const std::string& config_file, ABACPolicyEngine& engine) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        last_error_ = "Cannot open configuration file: " + config_file;
        return false;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    if (content.empty()) {
        last_error_ = "Configuration file is empty";
        return false;
    }
    
    try {
        // Clear existing policies
        engine.clearAllPolicies();
        
        // Parse policies section
        std::map<std::string, std::string> config = parseYamlSection(content, "access_control");
        
        // Parse individual policies
        std::string policies_section = config["policies"];
        if (policies_section.empty()) {
            last_error_ = "No policies section found in configuration";
            return false;
        }
        
        // Split policies section into individual policy blocks
        std::vector<std::string> policy_blocks = splitPolicyBlocks(policies_section);
        
        for (const auto& policy_block : policy_blocks) {
            try {
                ABACPolicyEngine::PolicyRule rule = parsePolicy(policy_block);
                if (!rule.rule_id.empty()) {
                    engine.addPolicy(rule);
                }
            } catch (const std::exception& e) {
                last_error_ = "Error parsing policy rule: " + std::string(e.what());
                return false;
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        last_error_ = "Error loading configuration: " + std::string(e.what());
        return false;
    }
}

bool PolicyConfigLoader::saveToFile(const std::string& config_file, const ABACPolicyEngine& engine) {
    std::ofstream file(config_file);
    if (!file.is_open()) {
        last_error_ = "Cannot create configuration file: " + config_file;
        return false;
    }
    
    try {
        file << "# ABAC Policy Configuration\n";
        file << "# Generated automatically\n\n";
        file << "access_control:\n";
        file << "  policy_engine: \"abac\"\n\n";
        file << "  policies:\n";
        
        auto policies = engine.getAllPolicies();
        for (const auto& policy : policies) {
            file << policyToYaml(policy) << "\n";
        }
        
        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = "Error saving configuration: " + std::string(e.what());
        return false;
    }
}

std::vector<std::string> PolicyConfigLoader::validateConfigFile(const std::string& config_file) {
    std::vector<std::string> errors;
    
    if (!isValidYamlFile(config_file)) {
        errors.push_back("Invalid YAML file format");
        return errors;
    }
    
    std::ifstream file(config_file);
    if (!file.is_open()) {
        errors.push_back("Cannot open configuration file: " + config_file);
        return errors;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    try {
        std::map<std::string, std::string> config = parseYamlSection(content, "access_control");
        
        if (config.find("policies") == config.end()) {
            errors.push_back("Missing 'policies' section in configuration");
        }
        
        // Validate each policy
        std::string policies_section = config["policies"];
        std::regex rule_regex(R"(- rule_id:\s*['\"]?([^'\"\n]+)['\"]?)");
        std::sregex_iterator iter(policies_section.begin(), policies_section.end(), rule_regex);
        std::sregex_iterator end;
        
        while (iter != end) {
            std::string rule_id = iter->str(1);
            if (rule_id.empty()) {
                errors.push_back("Empty rule_id found");
            }
            ++iter;
        }
        
    } catch (const std::exception& e) {
        errors.push_back("Configuration parsing error: " + std::string(e.what()));
    }
    
    return errors;
}

ABACPolicyEngine::PolicyRule PolicyConfigLoader::parsePolicy(const std::string& policy_str) {
    ABACPolicyEngine::PolicyRule rule;
    
    std::istringstream stream(policy_str);
    std::string line;
    std::string current_key;
    std::string current_value;
    bool in_list = false;
    
    while (std::getline(stream, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        
        // Check if this is a list item
        if (line.front() == '-' && line.find(':') == std::string::npos) {
            // This is a list item
            if (in_list) {
                std::string item = trim(line.substr(1));
                item = unquote(item);
                if (current_key == "constraints") {
                    rule.constraints.push_back(item);
                } else if (current_key == "obligations") {
                    rule.obligations.push_back(item);
                }
            }
            continue;
        }
        
        size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) continue;
        
        std::string key = trim(line.substr(0, colon_pos));
        std::string value = trim(line.substr(colon_pos + 1));
        
        // Remove leading dash for list items
        if (!key.empty() && key.front() == '-') {
            key = trim(key.substr(1));
        }
        
        value = unquote(value);
        
        // Reset list state
        in_list = false;
        current_key = key;
        
        if (key == "rule_id") {
            rule.rule_id = value;
        } else if (key == "name") {
            rule.name = value;
        } else if (key == "description") {
            rule.description = value;
        } else if (key == "subject_condition") {
            rule.subject_condition = value;
        } else if (key == "resource_condition") {
            rule.resource_condition = value;
        } else if (key == "action_condition") {
            rule.action_condition = value;
        } else if (key == "environment_condition") {
            rule.environment_condition = value;
        } else if (key == "effect") {
            rule.effect = value;
        } else if (key == "priority") {
            try {
                rule.priority = std::stoi(value);
            } catch (...) {
                rule.priority = 0;
            }
        } else if (key == "is_active") {
            rule.is_active = (value == "true" || value == "True" || value == "1");
        } else if (key == "constraints") {
            if (value.empty()) {
                in_list = true;
                current_key = "constraints";
            } else {
                rule.constraints = parseYamlList(value);
            }
        } else if (key == "obligations") {
            if (value.empty()) {
                in_list = true;
                current_key = "obligations";
            } else {
                rule.obligations = parseYamlList(value);
            }
        }
    }
    
    return rule;
}

std::string PolicyConfigLoader::policyToYaml(const ABACPolicyEngine::PolicyRule& rule) {
    std::ostringstream yaml;
    
    yaml << "    - rule_id: \"" << rule.rule_id << "\"\n";
    yaml << "      name: \"" << rule.name << "\"\n";
    
    if (!rule.description.empty()) {
        yaml << "      description: \"" << rule.description << "\"\n";
    }
    
    if (!rule.subject_condition.empty()) {
        yaml << "      subject_condition: \"" << rule.subject_condition << "\"\n";
    }
    
    if (!rule.resource_condition.empty()) {
        yaml << "      resource_condition: \"" << rule.resource_condition << "\"\n";
    }
    
    if (!rule.action_condition.empty()) {
        yaml << "      action_condition: \"" << rule.action_condition << "\"\n";
    }
    
    if (!rule.environment_condition.empty()) {
        yaml << "      environment_condition: \"" << rule.environment_condition << "\"\n";
    }
    
    yaml << "      effect: \"" << rule.effect << "\"\n";
    yaml << "      priority: " << rule.priority << "\n";
    yaml << "      is_active: " << (rule.is_active ? "true" : "false") << "\n";
    
    if (!rule.constraints.empty()) {
        yaml << "      constraints:\n";
        for (const auto& constraint : rule.constraints) {
            yaml << "        - \"" << constraint << "\"\n";
        }
    }
    
    if (!rule.obligations.empty()) {
        yaml << "      obligations:\n";
        for (const auto& obligation : rule.obligations) {
            yaml << "        - \"" << obligation << "\"\n";
        }
    }
    
    return yaml.str();
}

std::map<std::string, std::string> PolicyConfigLoader::parseYamlSection(const std::string& content, const std::string& section) {
    std::map<std::string, std::string> result;
    
    // Find the section
    std::string section_marker = section + ":";
    size_t section_pos = content.find(section_marker);
    if (section_pos == std::string::npos) {
        return result;
    }
    
    // Extract section content
    size_t start_pos = section_pos + section_marker.length();
    size_t end_pos = content.length();
    
    // Find the end of this section (next section at same level or end of file)
    std::istringstream temp_stream(content.substr(start_pos));
    std::string temp_line;
    size_t current_pos = start_pos;
    
    while (std::getline(temp_stream, temp_line)) {
        // If we find a line that starts without indentation and has a colon, it's a new section
        if (!temp_line.empty() && temp_line[0] != ' ' && temp_line[0] != '\t' && 
            temp_line.find(':') != std::string::npos && temp_line != section_marker) {
            end_pos = current_pos;
            break;
        }
        current_pos += temp_line.length() + 1; // +1 for newline
    }
    
    std::string section_content = content.substr(start_pos, end_pos - start_pos);
    
    // Parse key-value pairs with proper indentation handling
    std::istringstream stream(section_content);
    std::string line;
    std::string current_key;
    std::string current_value;
    int base_indent = -1;
    
    while (std::getline(stream, line)) {
        if (line.empty() || trim(line).empty() || trim(line)[0] == '#') continue;
        
        // Calculate indentation
        int indent = 0;
        for (char c : line) {
            if (c == ' ' || c == '\t') indent++;
            else break;
        }
        
        std::string trimmed_line = trim(line);
        
        // Check if this is a new key-value pair at the base level
        size_t colon_pos = trimmed_line.find(':');
        if (colon_pos != std::string::npos) {
            if (base_indent == -1) {
                base_indent = indent;
            }
            
            if (indent == base_indent) {
                // Save previous key-value pair
                if (!current_key.empty()) {
                    result[current_key] = current_value;
                }
                
                current_key = trim(trimmed_line.substr(0, colon_pos));
                current_value = trim(trimmed_line.substr(colon_pos + 1));
            } else {
                // This is part of the current value (nested content)
                if (!current_value.empty()) {
                    current_value += "\n";
                }
                current_value += line;
            }
        } else {
            // This is a continuation of the previous value
            if (!current_value.empty()) {
                current_value += "\n";
            }
            current_value += line;
        }
    }
    
    // Save the last key-value pair
    if (!current_key.empty()) {
        result[current_key] = current_value;
    }
    
    return result;
}

std::vector<std::string> PolicyConfigLoader::splitPolicyBlocks(const std::string& policies_section) {
    std::vector<std::string> policy_blocks;
    
    std::istringstream stream(policies_section);
    std::string line;
    std::string current_block;
    bool in_policy_block = false;
    
    while (std::getline(stream, line)) {
        std::string trimmed_line = trim(line);
        
        // Skip empty lines and comments
        if (trimmed_line.empty() || trimmed_line[0] == '#') {
            if (in_policy_block) {
                current_block += line + "\n";
            }
            continue;
        }
        
        // Check if this is the start of a new policy block
        if (trimmed_line.front() == '-' && trimmed_line.find("rule_id:") != std::string::npos) {
            // Save previous block if exists
            if (in_policy_block && !current_block.empty()) {
                policy_blocks.push_back(current_block);
            }
            
            // Start new block
            current_block = line + "\n";
            in_policy_block = true;
        } else if (in_policy_block) {
            // Continue current block
            current_block += line + "\n";
        }
    }
    
    // Save the last block
    if (in_policy_block && !current_block.empty()) {
        policy_blocks.push_back(current_block);
    }
    
    return policy_blocks;
}

std::vector<std::string> PolicyConfigLoader::parseYamlList(const std::string& list_str) {
    std::vector<std::string> result;
    
    if (list_str.empty()) {
        return result;
    }
    
    // Handle inline list format: [item1, item2, item3]
    if (list_str.front() == '[' && list_str.back() == ']') {
        std::string content = list_str.substr(1, list_str.length() - 2);
        std::istringstream stream(content);
        std::string item;
        
        while (std::getline(stream, item, ',')) {
            item = trim(unquote(item));
            if (!item.empty()) {
                result.push_back(item);
            }
        }
    } else {
        // Handle multi-line list format
        std::istringstream stream(list_str);
        std::string line;
        
        while (std::getline(stream, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            
            if (line.front() == '-') {
                std::string item = trim(line.substr(1));
                item = unquote(item);
                if (!item.empty()) {
                    result.push_back(item);
                }
            }
        }
    }
    
    return result;
}

std::string PolicyConfigLoader::trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    
    size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

std::string PolicyConfigLoader::unquote(const std::string& str) {
    if (str.length() >= 2 && 
        ((str.front() == '\'' && str.back() == '\'') ||
         (str.front() == '"' && str.back() == '"'))) {
        return str.substr(1, str.length() - 2);
    }
    return str;
}

bool PolicyConfigLoader::isValidYamlFile(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return false;
    }
    
    // Basic YAML validation - check for proper structure
    std::string line;
    bool has_content = false;
    
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        
        has_content = true;
        
        // Check for basic YAML structure
        if (line.find(':') != std::string::npos || 
            line.front() == '-' || 
            line.find("---") == 0) {
            continue;
        }
    }
    
    return has_content;
}

bool PolicyConfigLoader::validatePolicySection(const std::map<std::string, std::string>& policy_data) {
    // Check required fields
    if (policy_data.find("rule_id") == policy_data.end() ||
        policy_data.find("effect") == policy_data.end()) {
        return false;
    }
    
    // Validate effect value
    std::string effect = policy_data.at("effect");
    if (effect != "PERMIT" && effect != "DENY" && effect != "INDETERMINATE") {
        return false;
    }
    
    return true;
}

} // namespace access_control
} // namespace drone_control