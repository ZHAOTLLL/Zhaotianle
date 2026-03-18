/**
 * 空域资源管理
 * 从配置加载位置与空域信息，供访问控制与飞行计划查询使用。
 */
#include "resources/airspace_resource_manager.hpp"
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>

namespace resources {
namespace {
bool startsWith(const std::string& text, const std::string& prefix) {
    return text.rfind(prefix, 0) == 0;
}
}

bool AirspaceResourceManager::loadLocationsFromConfig(const std::string& config_file) {
    locations_.clear();

    std::ifstream file(config_file);
    if (!file.is_open()) {
        return false;
    }

    Location current;
    bool in_block = false;
    std::string line;
    while (std::getline(file, line)) {
        auto trimmed = trim(line);
        if (trimmed.empty() || trimmed.rfind('#', 0) == 0) {
            continue;
        }

        if (startsWith(trimmed, "-")) {
            if (in_block) {
                locations_.push_back(current);
                current = Location{};
            }
            in_block = true;
            trimmed = trim(trimmed.substr(1));
            // 如果"-"后面直接是"name:"，需要解析这一行
            // 例如: "- name: 蕙园8栋" -> "name: 蕙园8栋"
        }

        std::string key;
        std::string value;
        if (!parseKeyValue(trimmed, key, value)) {
            continue;
        }

        if (key == "name") {
            current.name = value;
        } else if (key == "code") {
            current.code = value;
        } else if (key == "description") {
            current.description = value;
        } else if (key == "latitude" || key == "lat") {
            // 支持latitude和lat两种格式
            current.latitude = std::stod(value);
        } else if (key == "longitude" || key == "lon") {
            // 支持longitude和lon两种格式
            current.longitude = std::stod(value);
        } else if (key == "altitude" || key == "alt") {
            // 支持altitude和alt两种格式
            current.altitude = std::stod(value);
        } else if (key == "access_restrictions") {
            current.access_restrictions.clear();
            if (startsWith(value, "[")) {
                std::stringstream ss(value.substr(1, value.size() - 2));
                std::string entry;
                while (std::getline(ss, entry, ',')) {
                    current.access_restrictions.push_back(trim(entry));
                }
            } else {
                current.access_restrictions.push_back(value);
            }
        } else {
            current.attributes[key] = value;
        }
    }

    if (in_block) {
        locations_.push_back(current);
    }

    return !locations_.empty();
}

std::vector<AirspaceResourceManager::Location> AirspaceResourceManager::getAllLocations() const {
    return locations_;
}

std::optional<AirspaceResourceManager::Location> AirspaceResourceManager::getLocationByCode(const std::string& code) const {
    for (const auto& location : locations_) {
        if (location.code == code) {
            return location;
        }
    }
    return std::nullopt;
}

std::optional<AirspaceResourceManager::Location> AirspaceResourceManager::getLocationByName(const std::string& name) const {
    // 提取查询名称中的数字部分（用于模糊匹配，如"蕙园8号楼"可以匹配"蕙园8栋"）
    auto extractNumbers = [](const std::string& s) -> std::string {
        std::string result;
        for (char c : s) {
            if (std::isdigit(c)) {
                result += c;
            }
        }
        return result;
    };
    
    std::string query_numbers = extractNumbers(name);
    
    // 首先尝试精确匹配
    for (const auto& location : locations_) {
        if (location.name == name) {
            std::cout << "[AirspaceResourceManager] 精确匹配: " << name << " -> " << location.name 
                      << " (" << location.latitude << ", " << location.longitude << ")" << std::endl;
            return location;
        }
    }
    
    // 如果精确匹配失败，尝试部分匹配（包含关系）
    // 注意：部分匹配可能返回错误的结果，所以优先使用精确匹配
    for (const auto& location : locations_) {
        if (location.name.find(name) != std::string::npos ||
            name.find(location.name) != std::string::npos) {
            // 如果部分匹配，但名称差异较大，可能是错误匹配
            // 例如："蕙园8号楼"不应该匹配到"通用建筑_8"
            // 检查是否有共同的前缀（至少2个字符）
            bool has_common_prefix = false;
            size_t common_len = 0;
            for (size_t i = 0; i < std::min(name.length(), location.name.length()) && i < 10; ++i) {
                if (name[i] == location.name[i]) {
                    common_len++;
                } else {
                    break;
                }
            }
            // 如果共同前缀至少2个字符，才认为是有效匹配
            if (common_len >= 2) {
                std::cout << "[AirspaceResourceManager] 部分匹配: " << name << " -> " << location.name 
                          << " (" << location.latitude << ", " << location.longitude << ")" << std::endl;
                return location;
            }
        }
    }
    
    // 如果部分匹配也失败，尝试基于数字的模糊匹配（如"蕙园8号楼"匹配"蕙园8栋"）
    if (!query_numbers.empty()) {
        for (const auto& location : locations_) {
            std::string loc_numbers = extractNumbers(location.name);
            // 如果数字部分匹配，且名称中都包含相同的非数字部分（如"蕙园"）
            if (loc_numbers == query_numbers && !loc_numbers.empty()) {
                // 提取非数字部分进行验证（简单检查：都包含"蕙园"）
                bool has_common_prefix = false;
                // 检查是否有共同的前缀（至少2个字符）
                for (size_t i = 0; i < std::min(name.length(), location.name.length()) && i < 10; ++i) {
                    if (name[i] == location.name[i] && !std::isdigit(name[i])) {
                        if (i >= 1) {  // 至少2个字符匹配
                            has_common_prefix = true;
                            break;
                        }
                    } else if (std::isdigit(name[i]) || std::isdigit(location.name[i])) {
                        break;  // 遇到数字就停止
                    }
                }
                if (has_common_prefix) {
                    std::cout << "[AirspaceResourceManager] 数字模糊匹配: " << name << " -> " << location.name 
                              << " (" << location.latitude << ", " << location.longitude << ")" << std::endl;
                    return location;
                }
            }
        }
    }
    
    return std::nullopt;
}

std::string AirspaceResourceManager::trim(const std::string& s) {
    const auto start = s.find_first_not_of(" \t");
    if (start == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t");
    return s.substr(start, end - start + 1);
}

bool AirspaceResourceManager::parseKeyValue(const std::string& line, std::string& key, std::string& value) {
    auto colon = line.find(':');
    if (colon == std::string::npos) {
        return false;
    }
    key = trim(line.substr(0, colon));
    value = trim(line.substr(colon + 1));
    if (!value.empty() && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    return true;
}

} // namespace resources
