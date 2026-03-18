/**
 * 属性提供者基类
 * BaseAttributeProvider 的通用实现，提供类型名、初始化状态等，供各具体提供者继承。
 */
#include "attributes/base_attribute_provider.hpp"
#include <iostream>

namespace drone_control {

BaseAttributeProvider::BaseAttributeProvider(const std::string& provider_type)
    : provider_type_(provider_type), initialized_(false) {
}

std::string BaseAttributeProvider::getProviderType() const {
    return provider_type_;
}

bool BaseAttributeProvider::initialize(const std::map<std::string, std::string>& config) // 初始化
{
    config_ = config;
    initialized_ = doInitialize(config);
    return initialized_;
}

bool BaseAttributeProvider::isInitialized() const // 是否已初始化
{
    return initialized_;
}

std::string BaseAttributeProvider::getConfigValue(const std::string& key, const std::string& default_value) const // 获取配置值
{
    auto it = config_.find(key);
    return (it != config_.end()) ? it->second : default_value;
}

int BaseAttributeProvider::getConfigValueInt(const std::string& key, int default_value) const // 获取配置值
{
    auto value_str = getConfigValue(key);
    if (value_str.empty()) {
        return default_value;
    }
    
    try {
        return std::stoi(value_str);
    } catch (const std::exception&) {
        return default_value;
    }
}

bool BaseAttributeProvider::validateRequiredConfig(const std::vector<std::string>& required_keys) const // 验证必需配置
{
    for (const auto& key : required_keys) {
        if (config_.find(key) == config_.end() || config_.at(key).empty()) {
            std::cerr << "Missing required configuration: " << key << std::endl;
            return false;
        }
    }
    return true;
}

} // namespace drone_control