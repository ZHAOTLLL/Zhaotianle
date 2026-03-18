/**
 * @file base_attribute_provider.hpp
 * @brief 提供属性提供者的通用基类，封装初始化配置、必填项校验等重复逻辑，方便具体提供者继承扩展。
 */
#pragma once

#include "attribute_provider.hpp"
#include <vector>

namespace drone_control {

/**
 * 属性提供者基础实现类
 * 提供一些通用的实用功能
 */
class BaseAttributeProvider : public AttributeProvider {
public:
    BaseAttributeProvider(const std::string& provider_type);
    virtual ~BaseAttributeProvider() = default;
    
    std::string getProviderType() const override;
    bool initialize(const std::map<std::string, std::string>& config) override;
    
protected:
    /**
     * 子类实现具体的初始化逻辑
     * @param config 配置参数
     * @return 是否初始化成功
     */
    virtual bool doInitialize(const std::map<std::string, std::string>& config) = 0;
    
    /**
     * 检查是否已初始化
     * @return 是否已初始化
     */
    bool isInitialized() const;
    
    /**
     * 获取配置值
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    std::string getConfigValue(const std::string& key, const std::string& default_value = "") const;
    
    /**
     * 获取配置值（整数）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    int getConfigValueInt(const std::string& key, int default_value = 0) const;
    
    /**
     * 验证必需的配置项
     * @param required_keys 必需的配置键列表
     * @return 是否所有必需配置都存在
     */
    bool validateRequiredConfig(const std::vector<std::string>& required_keys) const;

private:
    std::string provider_type_;
    std::map<std::string, std::string> config_;
    bool initialized_;
};

} // namespace drone_control