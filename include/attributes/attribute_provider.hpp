/**
 * 属性提供者接口
 * 从不同数据源（数据库、环境、遥测等）获取或更新无人机属性，供 ABAC 策略匹配使用。
 */
#pragma once

#include "common/types.hpp"
#include <string>
#include <map>

namespace drone_control {

class AttributeProvider {
public:
    virtual ~AttributeProvider() = default;
    
    /**
     * 获取无人机属性
     * @param drone_id 无人机ID
     * @return 属性映射
     */
    virtual std::map<std::string, std::string> getAttributes(DroneId drone_id) = 0;
    
    /**
     * 更新无人机属性
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param value 属性值
     * @return 是否更新成功
     */
    virtual bool updateAttribute(DroneId drone_id, const std::string& key, const std::string& value) = 0;
    
    /**
     * 批量更新属性
     * @param drone_id 无人机ID
     * @param attributes 属性映射
     * @return 是否更新成功
     */
    virtual bool updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) = 0;
    
    /**
     * 获取提供者类型
     * @return 提供者类型名称
     */
    virtual std::string getProviderType() const = 0;
    
    /**
     * 初始化提供者
     * @param config 配置参数
     * @return 是否初始化成功
     */
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
};

} // namespace drone_control