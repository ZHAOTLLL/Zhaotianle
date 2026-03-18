/**
 * @file attribute_aggregator.hpp
 * @brief 声明属性聚合器，负责协调多种属性提供者、做优先级合并与缓存，以供访问控制引擎快速获取完整属性。
 */
#pragma once

#include "attribute_provider.hpp"
#include "common/types.hpp"
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <mutex>

namespace drone_control {

/**
 * 属性缓存项
 */
struct AttributeCacheEntry {
    std::map<std::string, std::string> attributes;
    std::chrono::steady_clock::time_point last_update;
    std::chrono::seconds ttl;  // Time to live
    
    bool isExpired() const {
        return std::chrono::steady_clock::now() > (last_update + ttl);
    }
};

/**
 * 属性聚合器
 * 负责聚合多个属性提供者的数据，并提供缓存机制
 */
class AttributeAggregator {
public:
    AttributeAggregator();
    ~AttributeAggregator() = default;
    
    /**
     * 添加属性提供者
     * @param provider 属性提供者
     * @param priority 优先级（数值越大优先级越高）
     */
    void addProvider(std::unique_ptr<AttributeProvider> provider, int priority = 0);
    
    /**
     * 移除属性提供者
     * @param provider_type 提供者类型
     */
    void removeProvider(const std::string& provider_type);
    
    /**
     * 获取聚合后的属性
     * @param drone_id 无人机ID
     * @param use_cache 是否使用缓存
     * @return 聚合后的属性映射
     */
    std::map<std::string, std::string> getAggregatedAttributes(DroneId drone_id, bool use_cache = true);
    
    /**
     * 更新属性（会更新到相应的提供者）
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param value 属性值
     * @param provider_type 指定的提供者类型（为空则使用第一个支持的提供者）
     * @return 是否更新成功
     */
    bool updateAttribute(DroneId drone_id, const std::string& key, const std::string& value, 
                        const std::string& provider_type = "");
    
    /**
     * 批量更新属性
     * @param drone_id 无人机ID
     * @param attributes 属性映射
     * @param provider_type 指定的提供者类型
     * @return 是否更新成功
     */
    bool updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes,
                         const std::string& provider_type = "");
    
    /**
     * 清除缓存
     * @param drone_id 无人机ID（为空则清除所有缓存）
     */
    void clearCache(DroneId drone_id = 0);
    
    /**
     * 设置默认缓存TTL
     * @param ttl 缓存生存时间
     */
    void setDefaultCacheTTL(std::chrono::seconds ttl);
    
    /**
     * 获取提供者列表
     * @return 提供者类型列表
     */
    std::vector<std::string> getProviderTypes() const;
    
    /**
     * 强制刷新属性（忽略缓存）
     * @param drone_id 无人机ID
     * @return 刷新后的属性映射
     */
    std::map<std::string, std::string> refreshAttributes(DroneId drone_id);

private:
    struct ProviderInfo {
        std::unique_ptr<AttributeProvider> provider;
        int priority;
        
        bool operator<(const ProviderInfo& other) const {
            return priority > other.priority;  // 高优先级排在前面
        }
    };
    
    std::vector<ProviderInfo> providers_;
    std::map<DroneId, AttributeCacheEntry> cache_;
    std::chrono::seconds default_cache_ttl_;
    mutable std::mutex providers_mutex_;
    mutable std::mutex cache_mutex_;
    
    /**
     * 从所有提供者获取属性并合并
     * @param drone_id 无人机ID
     * @return 合并后的属性映射
     */
    std::map<std::string, std::string> fetchFromProviders(DroneId drone_id);
    
    /**
     * 合并属性映射（高优先级覆盖低优先级）
     * @param base 基础属性映射
     * @param overlay 覆盖属性映射
     * @return 合并后的属性映射
     */
    std::map<std::string, std::string> mergeAttributes(
        const std::map<std::string, std::string>& base,
        const std::map<std::string, std::string>& overlay);
    
    /**
     * 清理过期缓存
     */
    void cleanupExpiredCache();
};

} // namespace drone_control