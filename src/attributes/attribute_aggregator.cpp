/**
 * 属性聚合
 * 汇总多个属性提供者的属性并合并为一张表，供策略引擎使用。
 */
#include "attributes/attribute_aggregator.hpp"
#include <algorithm>
#include <iostream>

namespace drone_control {

AttributeAggregator::AttributeAggregator() 
    : default_cache_ttl_(std::chrono::seconds(300)) {  // 默认5分钟缓存
}

void AttributeAggregator::addProvider(std::unique_ptr<AttributeProvider> provider, int priority) {
    if (!provider) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    // 检查是否已存在相同类型的提供者
    auto provider_type = provider->getProviderType();
    auto it = std::find_if(providers_.begin(), providers_.end(),
        [&provider_type](const ProviderInfo& info) {
            return info.provider->getProviderType() == provider_type;
        });
    
    if (it != providers_.end()) {
        // 替换现有提供者
        it->provider = std::move(provider);
        it->priority = priority;
    } else {
        // 添加新提供者
        providers_.push_back({std::move(provider), priority});
    }
    
    // 按优先级排序
    std::sort(providers_.begin(), providers_.end());
}

void AttributeAggregator::removeProvider(const std::string& provider_type) {
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    providers_.erase(
        std::remove_if(providers_.begin(), providers_.end(),
            [&provider_type](const ProviderInfo& info) {
                return info.provider->getProviderType() == provider_type;
            }),
        providers_.end());
}

std::map<std::string, std::string> AttributeAggregator::getAggregatedAttributes(DroneId drone_id, bool use_cache) {
    // 【实验模式】禁用缓存机制，确保每次测试都重新获取属性
    // if (use_cache) {
    //     std::lock_guard<std::mutex> cache_lock(cache_mutex_);
    //     auto cache_it = cache_.find(drone_id);
    //     if (cache_it != cache_.end() && !cache_it->second.isExpired()) {
    //         return cache_it->second.attributes;
    //     }
    // }
    
    // 缓存未命中或已过期，从提供者获取
    auto attributes = fetchFromProviders(drone_id);
    
    // 【实验模式】禁用缓存机制，确保每次测试都重新获取属性
    // 更新缓存
    // if (use_cache) {
    //     std::lock_guard<std::mutex> cache_lock(cache_mutex_);
    //     cache_[drone_id] = {
    //         attributes,
    //         std::chrono::steady_clock::now(),
    //         default_cache_ttl_
    //     };
    // }
    
    return attributes;
}

bool AttributeAggregator::updateAttribute(DroneId drone_id, const std::string& key, 
                                        const std::string& value, const std::string& provider_type) {
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    bool updated = false;
    
    if (provider_type.empty()) {
        // 尝试所有提供者，直到有一个成功
        for (auto& provider_info : providers_) {
            if (provider_info.provider->updateAttribute(drone_id, key, value)) {
                updated = true;
                break;
            }
        }
    } else {
        // 使用指定的提供者
        auto it = std::find_if(providers_.begin(), providers_.end(),
            [&provider_type](const ProviderInfo& info) {
                return info.provider->getProviderType() == provider_type;
            });
        
        if (it != providers_.end()) {
            updated = it->provider->updateAttribute(drone_id, key, value);
        }
    }
    
    if (updated) {
        // 【实验模式】禁用缓存机制
        // 清除缓存以强制下次获取时重新加载
        // std::lock_guard<std::mutex> cache_lock(cache_mutex_);
        // cache_.erase(drone_id);
    }
    
    return updated;
}

bool AttributeAggregator::updateAttributes(DroneId drone_id, 
                                         const std::map<std::string, std::string>& attributes,
                                         const std::string& provider_type) {
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    bool updated = false;
    
    if (provider_type.empty()) {
        // 尝试所有提供者，直到有一个成功
        for (auto& provider_info : providers_) {
            if (provider_info.provider->updateAttributes(drone_id, attributes)) {
                updated = true;
                break;
            }
        }
    } else {
        // 使用指定的提供者
        auto it = std::find_if(providers_.begin(), providers_.end(),
            [&provider_type](const ProviderInfo& info) {
                return info.provider->getProviderType() == provider_type;
            });
        
        if (it != providers_.end()) {
            updated = it->provider->updateAttributes(drone_id, attributes);
        }
    }
    
    if (updated) {
        // 【实验模式】禁用缓存机制
        // 清除缓存以强制下次获取时重新加载
        // std::lock_guard<std::mutex> cache_lock(cache_mutex_);
        // cache_.erase(drone_id);
    }
    
    return updated;
}

void AttributeAggregator::clearCache(DroneId drone_id) {
    // 【实验模式】禁用缓存机制
    // std::lock_guard<std::mutex> lock(cache_mutex_);
    // 
    // if (drone_id == 0) {
    //     cache_.clear();
    // } else {
    //     cache_.erase(drone_id);
    // }
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    if (drone_id == 0) {
        cache_.clear();
    } else {
        cache_.erase(drone_id);
    }
}

void AttributeAggregator::setDefaultCacheTTL(std::chrono::seconds ttl) {
    default_cache_ttl_ = ttl;
}

std::vector<std::string> AttributeAggregator::getProviderTypes() const {
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    std::vector<std::string> types;
    types.reserve(providers_.size());
    
    for (const auto& provider_info : providers_) {
        types.push_back(provider_info.provider->getProviderType());
    }
    
    return types;
}

std::map<std::string, std::string> AttributeAggregator::refreshAttributes(DroneId drone_id) {
    return getAggregatedAttributes(drone_id, false);  // 强制不使用缓存
}

std::map<std::string, std::string> AttributeAggregator::fetchFromProviders(DroneId drone_id) {
    std::lock_guard<std::mutex> lock(providers_mutex_);
    
    std::map<std::string, std::string> aggregated_attributes;
    
    // 按优先级从低到高合并属性（高优先级覆盖低优先级）
    for (auto it = providers_.rbegin(); it != providers_.rend(); ++it) {
        try {
            auto provider_attributes = it->provider->getAttributes(drone_id);
            aggregated_attributes = mergeAttributes(aggregated_attributes, provider_attributes);
        } catch (const std::exception& e) {
            std::cerr << "Error getting attributes from provider " 
                      << it->provider->getProviderType() << ": " << e.what() << std::endl;
            // 继续处理其他提供者
        }
    }
    
    // 定期清理过期缓存
    cleanupExpiredCache();
    
    return aggregated_attributes;
}

std::map<std::string, std::string> AttributeAggregator::mergeAttributes(
    const std::map<std::string, std::string>& base,
    const std::map<std::string, std::string>& overlay) {
    
    auto result = base;
    
    for (const auto& [key, value] : overlay) {
        result[key] = value;  // 覆盖或添加新属性
    }
    
    return result;
}

void AttributeAggregator::cleanupExpiredCache() {
    // 【实验模式】禁用缓存机制
    // std::lock_guard<std::mutex> lock(cache_mutex_);
    // auto now = std::chrono::steady_clock::now();
    // 
    // for (auto it = cache_.begin(); it != cache_.end();) {
    //     if (it->second.isExpired()) {
    //         it = cache_.erase(it);
    //     } else {
    //         ++it;
    //     }
    // }
}

} // namespace drone_control