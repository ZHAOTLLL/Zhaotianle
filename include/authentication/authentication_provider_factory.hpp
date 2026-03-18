#pragma once

#include "authentication_provider.hpp"
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <functional>

namespace drone_control {

/**
 * 认证提供者工厂类
 * 使用工厂模式创建不同类型的认证提供者
 */
class AuthenticationProviderFactory {
public:
    using ProviderCreator = std::function<std::unique_ptr<AuthenticationProvider>()>;
    
    /**
     * 获取工厂单例实例
     */
    static AuthenticationProviderFactory& getInstance();
    
    /**
     * 注册认证提供者创建函数
     * @param provider_type 提供者类型名称
     * @param creator 创建函数
     */
    void registerProvider(const std::string& provider_type, ProviderCreator creator);
    
    /**
     * 创建认证提供者实例
     * @param provider_type 提供者类型名称
     * @return 认证提供者实例，如果类型不存在则返回nullptr
     */
    std::unique_ptr<AuthenticationProvider> createProvider(const std::string& provider_type);
    
    /**
     * 获取所有已注册的提供者类型
     * @return 提供者类型列表
     */
    std::vector<std::string> getRegisteredTypes() const;
    
    /**
     * 检查提供者类型是否已注册
     * @param provider_type 提供者类型名称
     * @return 是否已注册
     */
    bool isProviderRegistered(const std::string& provider_type) const;

private:
    AuthenticationProviderFactory() = default;
    ~AuthenticationProviderFactory() = default;
    
    // 禁止拷贝和赋值
    AuthenticationProviderFactory(const AuthenticationProviderFactory&) = delete;
    AuthenticationProviderFactory& operator=(const AuthenticationProviderFactory&) = delete;
    
    std::map<std::string, ProviderCreator> providers_;
};

/**
 * 认证提供者注册器辅助类
 * 用于自动注册认证提供者类型
 */
template<typename T>
class AuthenticationProviderRegistrar {
public:
    explicit AuthenticationProviderRegistrar(const std::string& provider_type) {
        AuthenticationProviderFactory::getInstance().registerProvider(
            provider_type,
            []() -> std::unique_ptr<AuthenticationProvider> {
                return std::make_unique<T>();
            }
        );
    }
};

// 宏定义，简化提供者注册
#define REGISTER_AUTHENTICATION_PROVIDER(ClassName, TypeName) \
    static AuthenticationProviderRegistrar<ClassName> g_##ClassName##_registrar(TypeName)

} // namespace drone_control