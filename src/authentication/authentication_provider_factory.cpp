/**
 * 认证提供者工厂
 * 按类型名注册与创建认证提供者（如 x509_certificate），供访问控制引擎使用。
 */
#include "../../include/authentication/authentication_provider_factory.hpp"
#include <stdexcept>
#include <algorithm>

namespace drone_control {

AuthenticationProviderFactory& AuthenticationProviderFactory::getInstance() {
    static AuthenticationProviderFactory instance;
    return instance;
}

void AuthenticationProviderFactory::registerProvider(const std::string& provider_type, ProviderCreator creator) {
    if (provider_type.empty()) {
        throw std::invalid_argument("Provider type cannot be empty");
    }
    
    if (!creator) {
        throw std::invalid_argument("Provider creator cannot be null");
    }
    
    providers_[provider_type] = creator;
}

std::unique_ptr<AuthenticationProvider> AuthenticationProviderFactory::createProvider(const std::string& provider_type) { // 创建提供者
    auto it = providers_.find(provider_type);
    if (it == providers_.end()) {
        return nullptr;
    }
    
    try {
        return it->second();
    } catch (const std::exception& e) {
        // 记录错误但不抛出异常，返回nullptr表示创建失败
        return nullptr;
    }
}

std::vector<std::string> AuthenticationProviderFactory::getRegisteredTypes() const { // 获取注册的类型
    std::vector<std::string> types;
    types.reserve(providers_.size());
    
    for (const auto& pair : providers_) {
        types.push_back(pair.first);
    }
    
    return types;
}

bool AuthenticationProviderFactory::isProviderRegistered(const std::string& provider_type) const {
    return providers_.find(provider_type) != providers_.end();
}

} // namespace drone_control