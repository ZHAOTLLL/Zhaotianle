/**
 * 认证提供者基类与认证请求/结果辅助方法
 * AuthenticationRequest、AuthenticationResult 的构造与属性存取实现。
 */
#include "../../include/authentication/authentication_provider.hpp"
#include <iostream>

namespace drone_control {

AuthenticationRequest::AuthenticationRequest(const std::string& cred, const std::string& type, const std::string& id)
    : credential(cred), credential_type(type), drone_id(id) {
}

void AuthenticationRequest::addMetadata(const std::string& key, const std::string& value) {
    metadata[key] = value;
}

std::string AuthenticationRequest::getMetadata(const std::string& key) const {
    auto it = metadata.find(key);
    return (it != metadata.end()) ? it->second : "";
}

// AuthenticationResult utility methods
AuthenticationResult::AuthenticationResult(bool succ, const std::string& error)
    : success(succ), error_message(error) {
}

void AuthenticationResult::addAttribute(const std::string& key, const std::string& value) {
    attributes[key] = value;
}

std::string AuthenticationResult::getAttribute(const std::string& key) const {
    auto it = attributes.find(key);
    return (it != attributes.end()) ? it->second : "";
}

void AuthenticationResult::setTrustLevel(const std::string& level) {
    trust_level = level;
}

} // namespace drone_control