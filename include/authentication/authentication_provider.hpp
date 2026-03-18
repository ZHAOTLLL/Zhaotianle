/**
 * 认证提供者接口
 * 访问控制引擎通过本接口进行身份认证；具体实现可为 X.509 证书、JWT 等。
 */
#pragma once

#include <string>
#include <map>

namespace drone_control {

/**
 * 认证请求结构体
 */
struct AuthenticationRequest {
    std::string credential;     // 认证凭据（证书、令牌等）
    std::string credential_type; // 凭据类型
    std::string drone_id;       // 无人机ID
    std::map<std::string, std::string> metadata; // 元数据
    
    AuthenticationRequest() = default;
    AuthenticationRequest(const std::string& cred, const std::string& type)
        : credential(cred), credential_type(type) {}
    AuthenticationRequest(const std::string& cred, const std::string& type, const std::string& id);
    
    // 元数据操作方法
    void addMetadata(const std::string& key, const std::string& value);
    std::string getMetadata(const std::string& key) const;
};

/**
 * 认证结果结构体
 */
struct AuthenticationResult {
    bool success;               // 认证是否成功
    std::string error_message;  // 错误信息
    std::map<std::string, std::string> attributes; // 提取的属性
    std::string trust_level;    // 信任级别
    
    AuthenticationResult() : success(false) {}
    AuthenticationResult(bool succ) : success(succ) {}
    AuthenticationResult(bool succ, const std::string& error);
    
    // 属性操作方法
    void addAttribute(const std::string& key, const std::string& value);
    std::string getAttribute(const std::string& key) const;
    void setTrustLevel(const std::string& level);
};

/** 认证提供者：执行认证、校验凭据、提取属性 */
class AuthenticationProvider {
public:
    virtual ~AuthenticationProvider() = default;
    
    /**
     * 执行身份认证
     * @param request 认证请求
     * @return 认证结果
     */
    virtual AuthenticationResult authenticate(const AuthenticationRequest& request) = 0;
    
    /**
     * 验证证书/令牌有效性
     * @param credential 认证凭据
     * @return 是否有效
     */
    virtual bool validateCredential(const std::string& credential) = 0;
    
    /**
     * 提取属性信息
     * @param credential 认证凭据
     * @return 属性映射
     */
    virtual std::map<std::string, std::string> extractAttributes(const std::string& credential) = 0;
    
    /**
     * 获取提供者类型
     * @return 提供者类型名称
     */
    virtual std::string getProviderType() const = 0;
};

} // namespace drone_control