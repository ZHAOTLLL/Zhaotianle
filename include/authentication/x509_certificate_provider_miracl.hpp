/**
 * x509_certificate_provider_miracl.hpp - X.509 证书认证（Miracl）
 *        使用Miracl库进行证书解析、签名验证和属性提取，与FCSACM*方案统一使用同一密码学库
 */
#pragma once

#include "authentication_provider.hpp"
#include <vector>
#include <memory>
#include <string>

// Miracl库头文件
#include "core.h"
#include "x509.h"

// 根据编译配置选择支持的曲线和算法
#ifdef USE_NIST256
#include "ecdh_NIST256.h"
#endif
#ifdef USE_ED25519
#include "eddsa_Ed25519.h"
#endif
#ifdef USE_ED448
#include "eddsa_Ed448.h"
#endif
#ifdef USE_RSA2048
#include "rsa_RSA2048.h"
#endif

namespace drone_control {

/**
 * 基于Miracl库的X.509证书认证提供者
 * 提供基本的证书验证和属性提取功能
 */
class X509CertificateProviderMiracl : public AuthenticationProvider {
public:
    X509CertificateProviderMiracl();
    ~X509CertificateProviderMiracl() override;
    
    // AuthenticationProvider接口实现
    AuthenticationResult authenticate(const AuthenticationRequest& request) override;
    bool validateCredential(const std::string& credential) override;
    std::map<std::string, std::string> extractAttributes(const std::string& credential) override;
    std::string getProviderType() const override;
    
    // X.509特定方法
    /**
     * 添加可信CA证书
     * @param ca_certificate PEM格式的CA证书（base64编码）
     * @return 是否添加成功
     */
    bool addTrustedCA(const std::string& ca_certificate);
    
    /**
     * 从文件加载可信CA证书
     * @param ca_file_path CA证书文件路径
     * @return 是否加载成功
     */
    bool loadTrustedCAFromFile(const std::string& ca_file_path);
    
    /**
     * 清除所有可信CA
     */
    void clearTrustedCAs();
    
    /**
     * 设置是否允许自签名证书
     * @param allow 是否允许
     */
    void setAllowSelfSigned(bool allow);

private:
    struct TrustedCA {
        octet cert_data;      // 证书数据
        pktype public_key;    // CA公钥
        int key_type;         // 密钥类型
    };
    
    std::vector<TrustedCA> trusted_cas_;  // 可信CA列表
    bool allow_self_signed_;              // 是否允许自签名证书
    
    /**
     * 解析PEM格式证书（从base64解码）
     * @param pem_data PEM格式证书数据（可能包含PEM头尾）
     * @param cert_out 输出的证书数据（octet）
     * @return 是否解析成功
     */
    bool parsePEMCertificate(const std::string& pem_data, octet& cert_out);
    
    /**
     * Base64解码
     * @param encoded base64编码的字符串
     * @param decoded 输出的解码数据
     * @return 是否解码成功
     */
    bool base64Decode(const std::string& encoded, octet& decoded);
    
    /**
     * 验证证书签名
     * @param cert 证书数据
     * @param sig 签名数据
     * @param sig_type 签名类型
     * @param ca_pubkey CA公钥
     * @return 是否验证成功
     */
    bool verifyCertificateSignature(const octet& cert, const octet& sig, 
                                    const pktype& sig_type, const octet& ca_pubkey, 
                                    const pktype& ca_key_type);
    
    /**
     * 查找匹配的可信CA
     * @param cert 证书数据
     * @return 匹配的CA索引，-1表示未找到
     */
    int findMatchingCA(const octet& cert);
    
    /**
     * 从证书中提取主题属性
     * @param cert 证书数据
     * @return 主题属性映射
     */
    std::map<std::string, std::string> extractSubjectAttributes(const octet& cert);
    
    /**
     * 从证书中提取公钥
     * @param cert 证书数据
     * @param pubkey_out 输出的公钥
     * @param key_type_out 输出的密钥类型
     * @return 是否提取成功
     */
    bool extractPublicKey(const octet& cert, octet& pubkey_out, pktype& key_type_out);
    
    /**
     * 将octet转换为十六进制字符串（用于公钥存储）
     * @param data octet数据
     * @return 十六进制字符串
     */
    std::string octetToHex(const octet& data);
    
    /**
     * 从证书中提取实体属性
     * @param cert 证书数据
     * @param section_ptr 实体部分指针
     * @param property_oid 属性OID
     * @return 属性值，空字符串表示未找到
     */
    std::string extractEntityProperty(const octet& cert, int section_ptr, const octet& property_oid);
};

} // namespace drone_control





