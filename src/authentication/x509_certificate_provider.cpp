/**
 * X.509 证书认证提供者
 * 使用 OpenSSL 解析证书、校验签名并提取属性，供访问控制身份认证使用。
 */
#include "../../include/authentication/x509_certificate_provider.hpp"
#include "../../include/authentication/authentication_provider_factory.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cctype>
#include <iostream>

// OpenSSL头文件（用于提取PEM格式公钥，与OpenSSL版本的X.509提供者保持一致）
// 注意：Miracl库的core.h定义了SHA256, SHA384, SHA512等宏，会与OpenSSL的函数名冲突
// 需要先保存这些宏的值，然后取消定义，包含OpenSSL头文件后再恢复
#ifdef SHA256
#define MIRACL_SHA256 SHA256
#undef SHA256
#endif
#ifdef SHA384
#define MIRACL_SHA384 SHA384
#undef SHA384
#endif
#ifdef SHA512
#define MIRACL_SHA512 SHA512
#undef SHA512
#endif

#include <openssl/x509.h>
#include <openssl/pem.h>
#include <openssl/evp.h>
#include <openssl/bio.h>

// 恢复Miracl的宏定义
#ifdef MIRACL_SHA256
#define SHA256 MIRACL_SHA256
#undef MIRACL_SHA256
#endif
#ifdef MIRACL_SHA384
#define SHA384 MIRACL_SHA384
#undef MIRACL_SHA384
#endif
#ifdef MIRACL_SHA512
#define SHA512 MIRACL_SHA512
#undef MIRACL_SHA512
#endif

namespace drone_control {

// 缓冲区大小定义
#define MAX_CERT_SIZE 5000
#define MAX_SIG_SIZE 512
#define MAX_KEY_SIZE 512

X509CertificateProvider::X509CertificateProvider() 
    : allow_self_signed_(true) {
}

X509CertificateProvider::~X509CertificateProvider() {
    clearTrustedCAs();
}

AuthenticationResult X509CertificateProvider::authenticate(const AuthenticationRequest& request) {
    if (request.credential.empty()) {
        return AuthenticationResult(false, "Empty certificate provided");
    }
    
    // 解析证书
    char cert_buffer[MAX_CERT_SIZE];
    octet cert_octet = {0, sizeof(cert_buffer), cert_buffer};
    
    if (!parsePEMCertificate(request.credential, cert_octet)) {
        return AuthenticationResult(false, "Failed to parse certificate");
    }
    
    // 提取证书签名
    char sig_buffer[MAX_SIG_SIZE];
    octet sig_octet = {0, sizeof(sig_buffer), sig_buffer};
    pktype sig_type = X509_extract_cert_sig(&cert_octet, &sig_octet);
    
    if (sig_type.type == 0) {
        return AuthenticationResult(false, "Failed to extract certificate signature");
    }
    
    // 提取证书本身（从签名证书中）
    char cert_data_buffer[MAX_CERT_SIZE];
    octet cert_data = {0, sizeof(cert_data_buffer), cert_data_buffer};
    int cert_len = X509_extract_cert(&cert_octet, &cert_data);
    
    if (cert_len == 0) {
        return AuthenticationResult(false, "Failed to extract certificate data");
    }
    
    // 提取公钥
    char pubkey_buffer[MAX_KEY_SIZE];
    octet pubkey_octet = {0, sizeof(pubkey_buffer), pubkey_buffer};
    pktype key_type = X509_extract_public_key(&cert_data, &pubkey_octet);
    
    if (key_type.type == 0) {
        return AuthenticationResult(false, "Failed to extract public key");
    }
    
    // 验证证书签名
    bool signature_valid = false;
    
    // 检查是否是自签名证书
    bool is_self_signed = X509_self_signed(&cert_data) != 0;
    
    if (is_self_signed) {
        if (!allow_self_signed_) {
            return AuthenticationResult(false, "Self-signed certificate not allowed");
        }
        // 验证自签名：使用证书自己的公钥验证签名
        signature_valid = verifyCertificateSignature(cert_data, sig_octet, sig_type, 
                                                     pubkey_octet, key_type);
    } else {
        // 查找匹配的可信CA
        int ca_index = findMatchingCA(cert_data);
        if (ca_index < 0) {
            return AuthenticationResult(false, "No matching trusted CA found");
        }
        
        // 使用CA公钥验证签名
        // 直接使用保存的公钥数据，避免重新提取
        const auto& ca = trusted_cas_[ca_index];
        // 创建非const副本，因为Miracl函数不接受const参数
        char ca_pubkey_copy[MAX_KEY_SIZE];
        octet ca_pubkey_octet = {0, ca.public_key_data.len, ca_pubkey_copy};
        memcpy(ca_pubkey_copy, ca.public_key_data.val, ca.public_key_data.len);
        ca_pubkey_octet.len = ca.public_key_data.len;
        
        signature_valid = verifyCertificateSignature(cert_data, sig_octet, sig_type,
                                                     ca_pubkey_octet, ca.public_key);
    }
    
    if (!signature_valid) {
        return AuthenticationResult(false, "Certificate signature verification failed");
    }
    
    // 提取属性
    auto attributes = extractAttributes(request.credential);
    
    // 添加密钥类型信息
    std::string cert_key_type = "unknown";
    if (key_type.type == X509_ECC) {
        cert_key_type = "ECC";
    } else if (key_type.type == X509_RSA) {
        cert_key_type = "RSA";
    } else if (key_type.type == X509_ECD) {
        cert_key_type = "EdDSA";
    }
    
    attributes["certificate_key_type"] = cert_key_type;
    attributes["credential_type"] = cert_key_type;
    
    // 添加公钥（十六进制格式，用于地理围栏解锁凭证）
    attributes["public_key"] = octetToHex(pubkey_octet);
    attributes["public_key_type"] = cert_key_type;
    
    // 从Miracl提取的原始公钥数据构造PEM格式（用于地理围栏解锁凭证生成）
    // 避免重新解析证书，直接使用已提取的公钥数据
    std::string public_key_pem;
    
    // 对于Ed25519，直接从32字节原始公钥构造PEM格式
    #ifdef EVP_PKEY_ED25519
    if (key_type.type == X509_ECD && key_type.curve == USE_ED25519 && pubkey_octet.len == 32) {
        // 使用OpenSSL从32字节原始公钥创建EVP_PKEY
        EVP_PKEY* pkey = EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519, nullptr, 
                                                      reinterpret_cast<const unsigned char*>(pubkey_octet.val), 
                                                      pubkey_octet.len);
        if (pkey) {
            BIO* pubkey_bio = BIO_new(BIO_s_mem());
            if (pubkey_bio) {
                if (PEM_write_bio_PUBKEY(pubkey_bio, pkey)) {
                    char* pubkey_data = nullptr;
                    long pubkey_len = BIO_get_mem_data(pubkey_bio, &pubkey_data);
                    if (pubkey_data && pubkey_len > 0) {
                        public_key_pem = std::string(pubkey_data, pubkey_len);
                    }
                }
                BIO_free(pubkey_bio);
            }
            EVP_PKEY_free(pkey);
        }
    }
    #endif
    
    // 对于其他密钥类型（RSA、ECC等），如果PEM格式尚未构造，则使用OpenSSL从证书中提取
    // 注意：这应该很少发生，因为Ed25519是主要使用的类型
    if (public_key_pem.empty()) {
        BIO* cert_bio = BIO_new_mem_buf(request.credential.c_str(), -1);
        if (cert_bio) {
            X509* cert = PEM_read_bio_X509(cert_bio, nullptr, nullptr, nullptr);
            BIO_free(cert_bio);
            
            if (cert) {
                EVP_PKEY* pubkey = X509_get_pubkey(cert);
                if (pubkey) {
                    BIO* pubkey_bio = BIO_new(BIO_s_mem());
                    if (pubkey_bio) {
                        if (PEM_write_bio_PUBKEY(pubkey_bio, pubkey)) {
                            char* pubkey_data = nullptr;
                            long pubkey_len = BIO_get_mem_data(pubkey_bio, &pubkey_data);
                            if (pubkey_data && pubkey_len > 0) {
                                public_key_pem = std::string(pubkey_data, pubkey_len);
                            }
                        }
                        BIO_free(pubkey_bio);
                    }
                    EVP_PKEY_free(pubkey);
                }
                X509_free(cert);
            }
        }
    }
    
    // 如果成功构造了PEM格式公钥，添加到属性中
    if (!public_key_pem.empty()) {
        attributes["public_key_pem"] = public_key_pem;
    }
    
    // 创建成功结果
    AuthenticationResult result(true);
    result.attributes = attributes;
    
    // 设置信任级别
    std::string org = result.getAttribute("organization");
    if (org == "government" || org == "police" || org == "emergency") {
        result.setTrustLevel("high");
    } else if (org == "commercial" || org == "delivery") {
        result.setTrustLevel("medium");
    } else {
        result.setTrustLevel("low");
    }
    
    return result;
}

bool X509CertificateProvider::validateCredential(const std::string& credential) {
    if (credential.empty()) {
        return false;
    }
    
    char cert_buffer[MAX_CERT_SIZE];
    octet cert_octet = {0, sizeof(cert_buffer), cert_buffer};
    
    if (!parsePEMCertificate(credential, cert_octet)) {
        return false;
    }
    
    // 提取签名
    char sig_buffer[MAX_SIG_SIZE];
    octet sig_octet = {0, sizeof(sig_buffer), sig_buffer};
    pktype sig_type = X509_extract_cert_sig(&cert_octet, &sig_octet);
    
    if (sig_type.type == 0) {
        return false;
    }
    
    // 提取证书数据
    char cert_data_buffer[MAX_CERT_SIZE];
    octet cert_data = {0, sizeof(cert_data_buffer), cert_data_buffer};
    int cert_len = X509_extract_cert(&cert_octet, &cert_data);
    
    if (cert_len == 0) {
        return false;
    }
    
    // 提取公钥
    char pubkey_buffer[MAX_KEY_SIZE];
    octet pubkey_octet = {0, sizeof(pubkey_buffer), pubkey_buffer};
    pktype key_type = X509_extract_public_key(&cert_data, &pubkey_octet);
    
    if (key_type.type == 0) {
        return false;
    }
    
    // 验证签名
    bool is_self_signed = X509_self_signed(&cert_data) != 0;
    
    if (is_self_signed) {
        if (!allow_self_signed_) {
            return false;
        }
        return verifyCertificateSignature(cert_data, sig_octet, sig_type, 
                                         pubkey_octet, key_type);
    } else {
        int ca_index = findMatchingCA(cert_data);
        if (ca_index < 0) {
            return false;
        }
        const auto& ca = trusted_cas_[ca_index];
        // 直接使用保存的公钥数据，避免重新提取
        // 创建非const副本，因为Miracl函数不接受const参数
        char ca_pubkey_copy[MAX_KEY_SIZE];
        octet ca_pubkey_octet = {0, ca.public_key_data.len, ca_pubkey_copy};
        memcpy(ca_pubkey_copy, ca.public_key_data.val, ca.public_key_data.len);
        ca_pubkey_octet.len = ca.public_key_data.len;
        
        return verifyCertificateSignature(cert_data, sig_octet, sig_type,
                                         ca_pubkey_octet, ca.public_key);
    }
}

std::map<std::string, std::string> X509CertificateProvider::extractAttributes(const std::string& credential) {
    std::map<std::string, std::string> attributes;
    
    char cert_buffer[MAX_CERT_SIZE];
    octet cert_octet = {0, sizeof(cert_buffer), cert_buffer};
    
    if (!parsePEMCertificate(credential, cert_octet)) {
        return attributes;
    }
    
    // 提取证书数据
    char cert_data_buffer[MAX_CERT_SIZE];
    octet cert_data = {0, sizeof(cert_data_buffer), cert_data_buffer};
    int cert_len = X509_extract_cert(&cert_octet, &cert_data);
    
    if (cert_len == 0) {
        return attributes;
    }
    
    // 提取主题属性
    auto subject_attrs = extractSubjectAttributes(cert_data);
    attributes.insert(subject_attrs.begin(), subject_attrs.end());
    
    return attributes;
}

std::string X509CertificateProvider::getProviderType() const {
    return "x509_certificate";
}

bool X509CertificateProvider::addTrustedCA(const std::string& ca_certificate) {
    // 分配缓冲区
    char cert_buffer[MAX_CERT_SIZE];
    octet cert_octet = {0, sizeof(cert_buffer), cert_buffer};
    
    if (!parsePEMCertificate(ca_certificate, cert_octet)) {
        std::cerr << "[X509CertificateProvider] Failed to parse CA certificate (PEM decode failed)" << std::endl;
        return false;
    }
    
    // 从完整的签名证书中提取证书数据（不含签名）
    char cert_data_buffer[MAX_CERT_SIZE];
    octet cert_data = {0, sizeof(cert_data_buffer), cert_data_buffer};
    int cert_len = X509_extract_cert(&cert_octet, &cert_data);
    
    if (cert_len == 0) {
        std::cerr << "[X509CertificateProvider] Failed to extract certificate data from CA cert" << std::endl;
        return false;
    }
    
    // 从提取的证书数据中提取CA公钥（X509_extract_public_key需要证书主体部分）
    char pubkey_buffer[MAX_KEY_SIZE];
    octet pubkey_octet = {0, sizeof(pubkey_buffer), pubkey_buffer};
    pktype public_key_type = X509_extract_public_key(&cert_data, &pubkey_octet);
    
    if (public_key_type.type == 0) {
        std::cerr << "[X509CertificateProvider] Failed to extract public key from CA certificate" << std::endl;
        return false;
    }
    
    // 创建新的TrustedCA并保存提取的证书数据（不含签名）
    // 注意：保存提取出的证书数据（cert_data），这是证书的主体部分，不含签名
    // X509_extract_public_key需要证书主体部分，而不是完整的签名证书
    TrustedCA new_ca;
    char new_cert_buffer[MAX_CERT_SIZE];
    new_ca.cert_data = {0, sizeof(new_cert_buffer), new_cert_buffer};
    OCT_copy(&new_ca.cert_data, &cert_data);  // 保存证书数据（不含签名）
    
    // 保存公钥数据，避免在验证时重新提取
    char new_pubkey_buffer[MAX_KEY_SIZE];
    new_ca.public_key_data = {0, sizeof(new_pubkey_buffer), new_pubkey_buffer};
    OCT_copy(&new_ca.public_key_data, &pubkey_octet);  // 保存公钥数据
    
    new_ca.public_key = public_key_type;
    new_ca.key_type = public_key_type.type;
    
    std::cout << "[X509CertificateProvider] CA certificate loaded successfully (type=" 
              << public_key_type.type << ", curve=" << public_key_type.curve << ", len=" << cert_data.len << ")" << std::endl;
    
    trusted_cas_.push_back(new_ca);
    return true;
}

bool X509CertificateProvider::loadTrustedCAFromFile(const std::string& ca_file_path) {
    std::ifstream file(ca_file_path);
    if (!file.is_open()) {
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string cert_data = buffer.str();
    file.close();
    
    return addTrustedCA(cert_data);
}

void X509CertificateProvider::clearTrustedCAs() {
    trusted_cas_.clear();
}

void X509CertificateProvider::setAllowSelfSigned(bool allow) {
    allow_self_signed_ = allow;
}

bool X509CertificateProvider::parsePEMCertificate(const std::string& pem_data, octet& cert_out) {
    std::string base64_data = pem_data;
    
    // 移除PEM头尾标记
    size_t begin_pos = base64_data.find("-----BEGIN");
    if (begin_pos != std::string::npos) {
        size_t end_line = base64_data.find('\n', begin_pos);
        if (end_line != std::string::npos) {
            base64_data = base64_data.substr(end_line + 1);
        }
    }
    
    size_t end_pos = base64_data.find("-----END");
    if (end_pos != std::string::npos) {
        base64_data = base64_data.substr(0, end_pos);
    }
    
    // 移除空白字符
    base64_data.erase(std::remove_if(base64_data.begin(), base64_data.end(),
                                     [](char c) { return std::isspace(c); }),
                     base64_data.end());
    
    // 处理JSON转义的换行符
    size_t pos = 0;
    while ((pos = base64_data.find("\\n", pos)) != std::string::npos) {
        base64_data.erase(pos, 2);
    }
    
    // Base64解码
    OCT_frombase64(&cert_out, const_cast<char*>(base64_data.c_str()));
    
    return cert_out.len > 0;
}

bool X509CertificateProvider::base64Decode(const std::string& encoded, octet& decoded) {
    OCT_frombase64(&decoded, const_cast<char*>(encoded.c_str()));
    return decoded.len > 0;
}

bool X509CertificateProvider::verifyCertificateSignature(const octet& cert, const octet& sig,
                                                               const pktype& sig_type, const octet& ca_pubkey,
                                                               const pktype& ca_key_type) {
    // 根据签名类型进行验证
    if (sig_type.type == X509_ECC) {
        // ECDSA签名验证
        #if defined(USE_NIST256) && defined(HAVE_NIST256)
        if (ca_key_type.curve == USE_NIST256) {
            // 分离r和s
            char r[64], s[64];  // NIST256需要32字节，使用64字节缓冲区
            octet R = {0, sizeof(r), r};
            octet S = {0, sizeof(s), s};
            
            
            OCT_chop(&sig, &S, sig.len / 2);
            OCT_copy(&R, &sig);
            
            int sha = 0;
            if (sig_type.hash == X509_H256) sha = SHA256;
            else if (sig_type.hash == X509_H384) sha = SHA384;
            else if (sig_type.hash == X509_H512) sha = SHA512;
            
            if (sha == 0) return false;
            
            return NIST256::ECP_VP_DSA(sha, &ca_pubkey, &cert, &R, &S) == 0;
        }
        #endif
        return false;
    } else if (sig_type.type == X509_ECD) {
        // EdDSA签名验证
        // 创建非const副本，因为Miracl函数不接受const参数
        char ca_pubkey_copy[MAX_KEY_SIZE];
        octet ca_pubkey_nonconst = {0, ca_pubkey.len, ca_pubkey_copy};
        memcpy(ca_pubkey_copy, ca_pubkey.val, ca_pubkey.len);
        ca_pubkey_nonconst.len = ca_pubkey.len;
        
        char cert_copy[MAX_CERT_SIZE];
        octet cert_nonconst = {0, cert.len, cert_copy};
        memcpy(cert_copy, cert.val, cert.len);
        cert_nonconst.len = cert.len;
        
        char sig_copy[MAX_KEY_SIZE];
        octet sig_nonconst = {0, sig.len, sig_copy};
        memcpy(sig_copy, sig.val, sig.len);
        sig_nonconst.len = sig.len;
        
        #ifdef USE_ED25519
        if (ca_key_type.curve == USE_ED25519) {
            std::cout << "[verifyCertificateSignature-Ed25519] 开始验证Ed25519签名..." << std::endl;
            std::cout << "  CA公钥长度: " << ca_pubkey_nonconst.len << " 字节" << std::endl;
            std::cout << "  证书长度: " << cert_nonconst.len << " 字节" << std::endl;
            std::cout << "  签名长度: " << sig_nonconst.len << " 字节" << std::endl;
            // Ed25519::EDDSA_VERIFY返回true表示验证成功，false表示失败
            bool verify_result = Ed25519::EDDSA_VERIFY(false, &ca_pubkey_nonconst, nullptr, &cert_nonconst, &sig_nonconst);
            std::cout << "  Ed25519签名验证结果: " << (verify_result ? "成功" : "失败") << std::endl;
            return verify_result;
        }
        #endif
        // 注意：我们只使用 Ed25519，不支持 Ed448
        return false;
    } else if (sig_type.type == X509_RSA) {
        // RSA签名验证
        #if defined(USE_RSA2048) && defined(HAVE_RSA2048)
        if (ca_key_type.curve == 2048) {
            RSA2048::rsa_public_key PK;
            PK.e = 65537; // 标准RSA公钥指数
            RSA2048::RSA_fromOctet(PK.n, &ca_pubkey);
            
            int sha = 0;
            if (sig_type.hash == X509_H256) sha = SHA256;
            else if (sig_type.hash == X509_H384) sha = SHA384;
            else if (sig_type.hash == X509_H512) sha = SHA512;
            
            if (sha == 0) return false;
            
            char hp[256];  // RSA2048的RFS是256字节
            octet HP = {0, sizeof(hp), hp};
            core::PKCS15(sha, &cert, &HP);
            
            char hh[256];
            octet HH = {0, sizeof(hh), hh};
            RSA2048::RSA_ENCRYPT(&PK, &sig, &HH);
            
            return OCT_comp(&HP, &HH) != 0;
        }
        #endif
        return false;
    }
    
    return false;
}

int X509CertificateProvider::findMatchingCA(const octet& cert) {
    // 简化实现：检查证书的issuer是否匹配CA的subject
    // 这里使用简单的匹配策略，实际应该比较完整的DN
    // 创建非const副本，因为Miracl函数不接受const参数
    char cert_copy[MAX_CERT_SIZE];
    octet cert_nonconst = {0, cert.len, cert_copy};
    memcpy(cert_copy, cert.val, cert.len);
    cert_nonconst.len = cert.len;
    
    int issuer_ptr, issuer_len;
    issuer_ptr = X509_find_issuer(&cert_nonconst, &issuer_len);
    
    if (issuer_ptr == 0) {
        return -1;
    }
    
    // 简化：返回第一个CA（实际应该进行完整的DN匹配）
    // TODO: 实现完整的DN匹配逻辑
    if (!trusted_cas_.empty()) {
        return 0;
    }
    
    return -1;
}

std::map<std::string, std::string> X509CertificateProvider::extractSubjectAttributes(const octet& cert) {
    std::map<std::string, std::string> attributes;
    
    // 创建非const副本，因为Miracl函数不接受const参数
    char cert_copy[MAX_CERT_SIZE];
    octet cert_nonconst = {0, cert.len, cert_copy};
    memcpy(cert_copy, cert.val, cert.len);
    cert_nonconst.len = cert.len;
    
    int subject_ptr, subject_len;
    subject_ptr = X509_find_subject(&cert_nonconst, &subject_len);
    
    if (subject_ptr == 0) {
        return attributes;
    }
    
    // 提取常用属性
    std::string cn = extractEntityProperty(cert, subject_ptr, X509_MN); // Common Name
    std::string ou = extractEntityProperty(cert, subject_ptr, X509_UN); // Organizational Unit
    std::string o = extractEntityProperty(cert, subject_ptr, X509_ON);  // Organization
    std::string c = extractEntityProperty(cert, subject_ptr, X509_CN);  // Country
    std::string email = extractEntityProperty(cert, subject_ptr, X509_EN); // Email
    
    if (!cn.empty()) attributes["common_name"] = cn;
    if (!ou.empty()) attributes["organizational_unit"] = ou;
    if (!o.empty()) attributes["organization"] = o;
    if (!c.empty()) attributes["country"] = c;
    if (!email.empty()) attributes["email"] = email;
    
    // 映射到系统使用的属性名
    if (!o.empty()) {
        attributes["organization"] = o;
    }
    if (!ou.empty()) {
        attributes["role"] = ou;
    }
    
    return attributes;
}

bool X509CertificateProvider::extractPublicKey(const octet& cert, octet& pubkey_out, pktype& key_type_out) {
    // 创建非const副本，因为Miracl函数不接受const参数
    char cert_copy[MAX_CERT_SIZE];
    octet cert_nonconst = {0, cert.len, cert_copy};
    memcpy(cert_copy, cert.val, cert.len);
    cert_nonconst.len = cert.len;
    
    key_type_out = X509_extract_public_key(&cert_nonconst, &pubkey_out);
    return key_type_out.type != 0;
}

std::string X509CertificateProvider::octetToHex(const octet& data) {
    std::string hex;
    hex.reserve(data.len * 2);
    
    for (int i = 0; i < data.len; i++) {
        char hex_byte[3];
        snprintf(hex_byte, sizeof(hex_byte), "%02x", data.val[i]);
        hex += hex_byte;
    }
    
    return hex;
}

std::string X509CertificateProvider::extractEntityProperty(const octet& cert, int section_ptr, const octet& property_oid) {
    // 创建非const副本，因为Miracl函数不接受const参数
    char cert_copy[MAX_CERT_SIZE];
    octet cert_nonconst = {0, cert.len, cert_copy};
    memcpy(cert_copy, cert.val, cert.len);
    cert_nonconst.len = cert.len;
    
    char oid_copy[64];
    octet oid_nonconst = {0, property_oid.len, oid_copy};
    memcpy(oid_copy, property_oid.val, property_oid.len);
    oid_nonconst.len = property_oid.len;
    
    int prop_len;
    int prop_ptr = X509_find_entity_property(&cert_nonconst, &oid_nonconst, section_ptr, &prop_len);
    
    if (prop_ptr == 0 || prop_len == 0) {
        return "";
    }
    
    return std::string(reinterpret_cast<const char*>(&cert.val[prop_ptr]), prop_len);
}

// 注册提供者
REGISTER_AUTHENTICATION_PROVIDER(X509CertificateProvider, "x509_certificate");

} // namespace drone_control

