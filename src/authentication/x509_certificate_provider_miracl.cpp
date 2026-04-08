/**
 * X.509 证书认证（Miracl Ed25519）
 * 使用 Miracl 与 Ed25519 曲线做证书解析与签名验证，与 DSS 方案统一。
 */
#include "../../include/authentication/x509_certificate_provider_miracl.hpp"
#include "../../include/authentication/authentication_provider_factory.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cctype>

// 包含ECPoint头文件（用于将Ed25519公钥转换为Miracl格式，像DSS方案一样）
#include "../../include/signature/dss_ms_signature.hpp"
#include <iomanip>
#include <vector>
#include <iostream>
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

namespace drone_control {

// 缓冲区大小定义
#define MAX_CERT_SIZE 5000
#define MAX_SIG_SIZE 512
#define MAX_KEY_SIZE 512

X509CertificateProviderMiracl::X509CertificateProviderMiracl() 
    : allow_self_signed_(true) {
}

X509CertificateProviderMiracl::~X509CertificateProviderMiracl() {
    clearTrustedCAs();
}

AuthenticationResult X509CertificateProviderMiracl::authenticate(const AuthenticationRequest& request) {
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
        // 需要从CA证书中提取公钥
        const auto& ca = trusted_cas_[ca_index];
        // 创建非const副本，因为Miracl函数不接受const参数
        char cert_data_copy[MAX_CERT_SIZE];
        octet cert_data_nonconst = {0, ca.cert_data.len, cert_data_copy};
        memcpy(cert_data_copy, ca.cert_data.val, ca.cert_data.len);
        cert_data_nonconst.len = ca.cert_data.len;
        
        char ca_pubkey_buffer[MAX_KEY_SIZE];
        octet ca_pubkey_octet = {0, sizeof(ca_pubkey_buffer), ca_pubkey_buffer};
        pktype ca_key_type = X509_extract_public_key(&cert_data_nonconst, &ca_pubkey_octet);
        
        if (ca_key_type.type == 0) {
            return AuthenticationResult(false, "Failed to extract CA public key");
        }
        
        signature_valid = verifyCertificateSignature(cert_data, sig_octet, sig_type,
                                                     ca_pubkey_octet, ca_key_type);
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
    
    // 【关键改进】将32字节原始Ed25519公钥转换为Miracl ECPoint格式（像DSS方案一样）
    // baseline方案现在优先使用public_key_ecpoint，不需要生成PEM格式，避免使用OpenSSL
    // 这样每次迭代的身份认证时间都会一致，不会有OpenSSL全局初始化的开销
    if (key_type.type == X509_ECD && key_type.curve == USE_ED25519 && pubkey_octet.len == 32) {
        try {
            // 从32字节原始Ed25519公钥创建ECPoint（使用33字节压缩格式）
            unsigned char x_parity_bit = static_cast<unsigned char>(pubkey_octet.val[31]) & 0x80;
            unsigned char prefix = (x_parity_bit != 0) ? 0x03 : 0x02;
            
            // 构建33字节压缩格式
            std::vector<unsigned char> compressed_pubkey(33);
            compressed_pubkey[0] = prefix;
            memcpy(compressed_pubkey.data() + 1, pubkey_octet.val, 32);
            
            // 创建ECPoint并序列化
            ECPoint uav_pk;
            octet compressed_oct = {0, 33, reinterpret_cast<char*>(compressed_pubkey.data())};
            int result = ECP_fromOctet((ECP*)&uav_pk.get(), &compressed_oct);
            
            if (result != 0) {
                // 序列化为Miracl格式（像DSS方案一样）
                std::string ecpoint_hex = uav_pk.serialize();
                attributes["public_key_ecpoint"] = ecpoint_hex;
            } else {
                std::cerr << "[X509CertificateProviderMiracl] 警告：无法将Ed25519公钥转换为ECPoint" << std::endl;
            }
        } catch (...) {
            std::cerr << "[X509CertificateProviderMiracl] 警告：转换Ed25519公钥为ECPoint时发生异常" << std::endl;
        }
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

bool X509CertificateProviderMiracl::validateCredential(const std::string& credential) {
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
        // 从CA证书中提取公钥
        // 创建非const副本，因为Miracl函数不接受const参数
        char cert_data_copy[MAX_CERT_SIZE];
        octet cert_data_nonconst = {0, ca.cert_data.len, cert_data_copy};
        memcpy(cert_data_copy, ca.cert_data.val, ca.cert_data.len);
        cert_data_nonconst.len = ca.cert_data.len;
        
        char ca_pubkey_buffer[MAX_KEY_SIZE];
        octet ca_pubkey_octet = {0, sizeof(ca_pubkey_buffer), ca_pubkey_buffer};
        pktype ca_key_type = X509_extract_public_key(&cert_data_nonconst, &ca_pubkey_octet);
        
        if (ca_key_type.type == 0) {
            return false;
        }
        
        return verifyCertificateSignature(cert_data, sig_octet, sig_type,
                                         ca_pubkey_octet, ca_key_type);
    }
}

std::map<std::string, std::string> X509CertificateProviderMiracl::extractAttributes(const std::string& credential) {
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
    
    // 【已更新】shared_pk现在直接从证书的Subject属性中提取（在extractSubjectAttributes中处理）
    // 不再需要从外部JSON文件读取
    
    return attributes;
}

std::string X509CertificateProviderMiracl::getProviderType() const {
    return "x509_certificate_miracl";
}

bool X509CertificateProviderMiracl::addTrustedCA(const std::string& ca_certificate) {
    TrustedCA ca;
    
    // 分配缓冲区
    char cert_buffer[MAX_CERT_SIZE];
    ca.cert_data = {0, sizeof(cert_buffer), cert_buffer};
    
    if (!parsePEMCertificate(ca_certificate, ca.cert_data)) {
        return false;
    }
    
    // 提取CA公钥
    char pubkey_buffer[MAX_KEY_SIZE];
    octet pubkey_octet = {0, sizeof(pubkey_buffer), pubkey_buffer};
    
    // 提取证书数据
    char cert_data_buffer[MAX_CERT_SIZE];
    octet cert_data = {0, sizeof(cert_data_buffer), cert_data_buffer};
    int cert_len = X509_extract_cert(&ca.cert_data, &cert_data);
    
    if (cert_len == 0) {
        return false;
    }
    
    ca.public_key = X509_extract_public_key(&cert_data, &pubkey_octet);
    
    if (ca.public_key.type == 0) {
        return false;
    }
    
    // 保存公钥（需要分配新的缓冲区）
    // 注意：这里我们保存证书数据而不是公钥，因为验证时需要完整的证书
    // 公钥信息已经在ca.public_key中保存了
    
    ca.key_type = ca.public_key.type;
    
    // 创建新的TrustedCA并复制数据
    TrustedCA new_ca;
    char new_cert_buffer[MAX_CERT_SIZE];
    new_ca.cert_data = {0, sizeof(new_cert_buffer), new_cert_buffer};
    OCT_copy(&new_ca.cert_data, &cert_data);  // 保存证书数据
    new_ca.public_key = ca.public_key;
    new_ca.key_type = ca.key_type;
    
    trusted_cas_.push_back(new_ca);
    return true;
}

bool X509CertificateProviderMiracl::loadTrustedCAFromFile(const std::string& ca_file_path) {
    // 【性能修复】每次读取文件前，先同步文件系统，确保从磁盘读取
    // 这样可以消除文件系统缓存的影响，确保每次迭代的文件读取时间一致
    std::ifstream file(ca_file_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // 禁用文件系统缓存：每次重新定位到文件开头
    file.seekg(0, std::ios::beg);
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string cert_data = buffer.str();
    file.close();
    
    return addTrustedCA(cert_data);
}

void X509CertificateProviderMiracl::clearTrustedCAs() {
    trusted_cas_.clear();
}

void X509CertificateProviderMiracl::setAllowSelfSigned(bool allow) {
    allow_self_signed_ = allow;
}

bool X509CertificateProviderMiracl::parsePEMCertificate(const std::string& pem_data, octet& cert_out) {
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

bool X509CertificateProviderMiracl::base64Decode(const std::string& encoded, octet& decoded) {
    OCT_frombase64(&decoded, const_cast<char*>(encoded.c_str()));
    return decoded.len > 0;
}

bool X509CertificateProviderMiracl::verifyCertificateSignature(const octet& cert, const octet& sig,
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
            return Ed25519::EDDSA_VERIFY(false, &ca_pubkey_nonconst, nullptr, &cert_nonconst, &sig_nonconst) != 0;
        }
        #endif
        #ifdef USE_ED448
        if (ca_key_type.curve == USE_ED448) {
            return Ed448::EDDSA_VERIFY(false, &ca_pubkey_nonconst, nullptr, &cert_nonconst, &sig_nonconst) != 0;
        }
        #endif
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

int X509CertificateProviderMiracl::findMatchingCA(const octet& cert) {
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

    for (size_t i = 0; i < trusted_cas_.size(); ++i) {
        const auto& ca = trusted_cas_[i];
        if (ca.cert_data.len <= 0 || ca.cert_data.len > MAX_CERT_SIZE) {
            continue;
        }

        char ca_cert_copy[MAX_CERT_SIZE];
        octet ca_cert_nonconst = {0, ca.cert_data.len, ca_cert_copy};
        memcpy(ca_cert_copy, ca.cert_data.val, ca.cert_data.len);
        ca_cert_nonconst.len = ca.cert_data.len;

        int subject_ptr = 0;
        int subject_len = 0;
        subject_ptr = X509_find_subject(&ca_cert_nonconst, &subject_len);
        if (subject_ptr == 0 || subject_len <= 0) {
            continue;
        }

        if (issuer_len == subject_len &&
            issuer_ptr + issuer_len <= cert.len &&
            subject_ptr + subject_len <= ca.cert_data.len &&
            memcmp(cert.val + issuer_ptr, ca.cert_data.val + subject_ptr, issuer_len) == 0) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

std::map<std::string, std::string> X509CertificateProviderMiracl::extractSubjectAttributes(const octet& cert) {
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
    if (!email.empty()) {
        attributes["email"] = email;
        
        // 【新增】从emailAddress字段提取shared_pk（格式：shared_pk@{hex_string}）
        if (email.find("shared_pk@") == 0) {
            std::string shared_pk_hex = email.substr(10);  // 去掉"shared_pk@"前缀
            if (shared_pk_hex.length() == 66) {  // 33字节 = 66十六进制字符
                attributes["shared_pk"] = shared_pk_hex;
                std::cout << "[X509CertificateProvider] 从证书Subject提取shared_pk: " << shared_pk_hex.substr(0, 20) << "..." << std::endl;
            }
        }
    }
    
    // 映射到系统使用的属性名
    if (!o.empty()) {
        attributes["organization"] = o;
    }
    if (!ou.empty()) {
        attributes["role"] = ou;
    }
    
    return attributes;
}

bool X509CertificateProviderMiracl::extractPublicKey(const octet& cert, octet& pubkey_out, pktype& key_type_out) {
    // 创建非const副本，因为Miracl函数不接受const参数
    char cert_copy[MAX_CERT_SIZE];
    octet cert_nonconst = {0, cert.len, cert_copy};
    memcpy(cert_copy, cert.val, cert.len);
    cert_nonconst.len = cert.len;
    
    key_type_out = X509_extract_public_key(&cert_nonconst, &pubkey_out);
    return key_type_out.type != 0;
}

std::string X509CertificateProviderMiracl::octetToHex(const octet& data) {
    std::string hex;
    hex.reserve(data.len * 2);
    
    for (int i = 0; i < data.len; i++) {
        char hex_byte[3];
        snprintf(hex_byte, sizeof(hex_byte), "%02x", data.val[i]);
        hex += hex_byte;
    }
    
    return hex;
}

std::string X509CertificateProviderMiracl::extractEntityProperty(const octet& cert, int section_ptr, const octet& property_oid) {
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
REGISTER_AUTHENTICATION_PROVIDER(X509CertificateProviderMiracl, "x509_certificate_miracl");

} // namespace drone_control

