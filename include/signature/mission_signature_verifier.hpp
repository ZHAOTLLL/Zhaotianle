#pragma once

#include "signature/dss_ms_signature.hpp"
#include <string>
#include <map>
#include <optional>
#include <gmp.h>
#include <gmpxx.h>

namespace drone_control {

/**
 * GCS端任务签名验证服务
 * 
 * 职责：
 * 1. 读取并解析清洗后的任务签名文件（JSON格式）
 * 2. 验证DSS-MS签名
 * 3. 从签名中提取任务属性（不可清洗部分）
 * 4. 返回提取的属性供访问控制引擎使用
 */
class MissionSignatureVerifier {
public:
    /**
     * 从签名中提取的任务属性
     */
    struct MissionAttributes {
        std::string mission_id;
        uint32_t drone_id;
        std::string operation_type;
        std::string target_alias;
        uint64_t timestamp;
        std::string company_cert_hash;
        
        MissionAttributes() : drone_id(0), timestamp(0) {}
    };
    
    /**
     * 验证结果
     */
    struct VerificationResult {
        bool is_valid;
        std::string error_message;
        MissionAttributes attributes;
        ECPoint sanitizer_pk;  // UAV的清洗者公钥（从DSS签名中提取，可用于加密）
        
        VerificationResult() : is_valid(false) {}
    };
    
    MissionSignatureVerifier();
    ~MissionSignatureVerifier();
    
    /**
     * 从JSON文件读取并验证任务签名
     * @param json_file_path JSON签名文件路径
     * @return 验证结果，包含是否有效和提取的属性
     */
    VerificationResult verifyFromFile(const std::string& json_file_path);
    
    /**
     * 从JSON字符串验证任务签名
     * @param json_content JSON字符串内容
     * @return 验证结果
     */
    VerificationResult verifyFromJson(const std::string& json_content);
    
    /**
     * 初始化DSS-MS系统（如果需要独立的参数）
     * 注意：如果使用相同的公共参数，可以重用TA的Setup结果
     */
    bool initialize();
    
private:
    /**
     * 解析JSON签名文件
     */
    bool parseJsonSignature(const std::string& json_content,
                           MissionAttributes& mission,
                           DSSMSSignature::Signature& signature,
                           ECPoint& company_pk,
                           ECPoint& sanitizer_pk);
    
    /**
     * 从十六进制字符串创建mpz_class
     */
    mpz_class hexStringToMPZ(const std::string& hex_str);
    
    /**
     * 从十六进制字符串创建ECPoint（压缩格式）
     */
    bool hexStringToECPoint(const std::string& hex_str, ECPoint& point);
    
    /**
     * 从任务消息构建m0（不可清洗部分）
     */
    mpz_class buildM0(const MissionAttributes& mission);
    
    /**
     * 从JSON解析任务消息
     */
    bool parseMissionFromJson(const std::string& json_content, MissionAttributes& mission);
    
    std::unique_ptr<DSSMSSignature> dss_ms_;
    bool initialized_;
};

} // namespace drone_control

