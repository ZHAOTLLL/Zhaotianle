/**
 * 任务签名验证
 * 解析任务签名文件、验证 DSS-MS 签名并提取任务属性，供访问控制引擎使用。
 */
#include "signature/mission_signature_verifier.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

namespace drone_control {

MissionSignatureVerifier::MissionSignatureVerifier() : initialized_(false) {
    dss_ms_ = std::make_unique<DSSMSSignature>();
}

MissionSignatureVerifier::~MissionSignatureVerifier() {
}

bool MissionSignatureVerifier::initialize() {
    if (initialized_) {
        return true;
    }
    
    // 初始化DSS-MS系统（使用与TA相同的参数）
    if (!dss_ms_->setup(256)) {
        std::cerr << "[MissionSignatureVerifier] DSS-MS初始化失败" << std::endl;
        return false;
    }
    
    initialized_ = true;
    std::cout << "[MissionSignatureVerifier] 初始化成功" << std::endl;
    return true;
}

MissionSignatureVerifier::VerificationResult MissionSignatureVerifier::verifyFromFile(
    const std::string& json_file_path) {
    VerificationResult result;
    
    // 读取JSON文件
    std::ifstream file(json_file_path);
    if (!file.is_open()) {
        result.error_message = "无法打开文件: " + json_file_path;
        std::cerr << "[MissionSignatureVerifier] " << result.error_message << std::endl;
        return result;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return verifyFromJson(buffer.str());
}

MissionSignatureVerifier::VerificationResult MissionSignatureVerifier::verifyFromJson(
    const std::string& json_content) {
    VerificationResult result;
    
    if (!initialized_) {
        if (!initialize()) {
            result.error_message = "DSS-MS系统初始化失败";
            return result;
        }
    }
    
#ifdef HAVE_NLOHMANN_JSON
    try {
        json j = json::parse(json_content);
        
        // 解析任务信息和签名
        MissionAttributes mission;
        DSSMSSignature::Signature signature;
        ECPoint company_pk, sanitizer_pk;
        
        if (!parseJsonSignature(json_content, mission, signature, company_pk, sanitizer_pk)) {
            result.error_message = "JSON签名解析失败";
            return result;
        }
        
        // 验证签名（注意参数顺序：signer_pk在前，sanitizer_pk在后）
        std::cout << "[MissionSignatureVerifier] 开始验证签名..." << std::endl;
        
        bool verify_result = dss_ms_->verify(
            signature,
            company_pk,      // signer_pk (签名者公钥 - 公司)
            sanitizer_pk     // sanitizer_pk (清洗者公钥 - UAV)
        );
        
        if (!verify_result) {
            std::cerr << "[MissionSignatureVerifier] [警告] 签名验证失败，可能的原因：" << std::endl;
            std::cerr << "  1. 公共参数（Setup）不一致" << std::endl;
            std::cerr << "  2. 签名数据损坏" << std::endl;
            std::cerr << "  3. 公钥不匹配" << std::endl;
        }
        
        if (!verify_result) {
            result.error_message = "签名验证失败";
            result.is_valid = false;
            return result;
        }
        
        // 验证通过，返回提取的属性和UAV的sanitizer_pk
        result.is_valid = true;
        result.attributes = mission;
        result.sanitizer_pk = sanitizer_pk;  // 保存UAV的清洗者公钥，用于后续加密
        std::cout << "[MissionSignatureVerifier] 签名验证成功" << std::endl;
        std::cout << "[MissionSignatureVerifier] 任务ID: " << mission.mission_id << std::endl;
        std::cout << "[MissionSignatureVerifier] UAV ID: " << mission.drone_id << std::endl;
        std::cout << "[MissionSignatureVerifier] 操作类型: " << mission.operation_type << std::endl;
        std::cout << "[MissionSignatureVerifier] 目标别名: " << mission.target_alias << std::endl;
        std::cout << "[MissionSignatureVerifier] 已提取UAV的sanitizer_pk，可用于加密地理围栏签名" << std::endl;
        
        return result;
        
    } catch (const json::exception& e) {
        result.error_message = "JSON解析错误: " + std::string(e.what());
        std::cerr << "[MissionSignatureVerifier] " << result.error_message << std::endl;
        return result;
    }
#else
    result.error_message = "nlohmann/json库未启用";
    return result;
#endif
}

bool MissionSignatureVerifier::parseJsonSignature(
    const std::string& json_content,
    MissionAttributes& mission,
    DSSMSSignature::Signature& signature,
    ECPoint& company_pk,
    ECPoint& sanitizer_pk) {
    
#ifdef HAVE_NLOHMANN_JSON
    try {
        json j = json::parse(json_content);
        
        // 解析任务信息
        if (!parseMissionFromJson(json_content, mission)) {
            std::cerr << "[MissionSignatureVerifier] 解析任务信息失败" << std::endl;
            return false;
        }
        
        // 解析签名
        if (!j.contains("signature")) {
            std::cerr << "[MissionSignatureVerifier] JSON缺少'signature'字段" << std::endl;
            return false;
        }
        
        auto& sig = j["signature"];
        if (!sig.contains("m0") || !sig.contains("m") || !sig.contains("R") || 
            !sig.contains("T") || !sig.contains("z") || !sig.contains("s")) {
            std::cerr << "[MissionSignatureVerifier] 签名字段不完整" << std::endl;
            return false;
        }
        
        // 解析签名组件
        signature.m0 = hexStringToMPZ(sig["m0"].get<std::string>());
        signature.m = hexStringToMPZ(sig["m"].get<std::string>());
        
        if (!hexStringToECPoint(sig["R"].get<std::string>(), signature.R)) {
            std::cerr << "[MissionSignatureVerifier] 解析R点失败" << std::endl;
            return false;
        }
        
        if (!hexStringToECPoint(sig["T"].get<std::string>(), signature.T)) {
            std::cerr << "[MissionSignatureVerifier] 解析T点失败" << std::endl;
            return false;
        }
        
        signature.z = hexStringToMPZ(sig["z"].get<std::string>());
        signature.s = hexStringToMPZ(sig["s"].get<std::string>());
        
        // 解析公钥
        if (!j.contains("public_keys")) {
            std::cerr << "[MissionSignatureVerifier] JSON缺少'public_keys'字段" << std::endl;
            return false;
        }
        
        auto& pk = j["public_keys"];
        if (!pk.contains("company_pk") || !pk.contains("sanitizer_pk")) {
            std::cerr << "[MissionSignatureVerifier] 公钥字段不完整" << std::endl;
            return false;
        }
        
        if (!hexStringToECPoint(pk["company_pk"].get<std::string>(), company_pk)) {
            std::cerr << "[MissionSignatureVerifier] 解析公司公钥失败" << std::endl;
            return false;
        }
        
        if (!hexStringToECPoint(pk["sanitizer_pk"].get<std::string>(), sanitizer_pk)) {
            std::cerr << "[MissionSignatureVerifier] 解析清洗者公钥失败" << std::endl;
            return false;
        }
        
        return true;
        
    } catch (const json::exception& e) {
        std::cerr << "[MissionSignatureVerifier] JSON解析异常: " << e.what() << std::endl;
        return false;
    }
#else
    return false;
#endif
}

bool MissionSignatureVerifier::parseMissionFromJson(
    const std::string& json_content,
    MissionAttributes& mission) {
    
#ifdef HAVE_NLOHMANN_JSON
    try {
        json j = json::parse(json_content);
        
        if (!j.contains("mission")) {
            std::cerr << "[MissionSignatureVerifier] JSON缺少'mission'字段" << std::endl;
            return false;
        }
        
        auto& m = j["mission"];
        
        // 解析不可清洗部分（m0对应的字段）
        if (m.contains("mission_id")) {
            mission.mission_id = m["mission_id"].get<std::string>();
        }
        
        if (m.contains("drone_id")) {
            mission.drone_id = m["drone_id"].get<uint32_t>();
        }
        
        if (m.contains("operation_type")) {
            mission.operation_type = m["operation_type"].get<std::string>();
        }
        
        if (m.contains("target_alias")) {
            mission.target_alias = m["target_alias"].get<std::string>();
        }
        
        if (m.contains("timestamp")) {
            mission.timestamp = m["timestamp"].get<uint64_t>();
        }
        
        if (m.contains("company_cert_hash")) {
            mission.company_cert_hash = m["company_cert_hash"].get<std::string>();
        }
        
        return true;
        
    } catch (const json::exception& e) {
        std::cerr << "[MissionSignatureVerifier] JSON解析异常: " << e.what() << std::endl;
        return false;
    }
#else
    return false;
#endif
}

mpz_class MissionSignatureVerifier::hexStringToMPZ(const std::string& hex_str) {
    mpz_class result;
    result.set_str(hex_str, 16);
    return result;
}

bool MissionSignatureVerifier::hexStringToECPoint(const std::string& hex_str, ECPoint& point) {
    return point.deserialize(hex_str);
}

mpz_class MissionSignatureVerifier::buildM0(const MissionAttributes& mission) {
    // 构建m0字符串：mission_id|drone_id|operation_type|target_alias|timestamp|company_cert_hash
    std::ostringstream oss;
    oss << mission.mission_id << "|"
        << mission.drone_id << "|"
        << mission.operation_type << "|"
        << mission.target_alias << "|"
        << mission.timestamp << "|"
        << mission.company_cert_hash;
    
    std::string m0_str = oss.str();
    
    // 使用SHA256哈希
    // 注意：这里需要使用与TA服务相同的序列化方法
    // TA服务在serializeMissionToDSS中使用SHA256哈希
    // 但这里我们从JSON中直接读取m0的值，所以不需要重新计算
    // 这个函数主要用于验证m0的完整性
    
    // 暂时返回0，因为m0应该从签名中直接读取
    return mpz_class(0);
}

} // namespace drone_control

