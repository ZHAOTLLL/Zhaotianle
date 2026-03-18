#pragma once

/**
 * 地理围栏签名服务
 * 为授权访问生成地理围栏临时签名/令牌，并支持验证与撤销；依赖 OpenSSL 与 DSS-MS。
 */
#include "common/types.hpp"
#include "flight_control/flight_plan.hpp"
#include <string>
#include <map>
#include <vector>
#include <set>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <gmp.h>
#include <gmpxx.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/sha.h>
#include "signature/dss_ms_signature.hpp"

namespace drone_control {

class GeofenceSignatureService {
public:
    /**
     * 签名数据结构
     */
    struct SignatureData {
        std::string signature;                              // 签名字符串
        std::chrono::steady_clock::time_point expiry_time;  // 过期时间
        DroneId drone_id;                                   // 无人机ID
        std::string region_id;                              // 区域ID
        std::map<std::string, std::string> constraints;     // 属性约束
        std::string operation_type;                         // 操作类型
        bool is_revoked;                                    // 是否已撤销
        
        SignatureData() : drone_id(0), is_revoked(false) {}
    };
    
    /**
     * 签名生成参数
     */
    struct SignatureParams {
        DroneId drone_id;
        std::string region_id;
        std::string operation_type;
        Duration validity_duration;
        std::map<std::string, std::string> attribute_constraints;
        std::string requester_id;
        std::string drone_public_key_pem;  // 从身份认证阶段获取的UAV公钥（对应草稿中的PK_k）
        
        SignatureParams() : drone_id(0), validity_duration(std::chrono::minutes(30)) {}
    };
    
    GeofenceSignatureService();
    ~GeofenceSignatureService();
    
    /**
     * 初始化签名服务
     * @param private_key_file 私钥文件路径
     * @param public_key_file 公钥文件路径
     * @return 是否初始化成功
     */
    bool initialize(const std::string& private_key_file, const std::string& public_key_file);
    
    /**
     * 生成地理围栏签名
     * @param params 签名参数
     * @return 签名字符串，失败时返回空字符串
     */
    std::string generateSignature(const SignatureParams& params);
    
    /**
     * 验证签名有效性
     * @param signature 签名字符串
     * @param drone_id 无人机ID
     * @param region_id 区域ID
     * @return 是否有效
     */
    bool verifySignature(const std::string& signature, DroneId drone_id, const std::string& region_id);
    
    /**
     * 验证签名并检查属性约束
     * @param signature 签名字符串
     * @param drone_id 无人机ID
     * @param region_id 区域ID
     * @param current_attributes 当前属性
     * @return 是否有效且满足约束
     */
    bool verifySignatureWithConstraints(const std::string& signature, 
                                       DroneId drone_id, 
                                       const std::string& region_id,
                                       const std::map<std::string, std::string>& current_attributes);
    
    /**
     * 撤销签名
     * @param signature 签名字符串
     * @return 是否撤销成功
     */
    bool revokeSignature(const std::string& signature);
    
    /**
     * 批量撤销无人机的所有签名
     * @param drone_id 无人机ID
     * @return 撤销的签名数量
     */
    int revokeAllSignatures(DroneId drone_id);
    
    /**
     * 清理过期签名
     * @return 清理的签名数量
     */
    int cleanupExpiredSignatures();
    
    /**
     * 获取签名信息
     * @param signature 签名字符串
     * @return 签名数据，不存在时返回nullptr
     */
    std::shared_ptr<SignatureData> getSignatureData(const std::string& signature);
    
    /**
     * 获取无人机的活跃签名列表
     * @param drone_id 无人机ID
     * @return 签名列表
     */
    std::vector<std::string> getActiveSignatures(DroneId drone_id);
    
    /**
     * 检查签名是否即将过期
     * @param signature 签名字符串
     * @param warning_minutes 预警时间（分钟）
     * @return 是否即将过期
     */
    bool isSignatureExpiringSoon(const std::string& signature, int warning_minutes = 5);
    
    /**
     * 更新签名的属性约束
     * @param signature 签名字符串
     * @param new_constraints 新的约束
     * @return 是否更新成功
     */
    bool updateSignatureConstraints(const std::string& signature, 
                                   const std::map<std::string, std::string>& new_constraints);

    // ========== DSS-MS 围栏签名功能（场景二） ==========
    
    /**
     * DSS-MS围栏消息结构（与TA服务一致）
     */
    struct DSSGeofenceMessage {
        std::string region_id;
        uint64_t timestamp;
        uint64_t validity_period;
        std::string ta_cert_hash;
        uint32_t drone_id;
        std::vector<uint32_t> fence_id_list;
        
        DSSGeofenceMessage() : timestamp(0), validity_period(0), drone_id(0) {}
    };
    
    /**
     * DSS-MS签名后的围栏消息
     */
    struct DSSSignedGeofence {
        DSSGeofenceMessage geofence;
        DSSMSSignature::Signature signature;
        ECPoint ta_pk;
        ECPoint sanitizer_pk;
        
        DSSSignedGeofence() {}
    };
    
    /**
     * 网格坐标映射器（8x8网格）
     * 将经纬度坐标映射到围栏ID
     */
    class GridMapper {
    public:
        /**
         * 网格边界结构
         */
        struct GridBounds {
            double lat_min, lat_max;
            double lon_min, lon_max;
        };
        
        GridMapper();
        ~GridMapper();
        
        /**
         * 初始化网格映射器
         * @param min_lat 最小纬度
         * @param max_lat 最大纬度
         * @param min_lon 最小经度
         * @param max_lon 最大经度
         * @param grid_size 网格大小（默认8x8）
         * @return 是否初始化成功
         */
        bool initialize(double min_lat, double max_lat, 
                       double min_lon, double max_lon,
                       int grid_size = 8);
        
        /**
         * 根据经纬度计算围栏ID
         * @param lat 纬度
         * @param lon 经度
         * @return 围栏ID，如果坐标超出范围则返回空
         */
        std::optional<uint32_t> latLonToFenceId(double lat, double lon);
        
        /**
         * 根据围栏ID计算网格边界
         * @param fence_id 围栏ID
         * @return 网格边界，如果ID无效则返回空
         */
        std::optional<GridBounds> fenceIdToBounds(uint32_t fence_id);
        
        /**
         * 批量计算：路径waypoints -> 围栏ID列表
         * @param waypoints 飞行路径航点列表
         * @return 围栏ID列表（已去重）
         */
        std::vector<uint32_t> pathToFenceIds(const std::vector<Waypoint>& waypoints);
        
        /**
         * 检查是否已初始化
         * @return 是否已初始化
         */
        bool isInitialized() const { return initialized_; }
        
    private:
        double min_lat_, max_lat_;
        double min_lon_, max_lon_;
        int grid_size_;
        double lat_step_, lon_step_;
        bool initialized_;
        
        /**
         * 计算两点之间的距离（米）
         */
        double calculateDistance(const Position& p1, const Position& p2) const;
        
        /**
         * 计算路径段经过的中间网格
         */
        std::vector<uint32_t> calculateIntermediateFences(
            const Waypoint& wp1, const Waypoint& wp2);
    };
    
    /**
     * 初始化DSS-MS围栏签名功能
     * @param sanitizer_sk_hex 清洗者私钥（十六进制字符串，GCS使用，可选）
     * @param u_s_hex DSS-MS u_s参数（十六进制字符串，从TA服务获取，可选）
     * @return 是否初始化成功
     * 
     * 注意：如果参数为空，函数会尝试从默认配置文件加载：
     *   - dss_ms_public_params.json (DSS-MS公共参数)
     *   - gcs_sanitizer_key.json (GCS清洗者私钥)
     */
    bool initializeDSSMSGeofence(const std::string& sanitizer_sk_hex = "", 
                                 const std::string& u_s_hex = "");
    
    /**
     * 从文件加载DSS-MS公共参数（TA服务生成）
     * @param filepath 参数文件路径
     * @return 是否加载成功
     */
    bool loadDSSMSPublicParamsFromFile(const std::string& filepath);
    
    /**
     * 从文件加载GCS清洗者私钥（TA服务生成）
     * @param filepath 密钥文件路径
     * @return 是否加载成功
     */
    bool loadGCSSanitizerKeyFromFile(const std::string& filepath);
    
    /**
     * 设置GCS清洗者私钥（从TA服务获取）
     * @param sanitizer_sk_hex 清洗者私钥（十六进制字符串）
     * @return 是否设置成功
     */
    bool setSanitizerPrivateKey(const std::string& sanitizer_sk_hex);
    
    /**
     * 从JSON文件加载TA生成的围栏签名
     * @param filepath 签名文件路径
     * @return 是否加载成功
     */
    bool loadDSSSignedGeofence(const std::string& filepath);
    
    /**
     * 根据飞行路径清洗围栏签名
     * @param waypoints 飞行路径航点列表
     * @param gcs_validity_period GCS设置的有效期（秒），0表示使用原始TA签名的有效期
     * @return 清洗后的签名，失败时返回空
     */
    std::optional<DSSSignedGeofence> sanitizeGeofenceByPath(
        const std::vector<Waypoint>& waypoints,
        uint64_t gcs_validity_period = 0);
    
    /**
     * 清洗整个region的所有围栏签名（特权场景使用）
     * @param gcs_validity_period GCS设置的有效期（秒），0表示使用原始TA签名的有效期
     * @return 清洗后的签名，失败时返回空
     */
    std::optional<DSSSignedGeofence> sanitizeEntireRegion(
        uint64_t gcs_validity_period = 0);
    
    /**
     * 使用UAV的sanitizer_pk加密清洗后的地理围栏签名（创新点：将签名和加密功能串起来）
     * @param sanitized_geofence 清洗后的地理围栏签名
     * @param uav_sanitizer_pk UAV的清洗者公钥（从任务签名验证中获取）
     * @return Base64编码的加密后的JSON字符串，失败时返回空字符串
     */
    std::string encryptSanitizedGeofenceWithECCPoint(
        const DSSSignedGeofence& sanitized_geofence,
        const ECPoint& uav_sanitizer_pk);
    
    /**
     * 为FCSACM方案生成完整的地理围栏数据（不使用DSS签名）
     * 与FCSACM*方案内容一致，但使用传统签名方案
     * @param waypoints 飞行路径航点列表
     * @param validity_period 有效期（秒）
     * @param drone_id 无人机ID
     * @param region_id 区域ID
     * @return JSON格式的地理围栏数据字符串，失败时返回空字符串
     */
    std::string generateGeofenceDataForPath(
        const std::vector<Waypoint>& waypoints,
        uint64_t validity_period,
        uint32_t drone_id,
        const std::string& region_id);
    
    /**
     * 为FCSACM方案生成整个region的地理围栏数据（特权场景使用）
     * 包含所有64个围栏ID（101-164）
     * @param validity_period 有效期（秒）
     * @param drone_id 无人机ID
     * @param region_id 区域ID
     * @return JSON格式的地理围栏数据字符串，失败时返回空字符串
     */
    std::string generateGeofenceDataForEntireRegion(
        uint64_t validity_period,
        uint32_t drone_id,
        const std::string& region_id);

    std::string generateGeofenceDataForPathWithTASignature(
        const std::vector<Waypoint>& waypoints,
        uint64_t validity_period,
        uint32_t drone_id,
        const std::string& region_id,
        const std::string& ta_signed_region_json);

    std::string generateTASignatureForRegion(
        uint64_t validity_period,
        const std::string& region_id,
        const std::string& ta_private_key_file,
        const std::string& ta_public_key_file);
    
    /**
     * 保存清洗后的围栏签名到文件
     * @param sanitized 清洗后的签名
     * @param filepath 保存路径
     * @return 是否保存成功
     */
    bool saveDSSSanitizedGeofence(const DSSSignedGeofence& sanitized, 
                                  const std::string& filepath);
    
    /**
     * 从JSON文件验证围栏签名
     * @param filepath 签名文件路径
     * @return 验证结果，包含是否有效和提取的围栏ID列表
     */
    struct DSSVerificationResult {
        bool is_valid;
        std::string error_message;
        std::vector<uint32_t> fence_ids;
        DSSGeofenceMessage geofence;
        
        DSSVerificationResult() : is_valid(false) {}
    };
    
    DSSVerificationResult verifyDSSGeofenceFromFile(const std::string& filepath);
    
    /**
     * 从JSON字符串验证围栏签名
     * @param json_content JSON字符串内容
     * @return 验证结果
     */
    DSSVerificationResult verifyDSSGeofenceFromJson(const std::string& json_content);
    
    /**
     * 根据经纬度获取围栏ID（用于查找最近的围栏区域）
     * @param lat 纬度
     * @param lon 经度
     * @return 围栏ID，如果坐标超出范围则返回空
     */
    std::optional<uint32_t> getFenceIdForPosition(double lat, double lon);
    
    /**
     * 根据围栏ID获取围栏边界（用于获取围栏中心点）
     * @param fence_id 围栏ID
     * @return 围栏边界，如果ID无效则返回空
     */
    std::optional<GridMapper::GridBounds> getFenceBounds(uint32_t fence_id);
    
    /**
     * 使用UAV公钥加密（对应草稿中的 D.cred = Enc_{PK_k}(M)）
     * @param public_key_pem UAV公钥的PEM格式字符串
     * @param plaintext 要加密的明文（令牌M）
     * @return Base64编码的密文，失败时返回空字符串
     */
    std::string encryptWithPublicKey(const std::string& public_key_pem, const std::string& plaintext);
    
    /**
     * 使用sanitizer_pk进行ECC加密（ECIES方案）
     * @param sanitizer_pk 清洗者公钥（ECPoint）
     * @param plaintext 要加密的明文
     * @return Base64编码的密文（格式：R_hex||IV_hex||ciphertext_hex||tag_hex），失败时返回空字符串
     */
    std::string encryptWithECCPoint(const ECPoint& sanitizer_pk, const std::string& plaintext);
    
    /**
     * 从十六进制字符串创建ECPoint（用于baseline方案，像DSS方案一样）
     * @param hex_str 十六进制字符串（Miracl序列化的ECPoint格式）
     * @param point 输出的ECPoint
     * @return 成功返回true，失败返回false
     */
    bool hexStringToECPoint(const std::string& hex_str, ECPoint& point);
    
    /**
     * 使用sanitizer_sk进行ECC解密（ECIES方案）- 公共接口用于测试
     * @param sanitizer_sk 清洗者私钥（mpz_class）
     * @param encrypted_data Base64编码的密文（格式：R_hex||IV_hex||ciphertext_hex||tag_hex）
     * @return 解密后的明文，失败时返回空字符串
     */
    std::string decryptWithECCScalar(const mpz_class& sanitizer_sk, const std::string& encrypted_data);

private:
    std::map<std::string, std::shared_ptr<SignatureData>> active_signatures_;
    std::unique_ptr<RSA, decltype(&RSA_free)> private_key_;
    std::unique_ptr<RSA, decltype(&RSA_free)> public_key_;
    std::mutex signature_mutex_;
    
    // DSS-MS相关成员
    std::unique_ptr<DSSMSSignature> dss_ms_;
    GridMapper grid_mapper_;
    DSSSignedGeofence original_dss_signed_geofence_;
    mpz_class sanitizer_sk_;
    mpz_class u_s_;  // DSS-MS u_s参数（从TA服务获取）
    bool dss_ms_initialized_;
    
    /**
     * 加载RSA私钥
     */
    bool loadPrivateKey(const std::string& key_file);
    
    /**
     * 加载RSA公钥
     */
    bool loadPublicKey(const std::string& key_file);
    
    /**
     * 创建签名数据字符串
     */
    std::string createSignatureData(const SignatureParams& params);
    
    /**
     * 解析签名数据
     */
    bool parseSignatureData(const std::string& signature_data, SignatureParams& params);
    
    /**
     * 计算RSA签名
     */
    std::string computeRSASignature(const std::string& data);
    
    /**
     * 验证RSA签名
     */
    bool verifyRSASignature(const std::string& data, const std::string& signature);
    
    /**
     * 检查属性约束是否满足
     */
    bool checkAttributeConstraints(const std::map<std::string, std::string>& constraints,
                                  const std::map<std::string, std::string>& current_attributes);
    
    /**
     * 生成唯一签名ID
     */
    std::string generateSignatureId();
    
    // DSS-MS相关私有方法
    /**
     * 序列化围栏消息为mpz_class（与TA服务一致）
     */
    void serializeDSSGeofenceToMPZ(const DSSGeofenceMessage& msg, mpz_class& m0, mpz_class& m);
    
    /**
     * 从JSON解析DSS-MS围栏签名
     */
    bool parseDSSJsonSignature(const std::string& json_content,
                               DSSGeofenceMessage& geofence,
                               DSSMSSignature::Signature& signature,
                               ECPoint& ta_pk,
                               ECPoint& sanitizer_pk);
    
    /**
     * 从十六进制字符串创建mpz_class
     */
    mpz_class hexStringToMPZ(const std::string& hex_str);
};

} // namespace drone_control