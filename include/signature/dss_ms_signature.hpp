#pragma once

/**
 * DSS-MS 数字签名（基于 Miracl Ed25519）
 * 提供椭圆曲线点封装、签名与验证，供任务签名与地理围栏签名使用。
 */
#include "common/types.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <gmp.h>
#include <gmpxx.h>
#include "../../third_party/miracl-core/cpp/core.h"
#include "../../third_party/miracl-core/cpp/ecp_Ed25519.h"
#include "../../third_party/miracl-core/cpp/big_B256_56.h"
#include "../../third_party/miracl-core/cpp/fp_F25519.h"
#include "../../third_party/miracl-core/cpp/randapi.h"
// octet类型已在core.h中定义，不需要单独的oct.h

using namespace core;
using namespace Ed25519;
using namespace B256_56;

namespace drone_control {

/**
 * 椭圆曲线点包装类，封装Miracl Core的ECP类型
 * 支持序列化、复制、赋值等操作
 */
class ECPoint {
public:
    ECP ecp_;  // Miracl Core的椭圆曲线点
    
    // 默认构造函数
    ECPoint() {
        ECP_inf(&ecp_);
    }
    
    // 从ECP构造
    ECPoint(const ECP& ecp) {
        ECP temp;
        ECP_copy(&temp, const_cast<ECP*>(&ecp));
        ECP_copy(&ecp_, &temp);
    }
    
    // 复制构造函数
    ECPoint(const ECPoint& other) {
        ECP_copy(&ecp_, const_cast<ECP*>(&other.ecp_));
    }
    
    // 赋值运算符
    ECPoint& operator=(const ECPoint& other) {
        if (this != &other) {
            ECP_copy(&ecp_, const_cast<ECP*>(&other.ecp_));
        }
        return *this;
    }
    
    // 获取内部ECP引用
    ECP& get() { return ecp_; }
    const ECP& get() const { return ecp_; }
    
    // 序列化为字符串（压缩格式）
    std::string serialize() const;
    
    // 从字符串反序列化
    bool deserialize(const std::string& data);
    
    // 判断是否为无穷远点
    bool isInfinity() const {
        return ECP_isinf(const_cast<ECP*>(&ecp_)) != 0;
    }
};

/**
 * DSS-MS可清洗签名模块
 * 实现动态可清洗签名算法（Dynamic Sanitizable Signature with Multiple Sanitizers）
 * 
 * 参考论文：Towards Privacy-Preserving Unmanned Aerial Vehicles Shared Logistics 
 *          via Dynamic Sanitizable Signature with Multiple Sanitizers
 */
class DSSMSSignature {
public:
    /**
     * 系统公共参数
     */
    struct PublicParams {
        mpz_class q;          // 椭圆曲线群的阶
        ECPoint P;            // 生成元
        mpz_class u_s;        // 清洗密钥掩码（使用CRT构造）
        
        PublicParams() {}
    };
    
    /**
     * 密钥对结构
     */
    struct KeyPair {
        mpz_class sk;         // 私钥
        ECPoint PK;           // 公钥（PK = sk * P）
        
        KeyPair() {}
    };
    
    /**
     * 签名结构
     */
    struct Signature {
        mpz_class m0;         // 不可清洗的消息部分
        mpz_class m;          // 可清洗的消息部分
        ECPoint R;            // 承诺组件
        ECPoint T;            // 变色龙哈希组件
        mpz_class z;          // Schnorr签名响应
        mpz_class s;          // 变色龙哈希参数
        
        Signature() {}
    };
    
    /**
     * CRT结构（用于管理多个清洗者）
     */
    struct CRTParams {
        std::vector<mpz_class> sk;  // 所有清洗者的私钥列表
        mpz_class M;                // M = product of all sk[i]
        mpz_class u;                // CRT参数
        
        CRTParams() {}
    };
    
    DSSMSSignature();
    ~DSSMSSignature();
    
    /**
     * 初始化系统参数
     * 对应算法：Setup(1^λ)
     * @param security_param 安全参数（用于选择曲线）
     * @return 是否初始化成功
     */
    bool setup(int security_param = 256);
    
    /**
     * 生成密钥对
     * 对应算法：KeyGen(pp, n)
     * @param num_sanitizers 清洗者数量
     * @param bits 密钥长度（位数）
     * @param signer_keypair 输出：签名者的密钥对
     * @return 清洗者的私钥列表
     */
    std::vector<mpz_class> keyGen(int num_sanitizers, int bits, KeyPair& signer_keypair);
    
    /**
     * 生成签名
     * 对应算法：Sign(pp, sk, M, PK_s)
     * @param signer_sk 签名者私钥
     * @param sanitizer_pk 清洗者公钥
     * @param m0 不可清洗消息部分
     * @param m 可清洗消息部分
     * @param proof_t 输出：证明密钥t（用于后续Proof）
     * @return 生成的签名
     */
    Signature sign(const mpz_class& signer_sk, 
                   const ECPoint& sanitizer_pk,
                   const mpz_class& m0,
                   const mpz_class& m,
                   mpz_class& proof_t);
    
    /**
     * 清洗签名
     * 对应算法：Sanitizing(σ, sk_i)
     * @param original_sigma 原始签名
     * @param sanitizer_sk_i 第i个清洗者的私钥
     * @param sanitizer_pk 清洗者公钥
     * @param new_m 新的可清洗消息部分
     * @return 清洗后的签名
     */
    Signature sanitizing(const Signature& original_sigma,
                        const mpz_class& sanitizer_sk_i,
                        const ECPoint& sanitizer_pk,
                        const mpz_class& new_m);
    
    /**
     * 验证签名
     * 对应算法：Verify(σ, PK)
     * @param sigma 待验证的签名（原始或清洗后）
     * @param signer_pk 签名者公钥
     * @param sanitizer_pk 清洗者公钥
     * @return 验证结果：true表示有效，false表示无效
     */
    bool verify(const Signature& sigma,
                const ECPoint& signer_pk,
                const ECPoint& sanitizer_pk);
    
    /**
     * 生成证明（用于可追溯性）
     * 对应算法：Proof(σ, t)
     * @param sigma 签名
     * @param proof_t 证明密钥t
     * @return 零知识证明
     */
    KeyPair proof(const Signature& sigma, const mpz_class& proof_t);
    
    /**
     * 判断签名来源
     * 对应算法：Judge(π)
     * @param proof_pi 零知识证明
     * @param sigma 签名
     * @return true表示来自原始签名者，false表示来自清洗者
     */
    bool judge(const KeyPair& proof_pi, const Signature& sigma);
    
    /**
     * 添加新的清洗者
     * 对应算法：Join(n)
     * @param sanitizer_sk 清洗者密钥（用于更新u_s）
     * @param bits 密钥长度
     * @param num_new_sanitizers 新增清洗者数量
     */
    void join(const mpz_class& sanitizer_sk, int bits, int num_new_sanitizers);
    
    /**
     * 撤销清洗者
     * 对应算法：Revoke(I)
     * @param sanitizer_sk 清洗者密钥（用于更新u_s）
     * @param revoked_sk_list 被撤销的清洗者私钥列表
     */
    void revoke(const mpz_class& sanitizer_sk, const std::vector<mpz_class>& revoked_sk_list);
    
    /**
     * 获取系统公共参数
     */
    const PublicParams& getPublicParams() const { return pp_; }
    
    /**
     * 设置u_s参数（用于GCS等不需要调用keyGen的场景）
     * @param u_s 清洗密钥掩码值
     */
    void setU_s(const mpz_class& u_s) { pp_.u_s = u_s; }
    
    /**
     * 设置P参数（生成元，用于确保与TA一致）
     * @param P_hex 生成元P的序列化字符串（十六进制）
     */
    bool setP(const std::string& P_hex);
    
    /**
     * 设置q参数（群的阶，用于确保与TA一致）
     * @param q_hex 群的阶q的十六进制字符串
     */
    void setQ(const std::string& q_hex) { 
        mpz_class q_val;
        q_val.set_str(q_hex, 16);
        pp_.q = q_val;
    }
    
    /**
     * 获取CRT参数
     */
    const CRTParams& getCRTParams() const { return crt_; }
    
    /**
     * 获取清洗者密钥对（所有清洗者共享的公钥）
     */
    const KeyPair& getSanitizerKeyPair() const { return sanitizer_keypair_; }
    
    /**
     * 获取清洗者私钥列表（每个清洗者有不同的私钥）
     */
    const std::vector<mpz_class>& getSanitizerPrivateKeys() const { return sanitizer_sks_; }
    
    /**
     * 生成随机大整数（公共接口，供TA等服务使用）
     */
    mpz_class randomMPZ(int bits);
    
    /**
     * 椭圆曲线点乘法：result = k * P（公共接口，供TA等服务使用）
     */
    ECPoint ecPointMultiply(const ECPoint& P, const mpz_class& k);
    
private:
    PublicParams pp_;         // 公共参数
    CRTParams crt_;           // CRT参数（用于管理清洗者）
    KeyPair sanitizer_keypair_;  // 清洗者密钥对（所有清洗者共享公钥）
    std::vector<mpz_class> sanitizer_sks_;  // 清洗者私钥列表
    
    /**
     * 检查两个大整数是否互质
     */
    bool areCoprime(const mpz_class& a, const mpz_class& b);
    
    /**
     * 计算大整数的模逆
     */
    mpz_class invertMPZ(const mpz_class& a, const mpz_class& mod);
    
    /**
     * 变色龙哈希函数 H_ch(m, T)
     */
    mpz_class hashChameleon(const mpz_class& m, const ECPoint& T);
    
    /**
     * 标准哈希函数 H(m0, R, CH)
     */
    mpz_class hashStandard(const mpz_class& m0, const ECPoint& R, const ECPoint& CH);
    
    /**
     * 椭圆曲线点加法：result = P + Q
     */
    ECPoint ecPointAdd(const ECPoint& P, const ECPoint& Q);
    
    /**
     * 椭圆曲线点取负：result = -P
     */
    ECPoint ecPointNegate(const ECPoint& P);
    
    /**
     * 椭圆曲线点相等判断
     */
    bool ecPointEqual(const ECPoint& P, const ECPoint& Q);
    
    /**
     * 初始化随机数生成器
     */
    void initRandomGenerator();
    
    // 随机数生成器状态（需要根据实际使用的库定义）
    void* rng_state_;
};

} // namespace drone_control
