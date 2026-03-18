/**
 * DSS-MS 数字签名（Miracl Ed25519）
 * 签名与验证、椭圆曲线点与随机数初始化，供任务签名与地理围栏签名使用。
 */
#include "signature/dss_ms_signature.hpp"
#include <iostream>
#include <random>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <ctime>

namespace drone_control {

csprng rng_;
gmp_randstate_t gmp_state_;

// 辅助函数：mpz_class转BIG
// Ed25519曲线的Zp元素是32字节（256位）
void mpz_to_BIG(const mpz_class& mpz, BIG& big) {
    std::string hex_str = mpz.get_str(16);
    // Ed25519的BIG类型是32字节，十六进制需要64个字符
    if (hex_str.length() < 64) {
        hex_str.insert(0, 64 - hex_str.length(), '0');
    }
    char bytes[32] = {0};
    for (size_t i = 0; i < 32; ++i) {
        std::string byte_str = hex_str.substr(2 * i, 2);
        bytes[i] = static_cast<unsigned char>(strtol(byte_str.c_str(), nullptr, 16));
    }
    BIG_fromBytesLen(big, bytes, 32);
}

// 辅助函数：BIG转mpz_class
mpz_class BIG_to_mpz(const BIG& big) {
    char bytes[32];
    BIG_toBytes(bytes, const_cast<BIG&>(big));
    std::ostringstream oss;
    for (size_t i = 0; i < 32; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(static_cast<unsigned char>(bytes[i]));
    }
    mpz_class result;
    result.set_str(oss.str(), 16);
    return result;
}

DSSMSSignature::DSSMSSignature() : rng_state_(nullptr) {
    initRandomGenerator();
    // 初始化公共参数
    setup(256);
}

DSSMSSignature::~DSSMSSignature() {
    // 清理资源
}

bool DSSMSSignature::setup(int security_param) {
    // 使用Ed25519曲线初始化（与X.509证书验证统一）
    // 设置生成元P
    ECP_generator(&pp_.P.get());
    
    // 设置群的阶q
    BIG order;
    BIG_rcopy(order, CURVE_Order);
    pp_.q = BIG_to_mpz(order);
    
    std::cout << "[DSS-MS] Setup with Ed25519 curve, security parameter: " << security_param << std::endl;
    return true;
}

bool DSSMSSignature::areCoprime(const mpz_class& a, const mpz_class& b) {
    mpz_class g;
    mpz_gcd(g.get_mpz_t(), a.get_mpz_t(), b.get_mpz_t());
    return g == 1;
}

mpz_class DSSMSSignature::invertMPZ(const mpz_class& a, const mpz_class& mod) {
    mpz_class result;
    if (mpz_invert(result.get_mpz_t(), a.get_mpz_t(), mod.get_mpz_t()) == 0) {
        throw std::runtime_error("Modular inverse does not exist");
    }
    return result;
}

std::vector<mpz_class> DSSMSSignature::keyGen(int num_sanitizers, int bits, KeyPair& signer_keypair) {
    std::vector<mpz_class> sks;
    mpz_class sk, M_batch = 1;
    int count = 0;
    
    // 使用全局gmp_state_，与原始代码一致
    // 生成互质的清洗者私钥
    while (count < num_sanitizers) {
        while (true) {
            mpz_urandomb(sk.get_mpz_t(), gmp_state_, bits);
            if (mpz_sizeinbase(sk.get_mpz_t(), 2) >= (bits - 5)) break;
        }
        
        sk |= 1;  // 确保是奇数
        if (areCoprime(sk, M_batch)) {
            sks.push_back(sk);
            M_batch *= sk;
            ++count;
        }
    }
    
    // 生成清洗者密钥对（用于计算u_s），与原始代码一致
    KeyPair sanitizer_keypair;
    mpz_urandomb(sanitizer_keypair.sk.get_mpz_t(), gmp_state_, bits - 5);
    ECP_generator(&sanitizer_keypair.PK.get());
    BIG sk_big;
    mpz_to_BIG(sanitizer_keypair.sk, sk_big);
    ECP_mul(&sanitizer_keypair.PK.get(), sk_big);
    
    // 生成签名者密钥对（原始代码中在Sign函数中生成，但这里提前生成）
    signer_keypair.sk = randomMPZ(bits);
    signer_keypair.PK = ecPointMultiply(pp_.P, signer_keypair.sk);
    
    // 计算CRT参数
    std::vector<mpz_class> M_i(num_sanitizers), y_i(num_sanitizers);
    for (int i = 0; i < num_sanitizers; ++i) {
        M_i[i] = M_batch / sks[i];
        y_i[i] = invertMPZ(M_i[i], sks[i]);
    }
    
    mpz_class u = 0;
    for (int i = 0; i < num_sanitizers; ++i) {
        u += y_i[i] * M_i[i];
    }
    
    crt_.sk = sks;
    crt_.M = M_batch;
    crt_.u = u;
    
    // 设置 pp_.u_s = sanitizer_sk * u（与原始代码一致）
    pp_.u_s = sanitizer_keypair.sk * u;
    
    // 保存清洗者密钥对和私钥列表
    sanitizer_keypair_ = sanitizer_keypair;
    sanitizer_sks_ = sks;
    
    return sks;
}

mpz_class DSSMSSignature::hashChameleon(const mpz_class& m, const ECPoint& T) {
    // 实现 H_ch(m, T) = Hash(m || T)
    // 使用Miracl Core的hashZp256，类似原始代码的实现
    
    hash256 h;
    HASH256_init(&h);
    
    // 将m转换为octet并添加到哈希
    size_t m_size = (mpz_sizeinbase(m.get_mpz_t(), 2) + 7) / 8;
    unsigned char* m_bytes = new unsigned char[m_size];
    mpz_export(m_bytes, nullptr, 1, 1, 0, 0, m.get_mpz_t());
    for (size_t i = 0; i < m_size; ++i) {
        HASH256_process(&h, m_bytes[i]);
    }
    delete[] m_bytes;
    
    // 将T转换为octet并添加到哈希
    char octet_buf[100];
    octet oct_T = {0, sizeof(octet_buf), octet_buf};
    ECP_toOctet(&oct_T, (ECP*)&T.get(), true);
    for (int i = 0; i < oct_T.len; ++i) {
        HASH256_process(&h, oct_T.val[i]);
    }
    
    // 计算哈希值（SHA256输出32字节）
    char hashstr[32];
    memset(hashstr, 0, 32);
    HASH256_hash(&h, hashstr);
    
    // 转换为BIG并模q
    BIG order, hash_big;
    BIG_rcopy(order, CURVE_Order);
    BIG_fromBytesLen(hash_big, hashstr, 32);
    BIG_mod(hash_big, order);
    
    return BIG_to_mpz(hash_big);
}

mpz_class DSSMSSignature::hashStandard(const mpz_class& m0, const ECPoint& R, const ECPoint& CH) {
    // 实现 H(m0, R, CH) = Hash(m0 || R || CH)
    // 使用Miracl Core的hashZp256
    
    hash256 h;
    HASH256_init(&h);
    
    // 添加m0
    size_t m0_size = (mpz_sizeinbase(m0.get_mpz_t(), 2) + 7) / 8;
    unsigned char* m0_bytes = new unsigned char[m0_size];
    mpz_export(m0_bytes, nullptr, 1, 1, 0, 0, m0.get_mpz_t());
    for (size_t i = 0; i < m0_size; ++i) {
        HASH256_process(&h, m0_bytes[i]);
    }
    delete[] m0_bytes;
    
    // 添加R
    char octet_buf1[100];
    octet oct_R = {0, sizeof(octet_buf1), octet_buf1};
    ECP_toOctet(&oct_R, (ECP*)&R.get(), true);
    for (int i = 0; i < oct_R.len; ++i) {
        HASH256_process(&h, oct_R.val[i]);
    }
    
    // 添加CH
    char octet_buf2[100];
    octet oct_CH = {0, sizeof(octet_buf2), octet_buf2};
    ECP_toOctet(&oct_CH, (ECP*)&CH.get(), true);
    for (int i = 0; i < oct_CH.len; ++i) {
        HASH256_process(&h, oct_CH.val[i]);
    }
    
    // 计算哈希值（SHA256输出32字节）
    char hashstr[32];
    memset(hashstr, 0, 32);
    HASH256_hash(&h, hashstr);
    
    // 转换为BIG并模q
    BIG order, hash_big;
    BIG_rcopy(order, CURVE_Order);
    BIG_fromBytesLen(hash_big, hashstr, 32);
    BIG_mod(hash_big, order);
    
    return BIG_to_mpz(hash_big);
}

mpz_class DSSMSSignature::randomMPZ(int bits) {
    // 使用GMP随机数生成，与原始代码的rand_mpz一致
    // 原始代码使用rand_mpz(state_DSS)，这里使用gmp_state_
    mpz_class result;
    mpz_class max_value = pp_.q - 1;  // 使用曲线阶-1作为最大值
    mpz_urandomm(result.get_mpz_t(), gmp_state_, max_value.get_mpz_t());
    return result + 1;  // 确保结果在[1, q-1]范围内
}

DSSMSSignature::Signature DSSMSSignature::sign(const mpz_class& signer_sk, 
                                                const ECPoint& sanitizer_pk,
                                                const mpz_class& m0,
                                                const mpz_class& m,
                                                mpz_class& proof_t) {
    Signature sigma;
    
    // 生成随机数 r, s, t
    mpz_class r = randomMPZ(256);
    mpz_class s = randomMPZ(256);
    proof_t = randomMPZ(256);
    
    // 计算 R = r * P（按照原始代码的方式）
    ECP_generator(&sigma.R.get());
    BIG r_big;
    mpz_to_BIG(r, r_big);
    ECP_mul(&sigma.R.get(), r_big);
    
    // 计算 T = t * P（按照原始代码的方式）
    ECP_generator(&sigma.T.get());
    BIG t_big;
    mpz_to_BIG(proof_t, t_big);
    ECP_mul(&sigma.T.get(), t_big);
    
    // 计算 e = H_ch(m, T)
    mpz_class e = hashChameleon(m, sigma.T);
    
    // 计算 CH = T + s*P + e*PK_s
    // 按照原始代码的顺序：CH = e*PK_s + s*P + T
    ECPoint temp_P;
    ECP_copy(&temp_P.get(), const_cast<ECP*>(&pp_.P.get()));
    ECPoint CH;
    ECP_copy(&CH.get(), const_cast<ECP*>(&sanitizer_pk.get()));
    
    // temp = s*P
    BIG s_big;
    mpz_to_BIG(s, s_big);
    ECP_mul(&temp_P.get(), s_big);
    
    // CH = e*PK_s
    BIG e_big;
    mpz_to_BIG(e, e_big);
    ECP_mul(&CH.get(), e_big);
    
    // CH = e*PK_s + s*P
    ECP_add(&CH.get(), &temp_P.get());
    
    // CH = T + s*P + e*PK_s
    ECP_add(&CH.get(), const_cast<ECP*>(&sigma.T.get()));
    
    // 计算 c = H(m0, R, CH)
    mpz_class c = hashStandard(m0, sigma.R, CH);
    
    // 计算 z = r + sk * c (mod q) - 按照原始代码的方式
    sigma.z = (r + signer_sk * c) % pp_.q;
    
    sigma.m0 = m0;
    sigma.m = m;
    sigma.s = s;
    
    return sigma;
}

DSSMSSignature::Signature DSSMSSignature::sanitizing(const Signature& original_sigma,
                                                     const mpz_class& sanitizer_sk_i,
                                                     const ECPoint& sanitizer_pk,
                                                     const mpz_class& new_m) {
    Signature sigma_p;
    
    // 计算 sk_s = u_s mod sk_i
    mpz_class sk_s = pp_.u_s % sanitizer_sk_i;
    
    // 重建原始CH = T + s*P + e*PK_s
    // 按照原始代码的顺序：CH = e*PK_s + s*P + T
    mpz_class e = hashChameleon(original_sigma.m, original_sigma.T);
    ECPoint temp_P;
    ECP_copy(&temp_P.get(), const_cast<ECP*>(&pp_.P.get()));
    ECPoint CH;
    ECP_copy(&CH.get(), const_cast<ECP*>(&sanitizer_pk.get()));
    
    // temp = s*P
    BIG s_big;
    mpz_to_BIG(original_sigma.s, s_big);
    ECP_mul(&temp_P.get(), s_big);
    
    // CH = e*PK_s
    BIG e_big;
    mpz_to_BIG(e, e_big);
    ECP_mul(&CH.get(), e_big);
    
    // CH = e*PK_s + s*P
    ECP_add(&CH.get(), &temp_P.get());
    
    // CH = T + s*P + e*PK_s
    ECP_add(&CH.get(), const_cast<ECP*>(&original_sigma.T.get()));
    
    // 生成新的随机数k
    mpz_class k = randomMPZ(256);
    
    // 计算 T' = CH - k*P（按照原始代码的方式）
    ECP_copy(&sigma_p.T.get(), &CH.get());
    ECPoint K;
    ECP_copy(&K.get(), const_cast<ECP*>(&pp_.P.get()));
    BIG k_big;
    mpz_to_BIG(k, k_big);
    ECP_mul(&K.get(), k_big);
    ECP_neg(&K.get());
    ECP_add(&sigma_p.T.get(), &K.get());
    
    // 计算 e' = H_ch(new_m, T')
    mpz_class e_p = hashChameleon(new_m, sigma_p.T);
    
    // 计算 s' = k - e'*sk_s (mod q)
    // 使用模运算的正确方式：s' = (k - e'*sk_s) % q
    // 为了处理负数，使用：s' = (k + q - (e'*sk_s % q)) % q
    mpz_class e_sk_s = (e_p * sk_s) % pp_.q;
    sigma_p.s = (k + pp_.q - e_sk_s) % pp_.q;
    
    sigma_p.m0 = original_sigma.m0;
    sigma_p.m = new_m;
    sigma_p.R = original_sigma.R;
    sigma_p.z = original_sigma.z;
    
    return sigma_p;
}

bool DSSMSSignature::verify(const Signature& sigma,
                            const ECPoint& signer_pk,
                            const ECPoint& sanitizer_pk) {
    // 计算 e = H_ch(m, T)
    mpz_class e = hashChameleon(sigma.m, sigma.T);
    
    // 重建 CH = T + s*P + e*PK_s
    // 按照原始代码的顺序：CH = e*PK_s + s*P + T
    ECPoint temp_P;
    ECP_copy(&temp_P.get(), const_cast<ECP*>(&pp_.P.get()));
    ECPoint CH;
    ECP_copy(&CH.get(), const_cast<ECP*>(&sanitizer_pk.get()));
    
    // temp = s*P
    BIG s_big;
    mpz_to_BIG(sigma.s, s_big);
    ECP_mul(&temp_P.get(), s_big);
    
    // CH = e*PK_s
    BIG e_big;
    mpz_to_BIG(e, e_big);
    ECP_mul(&CH.get(), e_big);
    
    // CH = e*PK_s + s*P
    ECP_add(&CH.get(), &temp_P.get());
    
    // CH = T + s*P + e*PK_s
    ECP_add(&CH.get(), const_cast<ECP*>(&sigma.T.get()));
    
    // 计算 c = H(m0, R, CH)
    mpz_class c = hashStandard(sigma.m0, sigma.R, CH);
    
    // 验证 z*P = R + c*PK
    // 按照原始代码的顺序
    ECPoint left;
    ECP_copy(&left.get(), const_cast<ECP*>(&pp_.P.get()));
    BIG z_big;
    mpz_to_BIG(sigma.z, z_big);
    ECP_mul(&left.get(), z_big);  // left = z*P
    
    ECPoint right;
    ECP_copy(&right.get(), const_cast<ECP*>(&signer_pk.get()));
    BIG c_big;
    mpz_to_BIG(c, c_big);
    ECP_mul(&right.get(), c_big);  // right = c*PK
    ECP_add(&right.get(), const_cast<ECP*>(&sigma.R.get()));  // right = R + c*PK
    
    bool result = ecPointEqual(left, right);
    
    if (!result) {
        std::cerr << "[DSS-MS Verify] 验证失败：" << std::endl;
        std::cerr << "  重建的CH序列化: " << CH.serialize().substr(0, 50) << "..." << std::endl;
        std::cerr << "  left (z*P) 序列化: " << left.serialize().substr(0, 50) << "..." << std::endl;
        std::cerr << "  right (R+c*PK) 序列化: " << right.serialize().substr(0, 50) << "..." << std::endl;
        std::cerr << "  c值: " << c.get_str(16).substr(0, 50) << "..." << std::endl;
        std::cerr << "  e值: " << e.get_str(16).substr(0, 50) << "..." << std::endl;
        std::cerr << "  m值: " << sigma.m.get_str(16).substr(0, 50) << "..." << std::endl;
        std::cerr << "  m0值: " << sigma.m0.get_str(16).substr(0, 50) << "..." << std::endl;
        
        // 检查是否可能是清洗后的签名（通过检查m是否包含"***"）
        // 这里只是输出信息，不影响验证逻辑
    }
    
    return result;
}

DSSMSSignature::KeyPair DSSMSSignature::proof(const Signature& sigma, const mpz_class& proof_t) {
    KeyPair pi;
    
    // 生成随机数r'
    mpz_class r_prime = randomMPZ(256);
    
    // 计算 R' = r' * P
    pi.PK = ecPointMultiply(pp_.P, r_prime);
    
    // 计算 c' = H(m, R', T)
    mpz_class c_prime = hashStandard(sigma.m, pi.PK, sigma.T);
    
    // 计算 z' = r' + c' * t (mod q)
    mpz_class z_prime_temp = r_prime + c_prime * proof_t;
    BIG z_prime_big, q_big;
    mpz_to_BIG(z_prime_temp, z_prime_big);
    BIG_rcopy(q_big, CURVE_Order);
    BIG_mod(z_prime_big, q_big);
    pi.sk = BIG_to_mpz(z_prime_big);
    
    return pi;
}

bool DSSMSSignature::judge(const KeyPair& proof_pi, const Signature& sigma) {
    // 验证 z'*P = R' + H(m, R', T)*T
    mpz_class c = hashStandard(sigma.m, proof_pi.PK, sigma.T);
    
    ECPoint left = ecPointMultiply(pp_.P, proof_pi.sk);
    ECPoint cT = ecPointMultiply(sigma.T, c);
    ECPoint right = ecPointAdd(proof_pi.PK, cT);
    
    return ecPointEqual(left, right);
}

void DSSMSSignature::join(const mpz_class& sanitizer_sk, int bits, int num_new_sanitizers) {
    mpz_class M_batch = 1;
    mpz_class sk_star;
    int i = 0;
    
    gmp_randstate_t state;
    gmp_randinit_default(state);
    gmp_randseed_ui(state, time(nullptr));
    
    // 生成新的互质私钥
    while (i < num_new_sanitizers) {
        while (true) {
            mpz_urandomb(sk_star.get_mpz_t(), state, bits);
            if (mpz_sizeinbase(sk_star.get_mpz_t(), 2) >= (bits - 5)) break;
        }
        sk_star |= 1;
        
        if (areCoprime(sk_star, crt_.M) && areCoprime(sk_star, M_batch)) {
            crt_.sk.push_back(sk_star);
            M_batch *= sk_star;
            i++;
        }
    }
    
    // 重新计算M和u
    crt_.M = 1;
    for (const auto& sk : crt_.sk) {
        crt_.M *= sk;
    }
    
    crt_.u = 0;
    int len = crt_.sk.size();
    for (int i = 0; i < len; ++i) {
        mpz_class Mi = crt_.M / crt_.sk[i];
        mpz_class yi = invertMPZ(Mi, crt_.sk[i]);
        crt_.u += Mi * yi;
    }
    
    pp_.u_s = sanitizer_sk * crt_.u;
    
    gmp_randclear(state);
}

void DSSMSSignature::revoke(const mpz_class& sanitizer_sk, const std::vector<mpz_class>& revoked_sk_list) {
    for (const auto& sk_star : revoked_sk_list) {
        auto it = std::find(crt_.sk.begin(), crt_.sk.end(), sk_star);
        if (it != crt_.sk.end()) {
            crt_.sk.erase(it);
            
            mpz_class M_star = crt_.M / sk_star;
            mpz_class y_star = invertMPZ(M_star, sk_star);
            crt_.u -= M_star * y_star;
            crt_.M = M_star;
        }
    }
    
    pp_.u_s = sanitizer_sk * crt_.u;
}

void DSSMSSignature::initRandomGenerator() {
    // 初始化Miracl Core随机数生成器
    char raw[100];
    octet RAW = {0, sizeof(raw), raw};
    unsigned long ran;
    time((time_t *) &ran);
    
    RAW.len = 100;
    RAW.val[0] = ran;
    RAW.val[1] = ran >> 8;
    RAW.val[2] = ran >> 16;
    RAW.val[3] = ran >> 24;
    for (int i = 4; i < 100; i++) {
        RAW.val[i] = i;
    }
    
    CREATE_CSPRNG((csprng*)&rng_, &RAW);
    
    // 初始化GMP随机数生成器
    gmp_randinit_default(gmp_state_);
    gmp_randseed_ui(gmp_state_, ran);
    rng_state_ = &gmp_state_;
}

// ECPoint相关辅助函数 - 使用Miracl Core实现
ECPoint DSSMSSignature::ecPointMultiply(const ECPoint& P, const mpz_class& k) {
    ECPoint result;
    BIG k_big;
    mpz_to_BIG(k, k_big);
    ECP_copy(&result.get(), const_cast<ECP*>(&P.get()));
    ECP_mul(&result.get(), k_big);
    return result;
}

ECPoint DSSMSSignature::ecPointAdd(const ECPoint& P, const ECPoint& Q) {
    ECPoint result;
    ECP_copy(&result.get(), const_cast<ECP*>(&P.get()));
    ECP_add(&result.get(), const_cast<ECP*>(&Q.get()));  // result = result + Q
    return result;
}

ECPoint DSSMSSignature::ecPointNegate(const ECPoint& P) {
    ECPoint result;
    ECP_copy(&result.get(), const_cast<ECP*>(&P.get()));
    ECP_neg(&result.get());
    return result;
}

bool DSSMSSignature::ecPointEqual(const ECPoint& P, const ECPoint& Q) {
    return ECP_equals(const_cast<ECP*>(&P.get()), const_cast<ECP*>(&Q.get())) != 0;
}

// ECPoint序列化和反序列化实现
std::string ECPoint::serialize() const {
    char octet_buf[100];
    octet oct = {0, sizeof(octet_buf), octet_buf};
    ECP_toOctet(&oct, (ECP*)&ecp_, true);  // true表示压缩格式
    std::ostringstream oss;
    for (int i = 0; i < oct.len; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(static_cast<unsigned char>(oct.val[i]));
    }
    return oss.str();
}

bool DSSMSSignature::setP(const std::string& P_hex) {
    ECPoint P_new;
    if (!P_new.deserialize(P_hex)) {
        std::cerr << "[DSS-MS] 无法反序列化P参数" << std::endl;
        return false;
    }
    pp_.P = P_new;
    return true;
}

bool ECPoint::deserialize(const std::string& data) {
    if (data.length() % 2 != 0) {
        std::cerr << "[ECPoint::deserialize] 数据长度不是偶数: " << data.length() << std::endl;
        return false;
    }
    
    size_t byte_len = data.length() / 2;
    
    // Ed25519格式：对于Ed25519曲线，Miracl库支持多种格式：
    // 1. 32字节格式：32字节原始Ed25519公钥（Ed25519标准格式）- 直接支持
    // 2. 33字节格式：1字节前缀（0x02或0x03）+ 32字节坐标（标准压缩格式）
    // 3. 49字节格式：serialize()可能输出的格式（未压缩或其他格式）
    
    // 如果数据长度是32字节，直接使用（Ed25519原始格式）
    if (byte_len == 32) {
        // 直接使用32字节原始Ed25519公钥
    } else if (byte_len == 33) {
        // 33字节压缩格式
    } else if (byte_len > 33) {
        // 如果数据长度大于33字节，使用前33字节（标准压缩格式）
        // serialize()可能输出了49字节，但ECP_fromOctet期望33字节
        byte_len = 33;
    } else {
        std::cerr << "[ECPoint::deserialize] 错误：数据长度 " << byte_len 
                  << " 字节，不支持的长度（期望32或33字节）" << std::endl;
        return false;
    }
    
    char* octet_buf = new char[byte_len + 10];
    for (size_t i = 0; i < byte_len; ++i) {
        std::string byte_str = data.substr(2 * i, 2);
        char* endptr;
        long val = strtol(byte_str.c_str(), &endptr, 16);
        if (*endptr != '\0' || val < 0 || val > 255) {
            std::cerr << "[ECPoint::deserialize] 无效的十六进制字节: " << byte_str << std::endl;
            delete[] octet_buf;
            return false;
        }
        octet_buf[i] = static_cast<char>(val);
    }
    
    octet oct = {0, static_cast<int>(byte_len), octet_buf};
    int result = ECP_fromOctet((ECP*)&ecp_, &oct);
    
    // 在删除缓冲区之前保存前缀字节用于错误报告
    unsigned char prefix_byte = (byte_len > 0) ? static_cast<unsigned char>(octet_buf[0]) : 0;
    delete[] octet_buf;
    
    if (result == 0) {
        std::cerr << "[ECPoint::deserialize] ECP_fromOctet返回失败，数据长度: " << byte_len 
                  << " 字节，前缀: 0x" << std::hex << static_cast<int>(prefix_byte) << std::dec << std::endl;
        return false;
    }
    
    return true;
}

} // namespace drone_control
