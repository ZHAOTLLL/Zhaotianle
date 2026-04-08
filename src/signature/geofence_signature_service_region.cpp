/**
 * GeofenceSignatureService 区域签名实现
 */
#include "signature/geofence_signature_service.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <openssl/rand.h>
#include <openssl/evp.h>

#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

namespace drone_control {

std::string GeofenceSignatureService::generateGeofenceDataForEntireRegion(
    uint64_t validity_period,
    uint32_t drone_id,
    const std::string& region_id) {
    if (!grid_mapper_.isInitialized()) {
        std::cout << "[GeofenceSignatureService] [FCSACM-特权] 初始化网格映射器..." << std::endl;
        if (!grid_mapper_.initialize(31.75229, 31.78575, 117.16185, 117.22901, 8)) {
            std::cerr << "[generateGeofenceDataForEntireRegion] 网格映射器初始化失败" << std::endl;
            return "";
        }
    }

    std::vector<uint32_t> all_fence_ids;
    all_fence_ids.reserve(64);
    for (uint32_t i = 101; i <= 164; ++i) {
        all_fence_ids.push_back(i);
    }

#ifdef HAVE_NLOHMANN_JSON
    try {
        json j;
        j["geofence"]["region_id"] = region_id;
        j["geofence"]["timestamp"] = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        j["geofence"]["validity_period"] = validity_period;
        j["geofence"]["ta_cert_hash"] = "";
        j["geofence"]["drone_id"] = drone_id;
        j["geofence"]["fence_id_list"] = all_fence_ids;

        std::ostringstream sig_data_oss;
        sig_data_oss << region_id << "|" << j["geofence"]["timestamp"].get<uint64_t>()
                     << "|" << validity_period << "|" << drone_id << "|";
        for (size_t i = 0; i < all_fence_ids.size(); ++i) {
            if (i > 0) sig_data_oss << ",";
            sig_data_oss << all_fence_ids[i];
        }
        std::string sig_data = sig_data_oss.str();

        BIG private_key, order;
        ECP public_key, generator;
        BIG_rcopy(order, CURVE_Order);
        char random_bytes[32];
        if (RAND_bytes(reinterpret_cast<unsigned char*>(random_bytes), 32) != 1) {
            return "";
        }
        DBIG dx;
        BIG_dfromBytesLen(dx, random_bytes, 32);
        BIG_ctdmod(private_key, dx, order, 8 * 32 - BIG_nbits(order));
        ECP_generator(&generator);
        ECP_copy(&public_key, &generator);
        ECP_mul(&public_key, private_key);
        ECP_affine(&public_key);

        char pk_bytes[2 * MODBYTES_B256_56 + 1];
        octet pk_oct = {0, sizeof(pk_bytes), pk_bytes};
        ECP_toOctet(&pk_oct, &public_key, true);
        std::ostringstream pk_hex_oss;
        for (int i = 0; i < pk_oct.len; ++i) {
            pk_hex_oss << std::hex << std::setw(2) << std::setfill('0')
                       << static_cast<int>(static_cast<unsigned char>(pk_bytes[i]));
        }

        unsigned char hash[SHA256_DIGEST_LENGTH];
        EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
        if (!mdctx) {
            return "";
        }
        if (EVP_DigestInit_ex(mdctx, EVP_sha256(), nullptr) != 1 ||
            EVP_DigestUpdate(mdctx, sig_data.c_str(), sig_data.length()) != 1 ||
            EVP_DigestFinal_ex(mdctx, hash, nullptr) != 1) {
            EVP_MD_CTX_free(mdctx);
            return "";
        }
        EVP_MD_CTX_free(mdctx);

        BIG hash_big;
        BIG_fromBytesLen(hash_big, reinterpret_cast<char*>(hash), SHA256_DIGEST_LENGTH);
        BIG_mod(hash_big, order);
        BIG k, k_inv, r, s, temp1, temp2;
        ECP kG;
        csprng rng;
        char seed[32];
        if (RAND_bytes(reinterpret_cast<unsigned char*>(seed), 32) != 1) {
            return "";
        }
        RAND_seed(&rng, 32, seed);
        BIG_randomnum(k, order, &rng);
        ECP_copy(&kG, &generator);
        ECP_mul(&kG, k);
        ECP_affine(&kG);
        BIG y_temp;
        ECP_get(r, y_temp, &kG);
        BIG_mod(r, order);
        BIG_modmul(temp1, r, private_key, order);
        BIG_modadd(temp2, hash_big, temp1, order);
        BIG_invmodp(k_inv, k, order);
        BIG_modmul(s, k_inv, temp2, order);

        char sig_bytes[2 * MODBYTES_B256_56];
        BIG_toBytes(sig_bytes, r);
        BIG_toBytes(sig_bytes + MODBYTES_B256_56, s);
        std::ostringstream sig_hex_oss;
        for (int i = 0; i < 2 * MODBYTES_B256_56; ++i) {
            sig_hex_oss << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(sig_bytes[i]));
        }
        j["signature"]["type"] = "Ed25519-ECDSA";
        j["signature"]["data"] = sig_hex_oss.str();
        j["signature"]["message"] = sig_data;
        j["public_keys"]["gcs_pk"] = pk_hex_oss.str();
        return j.dump();
    } catch (const std::exception& e) {
        std::cerr << "[GeofenceSignatureService] [FCSACM-特权] 异常: " << e.what() << std::endl;
        return "";
    }
#else
    return "";
#endif
}

} // namespace drone_control
