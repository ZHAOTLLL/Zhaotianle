/**
 * MAVLinkManager 任务消息接入实现
 * 处理任务目的上传、证书上传、消息订阅和证书文件读取流程。
 */
#include "communication/mavlink_manager.hpp"
#include "access_control/access_request.hpp"
#include "state/drone_state_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <sys/stat.h>

namespace drone_control {

void MAVLinkManager::handleMissionPurposeUpload(
    uint32_t drone_id,
    const std::string& mission_name,
    const std::string& target_location,
    const std::string& operation_type,
    const std::string& certificate_hash,
    const std::string& mission_description,
    uint64_t /*timestamp*/)
{
    MissionPurposeCache cache_entry;
    cache_entry.mission_name = mission_name;
    cache_entry.target_location = target_location;
    cache_entry.operation_type = operation_type;
    cache_entry.certificate_hash = certificate_hash;
    cache_entry.mission_description = mission_description;
    cache_entry.last_evaluated = std::chrono::steady_clock::time_point::min();
    cache_entry.last_decision_granted = false;
    cache_entry.last_decision_reason.clear();

    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        mission_purpose_cache_[drone_id] = cache_entry;
    }

#ifdef HAVE_MAVSDK
    sendMissionConfirmation(drone_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    requestCertificateUpload(drone_id);
#else
    sendAccessControlResult(drone_id, false, "MAVSDK unavailable");
#endif
}

bool MAVLinkManager::confirmMissionAndRequestCertificate(uint32_t drone_id) {
#ifdef HAVE_MAVSDK
    if (!isDroneConnected(drone_id)) {
        std::cout << "[MAVLink] 无人机 " << drone_id << " 未连接，无法确认任务" << std::endl;
        return false;
    }
    sendMissionConfirmation(drone_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    requestCertificateUpload(drone_id);
    std::cout << "[MAVLink] 已向无人机 " << drone_id << " 发送任务确认并请求证书" << std::endl;
    return true;
#else
    (void)drone_id;
    return false;
#endif
}

AccessRequest MAVLinkManager::constructAccessRequest(
    uint32_t drone_id,
    const std::string& mission_name,
    const std::string& target_location,
    const std::string& operation_type,
    const std::string& certificate_hash,
    const std::string& mission_description)
{
    AccessRequest request;
    request.drone_id = drone_id;
    request.request_time = std::chrono::system_clock::now();
    request.request_source = "mission_purpose_upload";
    request.target_location = target_location;
    request.operation_type = operation_type;

    if (state_manager_) {
        auto drone_state = state_manager_->getDroneState(drone_id);
        if (drone_state) {
            request.context["drone_model"] = drone_state->drone_model;
            request.context["owner_organization"] = drone_state->owner_organization;
            request.context["is_privileged"] = drone_state->is_privileged ? "true" : "false";
            request.context["battery_percentage"] = std::to_string(drone_state->battery_percentage);
            request.context["flight_status"] = std::to_string(static_cast<int>(drone_state->flight_status));
            request.context["current_latitude"] = std::to_string(drone_state->position.latitude);
            request.context["current_longitude"] = std::to_string(drone_state->position.longitude);
            request.context["current_altitude"] = std::to_string(drone_state->position.altitude);
        }
    }

    if (!certificate_hash.empty()) {
        request.context["certificate_hash"] = certificate_hash;
    }
    request.context["mission_name"] = mission_name;
    request.context["mission_description"] = mission_description;

    if (operation_type == "delivery") {
        request.context["operation_category"] = "commercial";
        request.context["payload_type"] = "package";
    } else if (operation_type == "patrol") {
        request.context["operation_category"] = "surveillance";
        request.context["payload_type"] = "camera";
    } else if (operation_type == "emergency") {
        request.context["operation_category"] = "emergency";
        request.context["payload_type"] = "medical";
    } else {
        request.context["operation_category"] = "general";
        request.context["payload_type"] = "unknown";
    }

    std::cout << "[AccessControl] 构造访问请求，drone=" << request.drone_id
              << ", operation=" << request.operation_type << ", ctx=" << request.context.size()
              << std::endl;
    return request;
}

void MAVLinkManager::setupMavlinkMessageHandlers(DroneConnection& conn) {
#ifdef HAVE_MAVSDK
    if (!conn.mavlink_passthrough) {
        std::cout << "[错误] MAVLink passthrough plugin not available for drone " << conn.drone_id
                  << std::endl;
        return;
    }
#ifdef USE_BOOST_ASIO
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_STATUSTEXT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            if (io_context_) {
                boost::asio::post(*io_context_, [this, drone_id, message]() {
                    handleMavlinkMessage(drone_id, message);
                });
            } else {
                handleMavlinkMessage(drone_id, message);
            }
        });
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            (void)drone_id;
            (void)message;
        });
    conn.mavlink_passthrough->subscribe_message(
        200,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            if (io_context_) {
                boost::asio::post(*io_context_, [this, drone_id, message]() {
                    handleMavlinkMessage(drone_id, message);
                });
            } else {
                handleMavlinkMessage(drone_id, message);
            }
        });
    conn.mavlink_passthrough->subscribe_message(
        12921,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            std::cout << "[MAVLink] 收到消息ID 12921 (MISSION_PURPOSE_UPLOAD)" << std::endl;
            if (io_context_) {
                boost::asio::post(*io_context_, [this, drone_id, message]() {
                    handleMavlinkMessage(drone_id, message);
                });
            } else {
                handleMavlinkMessage(drone_id, message);
            }
        });
    std::cout << "[MAVLink] 已在无人机 " << conn.drone_id
              << " 上注册消息回调（Boost.Asio异步处理）" << std::endl;
#else
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_STATUSTEXT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            handleMavlinkMessage(drone_id, message);
        });
    conn.mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            (void)drone_id;
            (void)message;
        });
    conn.mavlink_passthrough->subscribe_message(
        200,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            handleMavlinkMessage(drone_id, message);
        });
    conn.mavlink_passthrough->subscribe_message(
        12921,
        [this, drone_id = conn.drone_id](const mavlink_message_t& message) {
            std::cout << "[MAVLink] 收到消息ID 12921 (MISSION_PURPOSE_UPLOAD)" << std::endl;
            handleMavlinkMessage(drone_id, message);
        });
    std::cout << "[MAVLink] 已在无人机 " << conn.drone_id << " 上注册消息回调" << std::endl;
#endif
#else
    std::cout << "[MAVLink] MAVSDK 不可用，无法注册消息回调" << std::endl;
#endif
}

void MAVLinkManager::handleMavlinkMessage(DroneId drone_id, const mavlink_message_t& message) {
#ifdef HAVE_MAVSDK
    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t statustext;
        mavlink_msg_statustext_decode(&message, &statustext);
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto cache_it = mission_purpose_cache_.find(drone_id);
            if (cache_it != mission_purpose_cache_.end() && !cache_it->second.certificate_content.empty()) {
                return;
            }
        }
        std::string current_text(statustext.text);
        auto now = std::chrono::steady_clock::now();
        if (current_text.find("MISSION_PURPOSE:") == 0) {
            std::vector<std::string> parts;
            std::stringstream ss(current_text);
            std::string item;
            while (std::getline(ss, item, ':')) {
                parts.push_back(item);
            }
            if (parts.size() >= 4) {
                MissionPurposeCache cache_entry;
                cache_entry.mission_name = parts.size() > 1 ? parts[1] : "";
                cache_entry.target_location = parts.size() > 2 ? parts[2] : "";
                cache_entry.operation_type = parts.size() > 3 ? parts[3] : "";
                cache_entry.certificate_hash = parts.size() > 4 ? parts[4] : "government_drone_001";
                cache_entry.mission_description =
                    "无人机前往" + cache_entry.target_location + "执行" + cache_entry.operation_type + "任务";
                cache_entry.last_evaluated = std::chrono::steady_clock::now();
                cache_entry.last_decision_granted = false;
                cache_entry.last_decision_reason = "";
                {
                    std::lock_guard<std::mutex> lock(connections_mutex_);
                    mission_purpose_cache_[drone_id] = cache_entry;
                    statustext_buffers_.erase(drone_id);
                }
                std::cout << "[MAVLink] 收到无人机 " << drone_id << " 的任务目的，目标: "
                          << cache_entry.target_location << std::endl;
                auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count();
                handleMissionPurposeUpload(
                    drone_id,
                    cache_entry.mission_name,
                    cache_entry.target_location,
                    cache_entry.operation_type,
                    cache_entry.certificate_hash,
                    cache_entry.mission_description,
                    current_time);
                return;
            }
        }
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            auto& buffer = statustext_buffers_[drone_id];
            buffer.buffer += current_text;
            buffer.last_update = now;
        }
        std::string full_message;
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            full_message = statustext_buffers_[drone_id].buffer;
        }
        if (full_message.find("MISSION_PURPOSE:") == 0) {
            std::vector<std::string> parts;
            std::stringstream ss(full_message);
            std::string item;
            while (std::getline(ss, item, ':')) {
                parts.push_back(item);
            }
            if (parts.size() >= 4) {
                MissionPurposeCache cache_entry;
                cache_entry.mission_name = parts.size() > 1 ? parts[1] : "";
                cache_entry.target_location = parts.size() > 2 ? parts[2] : "";
                cache_entry.operation_type = parts.size() > 3 ? parts[3] : "";
                cache_entry.certificate_hash = parts.size() > 4 ? parts[4] : "government_drone_001";
                cache_entry.mission_description =
                    "无人机前往" + cache_entry.target_location + "执行" + cache_entry.operation_type + "任务";
                cache_entry.last_evaluated = std::chrono::steady_clock::now();
                cache_entry.last_decision_granted = false;
                cache_entry.last_decision_reason = "";
                {
                    std::lock_guard<std::mutex> lock(connections_mutex_);
                    mission_purpose_cache_[drone_id] = cache_entry;
                    statustext_buffers_.erase(drone_id);
                }
                std::cout << "[MAVLink] 收到无人机 " << drone_id << " 的任务目的，目标: "
                          << cache_entry.target_location << std::endl;
                auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count();
                handleMissionPurposeUpload(
                    drone_id,
                    cache_entry.mission_name,
                    cache_entry.target_location,
                    cache_entry.operation_type,
                    cache_entry.certificate_hash,
                    cache_entry.mission_description,
                    current_time);
                return;
            }
        }
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            if (mission_purpose_cache_.find(drone_id) != mission_purpose_cache_.end()) {
                return;
            }
        }
        return;
    }
    if (message.msgid == 12921) {
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            if (mission_purpose_cache_.find(drone_id) != mission_purpose_cache_.end()) {
                return;
            }
        }
        const uint8_t* payload = (const uint8_t*)message.payload64;
        uint64_t timestamp = 0;
        memcpy(&timestamp, payload, 8);
        payload += 8;
        uint32_t mission_drone_id = 0;
        memcpy(&mission_drone_id, payload, 4);
        payload += 4;
        char mission_name[33] = {0};
        char target_location[33] = {0};
        char operation_type[17] = {0};
        char certificate_hash[65] = {0};
        char mission_description[65] = {0};
        strncpy(mission_name, (const char*)payload, 32);
        payload += 32;
        strncpy(target_location, (const char*)payload, 32);
        payload += 32;
        strncpy(operation_type, (const char*)payload, 16);
        payload += 16;
        strncpy(certificate_hash, (const char*)payload, 64);
        payload += 64;
        strncpy(mission_description, (const char*)payload, 64);
        handleMissionPurposeUpload(
            mission_drone_id,
            std::string(mission_name),
            std::string(target_location),
            std::string(operation_type),
            std::string(certificate_hash),
            std::string(mission_description),
            timestamp);
    } else if (message.msgid == 200) {
        const uint8_t* payload = (const uint8_t*)message.payload64;
        uint32_t cert_drone_id = 0;
        uint32_t certificate_size = 0;
        uint16_t chunk_index = 0;
        uint16_t total_chunks = 0;
        uint64_t timestamp = 0;
        char certificate_data[201] = {0};
        memcpy(&cert_drone_id, payload, 4);
        payload += 4;
        memcpy(&certificate_size, payload, 4);
        payload += 4;
        memcpy(&chunk_index, payload, 2);
        payload += 2;
        memcpy(&total_chunks, payload, 2);
        payload += 2;
        memcpy(certificate_data, payload, 200);
        payload += 200;
        memcpy(&timestamp, payload, 8);
        handleCertificateUpload(
            cert_drone_id,
            certificate_size,
            chunk_index,
            total_chunks,
            std::string(certificate_data),
            timestamp);
    } else {
        std::cout << "📨 收到其他MAVLink消息: ID=" << message.msgid << " from drone " << drone_id
                  << std::endl;
    }
#else
    (void)drone_id;
    (void)message;
    std::cout << "[警告] MAVSDK 不可用，无法处理 MAVLink 消息" << std::endl;
#endif
}

void MAVLinkManager::handleCertificateUpload(
    uint32_t drone_id,
    uint32_t certificate_size,
    uint16_t chunk_index,
    uint16_t total_chunks,
    const std::string& certificate_data,
    uint64_t timestamp)
{
    std::cout << "处理证书上传消息:" << std::endl;
    std::cout << "   无人机ID: " << drone_id << std::endl;
    std::cout << "   📏 证书大小: " << certificate_size << " 字节" << std::endl;
    std::cout << "   分块信息: " << (chunk_index + 1) << "/" << total_chunks << std::endl;
    std::cout << "   时间戳: " << timestamp << std::endl;
    std::cout << "   当前分块数据长度: " << certificate_data.length() << " 字节" << std::endl;
    std::cout << "   当前分块数据预览: "
              << certificate_data.substr(0, std::min(50, (int)certificate_data.length())) << "..."
              << std::endl;

    if (chunk_index == 0) {
        std::cout << "   🆕 初始化证书缓存，无人机ID: " << drone_id << std::endl;
        certificate_cache_[drone_id] = std::string();
        certificate_cache_[drone_id].reserve(certificate_size);
        std::cout << "   证书缓存已初始化，预留空间: " << certificate_size << " 字节" << std::endl;
    }

    auto cache_it = certificate_cache_.find(drone_id);
    if (cache_it == certificate_cache_.end()) {
        std::cout << "[错误] 证书缓存不存在，忽略分块 " << chunk_index << std::endl;
        return;
    }

    std::cout << "   添加分块数据到缓存，当前缓存大小: " << cache_it->second.length() << " 字节"
              << std::endl;
    cache_it->second += certificate_data;
    std::cout << "   添加后缓存大小: " << cache_it->second.length() << " 字节" << std::endl;

    if (chunk_index == total_chunks - 1) {
        if (cache_it->second.length() != certificate_size) {
            certificate_cache_.erase(cache_it);
            std::cout << "[MAVLink] 证书大小不匹配，无法验证" << std::endl;
            return;
        }
        std::cout << "[MAVLink] 无人机 " << drone_id << " 证书已完整接收，开始解码验证" << std::endl;

        std::string decoded_cert = base64Decode(cache_it->second);
        if (decoded_cert.empty()) {
            std::cout << "[MAVLink] 证书解码失败" << std::endl;
            certificate_cache_.erase(cache_it);
            return;
        }

        if (auth_provider_) {
            AuthenticationRequest request;
            request.credential = decoded_cert;
            request.credential_type = "x509_pem";
            request.drone_id = std::to_string(drone_id);

            AuthenticationResult result = auth_provider_->authenticate(request);
            if (result.success) {
                auto conn_it = connections_.find(drone_id);
                if (conn_it != connections_.end() && conn_it->second) {
                    conn_it->second->authentication_credential = decoded_cert;
                    conn_it->second->credential_type = "x509_pem";
                    conn_it->second->extracted_attributes = result.attributes;
                    conn_it->second->is_authenticated = true;
                    conn_it->second->status = ConnectionStatus::AUTHENTICATED;
                    if (result.getAttribute("organization") == "government" ||
                        result.getAttribute("organization") == "police" ||
                        result.getAttribute("organization") == "emergency") {
                        conn_it->second->state.is_privileged = true;
                    }
                    conn_it->second->state.owner_organization = result.getAttribute("organization");
                    conn_it->second->state.is_authenticated = true;
                    conn_it->second->state.authentication_type = conn_it->second->credential_type;
                    conn_it->second->state.trust_level = result.trust_level;
                    auto cert_it = result.attributes.find("certificate_id");
                    if (cert_it != result.attributes.end()) {
                        conn_it->second->state.certificate_id = cert_it->second;
                    }
                    conn_it->second->state.auth_timestamp = std::chrono::system_clock::now();
                    conn_it->second->state.auth_attributes = result.attributes;
                    for (auto& callback : connection_callbacks_) {
                        callback(drone_id, ConnectionStatus::AUTHENTICATED);
                    }
                    if (state_manager_) {
                        state_manager_->updateDroneState(drone_id, conn_it->second->state);
                    }
                }

                auto mission_it = mission_purpose_cache_.find(drone_id);
                if (mission_it != mission_purpose_cache_.end()) {
                    performAccessControlEvaluation(drone_id);
                } else {
                    std::cout << "[AccessControl] 无任务目的缓存，跳过评估" << std::endl;
                }

                validated_certificates_[drone_id] = {
                    decoded_cert, std::chrono::system_clock::now(), result.attributes};
            } else {
                std::cout << "[MAVLink] 证书验证失败: " << result.error_message << std::endl;
            }
        } else {
            std::cout << "[MAVLink] 未配置认证提供者，跳过证书验证" << std::endl;
        }
        certificate_cache_.erase(cache_it);
    }
}

std::string MAVLinkManager::base64Decode(const std::string& encoded) {
    const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string decoded;
    int val = 0, valb = -8;
    for (char c : encoded) {
        if (chars.find(c) == std::string::npos) {
            break;
        }
        val = (val << 6) + chars.find(c);
        valb += 6;
        if (valb >= 0) {
            decoded.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return decoded;
}

void MAVLinkManager::downloadCertificateViaFTP(DroneId drone_id) {
    downloadCertificateViaFTPWithRetry(drone_id, 1);
}

void MAVLinkManager::downloadCertificateViaFTPWithRetry(DroneId drone_id, int max_retries) {
    std::cout << "[MAVLink] 开始通过FTP下载无人机 " << drone_id << " 的证书文件（最多重试 "
              << max_retries << " 次）..." << std::endl;
#ifdef HAVE_MAVSDK
    const std::string remote_file_path = "./fs/microsd/certificates/drone_cert.pem";
    const std::string local_file_path = "/tmp/drone_" + std::to_string(drone_id) + "_cert.pem";
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        std::cout << "[MAVLink] FTP下载尝试 " << attempt << "/" << max_retries << "..." << std::endl;
        std::unique_lock<std::mutex> lock(connections_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            std::cout << "[MAVLink] 无法获取连接锁，等待后重试..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            lock = std::unique_lock<std::mutex>(connections_mutex_, std::try_to_lock);
            if (!lock.owns_lock()) {
                std::cout << "[MAVLink] 重试后仍无法获取连接锁" << std::endl;
                if (attempt < max_retries) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }
                std::cout << "[MAVLink] 达到最大重试次数，放弃FTP下载" << std::endl;
                return;
            }
        }
        auto it = connections_.find(drone_id);
        if (it == connections_.end()) {
            std::cout << "[MAVLink] 无人机 " << drone_id << " 连接不存在" << std::endl;
            return;
        }
        if (!it->second->ftp) {
            std::cout << "[MAVLink] 无人机 " << drone_id << " FTP 未初始化" << std::endl;
            return;
        }
        auto* ftp_plugin = it->second->ftp.get();
        lock.unlock();

        std::cout << "[MAVLink] 尝试下载证书文件: " << remote_file_path << " -> " << local_file_path
                  << std::endl;
        std::atomic<bool> download_complete{false};
        std::atomic<bool> download_success{false};

        ftp_plugin->download_async(
            remote_file_path,
            local_file_path,
            false,
            [&download_complete, &download_success, drone_id, local_file_path, this](
                mavsdk::Ftp::Result result, mavsdk::Ftp::ProgressData progress) {
                (void)progress;
                if (result == mavsdk::Ftp::Result::Success) {
                    std::cout << "[MAVLink] 无人机 " << drone_id << " 证书文件下载成功" << std::endl;
                    download_success = true;
                    download_complete = true;
                    handleCertificateFileDownloaded(drone_id, local_file_path, true);
                } else if (
                    result == mavsdk::Ftp::Result::ProtocolError ||
                    result == mavsdk::Ftp::Result::InvalidParameter ||
                    result == mavsdk::Ftp::Result::NoSystem) {
                    std::cout << "[MAVLink] 证书文件下载失败（不可恢复错误），result="
                              << static_cast<int>(result) << std::endl;
                    download_success = false;
                    download_complete = true;
                    handleCertificateFileDownloaded(drone_id, local_file_path, false);
                } else {
                    std::cout << "[MAVLink] 证书文件下载失败（可重试），result="
                              << static_cast<int>(result) << std::endl;
                    download_success = false;
                    download_complete = true;
                }
            });

        auto start_time = std::chrono::steady_clock::now();
        while (!download_complete) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(10)) {
                std::cout << "[MAVLink] FTP下载超时（10秒）" << std::endl;
                break;
            }
        }
        if (download_success) {
            std::cout << "[MAVLink] 证书下载成功完成" << std::endl;
            return;
        }
        if (attempt < max_retries) {
            std::cout << "[MAVLink] 等待 " << (attempt * 2) << " 秒后重试..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(attempt * 2));
        }
    }
    std::cout << "[MAVLink] 达到最大重试次数，证书下载失败" << std::endl;
    handleCertificateFileDownloaded(drone_id, local_file_path, false);
#else
    (void)drone_id;
    (void)max_retries;
    std::cout << "[MAVLink] MAVSDK 不可用，无法执行FTP下载" << std::endl;
#endif
}

void MAVLinkManager::handleCertificateFileDownloaded(
    DroneId drone_id, const std::string& file_path, bool success) {
    if (!success) {
        std::cout << "[MAVLink] 无人机 " << drone_id << " 的证书文件下载失败" << std::endl;
        return;
    }
    std::string actual_file_path = file_path;
    struct stat path_stat;
    if (stat(file_path.c_str(), &path_stat) == 0 && S_ISDIR(path_stat.st_mode)) {
        actual_file_path = file_path + "/drone_cert.pem";
    }

    std::ifstream cert_file(actual_file_path, std::ios::binary);
    if (!cert_file.is_open()) {
        std::cout << "[MAVLink] 无法打开证书文件: " << actual_file_path << std::endl;
        return;
    }
    std::string certificate_data(
        (std::istreambuf_iterator<char>(cert_file)), std::istreambuf_iterator<char>());
    cert_file.close();
    if (certificate_data.empty()) {
        std::cout << "[MAVLink] 证书文件为空" << std::endl;
        return;
    }
    if (auto mission_it = mission_purpose_cache_.find(drone_id); mission_it != mission_purpose_cache_.end()) {
        mission_it->second.certificate_content = certificate_data;
        std::cout << "[MAVLink] 已存储无人机 " << drone_id << " 的证书内容，准备评估" << std::endl;
    }
    performAccessControlEvaluation(drone_id);
}

} // namespace drone_control

