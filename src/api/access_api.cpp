/**
 * 访问评估 API 实现
 * 解析 JSON 请求体为访问请求，调用访问控制引擎评估并返回决策 JSON。
 */
#include "access_api.hpp"
#include "access_control/access_control_engine.hpp"
#include "access_control/access_request.hpp"
#include "access_control/access_decision.hpp"
#include <sstream>
#include <iostream>
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

namespace drone_control {
namespace api {

namespace {

std::string escapeForJson(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else out += c;
    }
    return out;
}

std::string buildDecisionResponse(const AccessDecision& decision) {
    std::ostringstream response;
    response << "{"
             << "\"decision\": \"" << (decision.granted ? "PERMIT" : "DENY") << "\","
             << "\"reason\": \"" << escapeForJson(decision.reason) << "\","
             << "\"decision_id\": \"" << escapeForJson(decision.decision_id) << "\","
             << "\"access_level\": \"" << static_cast<int>(decision.level) << "\","
             << "\"constraints\": [";
    bool first = true;
    for (const auto& c : decision.restricted_capabilities) {
        if (!first) response << ",";
        response << "\"" << escapeForJson(c) << "\"";
        first = false;
    }
    response << "],\"required_actions\": [";
    first = true;
    for (const auto& a : decision.required_actions) {
        if (!first) response << ",";
        response << "\"" << escapeForJson(a) << "\"";
        first = false;
    }
    response << "],\"applied_policies\": [";
    first = true;
    for (const auto& p : decision.applied_policies) {
        if (!first) response << ",";
        response << "\"" << escapeForJson(p) << "\"";
        first = false;
    }
    response << "],\"validity_duration\": " << decision.validity_duration.count() << ","
             << "\"signature\": \"" << escapeForJson(decision.geofence_signature) << "\"}";
    return response.str();
}

std::string buildEvaluationErrorResponse(const std::string& message) {
    return "{\"error\": \"Access evaluation failed: " + escapeForJson(message) + "\",\"decision\": \"DENY\"}";
}

} // namespace

AccessAPI::AccessAPI(std::shared_ptr<AccessControlEngine> access_engine)
    : access_engine_(access_engine) {
}

std::string AccessAPI::handleAccessEvaluateWithBody(const std::string& path, const std::string& method, const std::string& body) {
    if (method != "POST") return ResponseUtils::createErrorResponse("Method not allowed");
    if (!access_engine_) return ResponseUtils::createErrorResponse("Access control engine not available");
    try {
        AccessRequest request = parseAccessRequestFromJson(body);
        AccessDecision decision = access_engine_->evaluateAccess(request);
        return buildDecisionResponse(decision);
    } catch (const std::exception& e) {
        return buildEvaluationErrorResponse(e.what());
    }
}

AccessRequest AccessAPI::parseAccessRequestFromJson(const std::string& json_body) {
    AccessRequest request;
    
#ifdef HAVE_NLOHMANN_JSON
    // 使用nlohmann/json库解析JSON
    try {
        json j = json::parse(json_body);
        
        // drone_id是必需字段，必须从请求中提取，不能使用默认值
        if (!j.contains("drone_id")) {
            throw std::runtime_error("请求中缺少必需字段: drone_id");
        }
        request.drone_id = j["drone_id"].get<int>();
        if (request.drone_id <= 0) {
            throw std::runtime_error("无效的drone_id: " + std::to_string(request.drone_id));
        }
        if (j.contains("target_location")) {
            request.target_location = j["target_location"].get<std::string>();
        }
        if (j.contains("operation_type")) {
            request.operation_type = j["operation_type"].get<std::string>();
        }
        if (j.contains("mission_signature")) {
            // mission_signature可能是字符串或JSON对象
            if (j["mission_signature"].is_string()) {
                request.context["mission_signature"] = j["mission_signature"].get<std::string>();
            } else if (j["mission_signature"].is_object()) {
                request.context["mission_signature"] = j["mission_signature"].dump();
            }
        }
        if (j.contains("certificate_data")) {
            request.context["credential"] = j["certificate_data"].get<std::string>();
            request.context["credential_type"] = "x509_base64";
        }
        if (j.contains("context")) {
            auto ctx = j["context"];
            if (ctx.is_object()) {
                if (ctx.contains("credential")) {
                    request.context["credential"] = ctx["credential"].get<std::string>();
                }
                if (ctx.contains("credential_type")) {
                    request.context["credential_type"] = ctx["credential_type"].get<std::string>();
                }
            }
        }
        
        request.request_time = std::chrono::system_clock::now();
        
        // 注意：属性应该从任务签名验证后获取，而不是硬编码
        // 如果请求中包含mission_signature，属性将在认证阶段从签名中提取
        // 这里不设置默认属性，让认证阶段从实际数据中提取
        
        return request;
    } catch (const std::exception& e) {
        std::cerr << "[错误] JSON解析失败: " << e.what() << std::endl;
        // 继续使用简单解析作为后备
    }
#endif
    
    // 无 nlohmann 时的简单后备解析
    
    // 解析drone_id（必需字段，不能使用默认值）
    size_t drone_id_pos = json_body.find("\"drone_id\":");
    if (drone_id_pos == std::string::npos) {
        throw std::runtime_error("请求中缺少必需字段: drone_id");
    }
    size_t start = json_body.find(":", drone_id_pos) + 1;
    size_t end = json_body.find_first_of(",}", start);
    if (start == std::string::npos || end == std::string::npos) {
        throw std::runtime_error("无法解析drone_id字段");
    }
    std::string drone_id_str = json_body.substr(start, end - start);
    // 去除空格
    drone_id_str.erase(0, drone_id_str.find_first_not_of(" \t"));
    drone_id_str.erase(drone_id_str.find_last_not_of(" \t") + 1);
    try {
        request.drone_id = std::stoi(drone_id_str);
        if (request.drone_id <= 0) {
            throw std::runtime_error("无效的drone_id: " + drone_id_str);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("无法解析drone_id: " + drone_id_str + " (" + e.what() + ")");
    }
    
    // 解析target_location
    size_t location_pos = json_body.find("\"target_location\":");
    if (location_pos != std::string::npos) {
        size_t start = json_body.find("\"", location_pos) + 1;
        size_t end = json_body.find("\"", start);
        if (start != std::string::npos && end != std::string::npos) {
            request.target_location = json_body.substr(start, end - start);
        }
    }
    
    // 解析operation_type
    size_t operation_pos = json_body.find("\"operation_type\":");
    if (operation_pos != std::string::npos) {
        size_t start = json_body.find("\"", operation_pos) + 1;
        size_t end = json_body.find("\"", start);
        if (start != std::string::npos && end != std::string::npos) {
            request.operation_type = json_body.substr(start, end - start);
        }
    }
    
    // 解析mission_signature（DSS方案）
    size_t mission_sig_pos = json_body.find("\"mission_signature\":");
    if (mission_sig_pos != std::string::npos) {
        size_t colon_pos = json_body.find(":", mission_sig_pos);
        if (colon_pos != std::string::npos) {
            // 跳过冒号和可能的空格
            size_t start = colon_pos + 1;
            while (start < json_body.length() && (json_body[start] == ' ' || json_body[start] == '\t')) {
                start++;
            }
            
            if (start < json_body.length()) {
                size_t end = start;
                
                // 如果是字符串（以引号开始）
                if (json_body[start] == '"') {
                    start++; // 跳过开始引号
                    size_t string_start = start; // 记录字符串开始位置
                    bool escaped = false;
                    while (end < json_body.length()) {
                        if (json_body[end] == '\\' && !escaped) {
                            escaped = true;
                            end++;
                            continue;
                        }
                        if (escaped) {
                            // 处理转义字符，包括\n
                            if (json_body[end] == 'n') {
                                // \n 转义，跳过
                            } else if (json_body[end] == 'r') {
                                // \r 转义，跳过
                            } else if (json_body[end] == 't') {
                                // \t 转义，跳过
                            } else if (json_body[end] == '\\') {
                                // \\ 转义，保留一个反斜杠
                            } else if (json_body[end] == '"') {
                                // \" 转义，保留引号
                            }
                            escaped = false;
                            end++;
                            continue;
                        }
                        if (json_body[end] == '"' && !escaped) {
                            break;
                        }
                        end++;
                    }
                    
                    // 提取字符串内容并处理转义
                    if (end > string_start) {
                        std::string mission_sig_raw = json_body.substr(string_start, end - string_start);
                        std::string mission_sig;
                        // 处理转义字符
                        for (size_t i = 0; i < mission_sig_raw.length(); i++) {
                            if (mission_sig_raw[i] == '\\' && i + 1 < mission_sig_raw.length()) {
                                if (mission_sig_raw[i + 1] == 'n') {
                                    mission_sig += '\n';
                                    i++; // 跳过n
                                } else if (mission_sig_raw[i + 1] == 'r') {
                                    mission_sig += '\r';
                                    i++; // 跳过r
                                } else if (mission_sig_raw[i + 1] == 't') {
                                    mission_sig += '\t';
                                    i++; // 跳过t
                                } else if (mission_sig_raw[i + 1] == '\\') {
                                    mission_sig += '\\';
                                    i++; // 跳过第二个反斜杠
                                } else if (mission_sig_raw[i + 1] == '"') {
                                    mission_sig += '"';
                                    i++; // 跳过引号
                                } else {
                                    mission_sig += mission_sig_raw[i];
                                }
                            } else {
                                mission_sig += mission_sig_raw[i];
                            }
                        }
                        request.context["mission_signature"] = mission_sig;
                    }
                } else if (json_body[start] == '{') {
                    // 如果是JSON对象，需要匹配大括号
                    int brace_count = 0;
                    bool in_string = false;
                    bool escaped = false;
                    
                    while (end < json_body.length()) {
                        if (json_body[end] == '\\' && !escaped) {
                            escaped = true;
                            end++;
                            continue;
                        }
                        if (escaped) {
                            escaped = false;
                            end++;
                            continue;
                        }
                        if (json_body[end] == '"' && !escaped) {
                            in_string = !in_string;
                        } else if (!in_string) {
                            if (json_body[end] == '{') {
                                brace_count++;
                            } else if (json_body[end] == '}') {
                                brace_count--;
                                if (brace_count == 0) {
                                    end++; // 包含结束大括号
                                    break;
                                }
                            }
                        }
                        end++;
                    }
                    
                    if (end > start) {
                        request.context["mission_signature"] = json_body.substr(start, end - start);
                    }
                }
            }
        }
    }
    
    // 解析certificate_data - 改进的JSON解析
    size_t cert_pos = json_body.find("\"certificate_data\":");
    if (cert_pos != std::string::npos) {
        // 找到冒号后的第一个引号
        size_t start = json_body.find("\"", cert_pos + 18); // 18 = "certificate_data":的长度
        if (start != std::string::npos) {
            start++; // 跳过开始的引号
            
            // 查找结束引号，处理转义字符
            size_t end = start;
            bool escaped = false;
            while (end < json_body.length()) {
                if (json_body[end] == '\\' && !escaped) {
                    escaped = true;
                } else if (json_body[end] == '"' && !escaped) {
                    break;
                } else {
                    escaped = false;
                }
                end++;
            }
            
            if (end < json_body.length()) {
                std::string cert_data = json_body.substr(start, end - start);
                
                // 处理JSON转义字符
                std::string processed_cert;
                for (size_t i = 0; i < cert_data.length(); i++) {
                    if (cert_data[i] == '\\' && i + 1 < cert_data.length()) {
                        if (cert_data[i + 1] == 'n') {
                            processed_cert += '\n';
                            i++; // 跳过下一个字符
                        } else if (cert_data[i + 1] == 'r') {
                            processed_cert += '\r';
                            i++; // 跳过下一个字符
                        } else if (cert_data[i + 1] == 't') {
                            processed_cert += '\t';
                            i++; // 跳过下一个字符
                        } else if (cert_data[i + 1] == '\\') {
                            processed_cert += '\\';
                            i++; // 跳过下一个字符
                        } else if (cert_data[i + 1] == '"') {
                            processed_cert += '"';
                            i++; // 跳过下一个字符
                        } else {
                            processed_cert += cert_data[i];
                        }
                    } else {
                        processed_cert += cert_data[i];
                    }
                }
                
                request.context["credential"] = processed_cert;
                request.context["credential_type"] = "x509_base64";
            }
        }
    }
    
    // 解析context
    size_t context_pos = json_body.find("\"context\":");
    if (context_pos != std::string::npos) {
        size_t start = json_body.find("{", context_pos);
        size_t end = json_body.find("}", start);
        if (start != std::string::npos && end != std::string::npos) {
            std::string context_str = json_body.substr(start + 1, end - start - 1);
            
            // 解析credential
            size_t cred_pos = context_str.find("\"credential\":");
            if (cred_pos != std::string::npos) {
                size_t colon_pos = context_str.find(":", cred_pos);
                if (colon_pos != std::string::npos) {
                    size_t cred_start = context_str.find("\"", colon_pos) + 1;
                    size_t cred_end = context_str.find("\"", cred_start);
                    if (cred_start != std::string::npos && cred_end != std::string::npos) {
                        request.context["credential"] = context_str.substr(cred_start, cred_end - cred_start);
                    }
                }
            }
            
            // 解析credential_type
            size_t type_pos = context_str.find("\"credential_type\":");
            if (type_pos != std::string::npos) {
                size_t colon_pos = context_str.find(":", type_pos);
                if (colon_pos != std::string::npos) {
                    size_t type_start = context_str.find("\"", colon_pos) + 1;
                    size_t type_end = context_str.find("\"", type_start);
                    if (type_start != std::string::npos && type_end != std::string::npos) {
                        request.context["credential_type"] = context_str.substr(type_start, type_end - type_start);
                    }
                }
            }
        }
    }
    
    // 注意：属性应该从任务签名验证后获取，而不是硬编码
    // 如果请求中包含mission_signature，属性将在认证阶段从签名中提取
    // 环境属性（如天气、电池等）应该从属性提供者获取，而不是硬编码
    
    request.request_time = std::chrono::system_clock::now();
    
    return request;
}

} // namespace api
} // namespace drone_control


