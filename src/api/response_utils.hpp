#pragma once

/**
 * HTTP 响应构造工具
 * 统一生成 JSON 成功/错误响应及时间戳，供各 API 模块使用。
 */
#include <string>
#include <map>
#include <chrono>

namespace drone_control {
namespace api {

class ResponseUtils {
public:
    static std::string createJsonResponse(const std::map<std::string, std::string>& fields);
    static std::string createSuccessResponse(int drone_id, const std::string& command, bool success = true);
    static std::string createErrorResponse(const std::string& error);
    static std::string getCurrentTimestamp();
    static std::string getCurrentTimestampMs();
};

} // namespace api
} // namespace drone_control


