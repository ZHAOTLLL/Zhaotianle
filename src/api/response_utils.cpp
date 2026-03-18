/**
 * HTTP 响应构造工具实现
 * 组装 JSON 成功/错误响应及时间戳字符串。
 */
#include "response_utils.hpp"
#include <sstream>
#include <iomanip>

namespace drone_control {
namespace api {

std::string ResponseUtils::createJsonResponse(const std::map<std::string, std::string>& fields) {
    std::ostringstream response;
    response << "{";
    bool first = true;
    for (const auto& [key, value] : fields) {
        if (!first) response << ",";
        response << "\"" << key << "\":";
        if (value.front() == '{' || value.front() == '[' || 
            value == "true" || value == "false" || 
            (value.front() >= '0' && value.front() <= '9')) {
            response << value;
        } else {
            response << "\"" << value << "\"";
        }
        first = false;
    }
    response << "}";
    return response.str();
}

std::string ResponseUtils::createSuccessResponse(int drone_id, const std::string& command, bool success) {
    return createJsonResponse({
        {"drone_id", std::to_string(drone_id)},
        {"command", command},
        {"success", success ? "true" : "false"},
        {"timestamp", getCurrentTimestamp()}
    });
}

std::string ResponseUtils::createErrorResponse(const std::string& error) {
    return createJsonResponse({{"error", error}});
}

std::string ResponseUtils::getCurrentTimestamp() {
    return std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string ResponseUtils::getCurrentTimestampMs() {
    return std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

} // namespace api
} // namespace drone_control


