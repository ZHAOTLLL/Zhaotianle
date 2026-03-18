#pragma once

/**
 * 访问评估 API
 * 接收 JSON 请求体，构造访问请求并调用访问控制引擎，返回评估结果 JSON。
 */
#include <string>
#include <memory>
#include "response_utils.hpp"
#include "access_control/access_request.hpp"

namespace drone_control {
    class AccessControlEngine;
}

namespace drone_control {
namespace api {

/** 处理 POST /api/v1/access/evaluate，请求体为访问请求 JSON */
class AccessAPI {
public:
    explicit AccessAPI(std::shared_ptr<AccessControlEngine> access_engine);

    std::string handleAccessEvaluateWithBody(const std::string& path, const std::string& method, const std::string& body);

private:
    std::shared_ptr<AccessControlEngine> access_engine_;

    AccessRequest parseAccessRequestFromJson(const std::string& json_body);
};

} // namespace api
} // namespace drone_control
