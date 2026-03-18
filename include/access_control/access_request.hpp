/**
 * 访问请求
 * 一次访问控制评估的输入：无人机 ID、目标位置、操作类型、属性及上下文等。
 */
#pragma once

#include "common/types.hpp"
#include "common/drone_attributes.hpp"
#include <string>
#include <map>

namespace drone_control {

/** 访问请求：携带身份、目标、操作与上下文，供策略引擎评估 */
struct AccessRequest {
    DroneId drone_id;                           // 无人机ID
    std::string target_location;                // 目标位置
    std::string operation_type;                 // 操作类型
    DroneAttributes attributes;                 // 无人机属性
    std::map<std::string, std::string> context; // 上下文信息
    
    // 请求时间戳
    Timestamp request_time;
    
    // 请求来源
    std::string request_source;
    
    AccessRequest() 
        : drone_id(0)
        , request_time(std::chrono::system_clock::now())
        , request_source("unknown") {}
        
    AccessRequest(DroneId id, const std::string& location, const std::string& operation)
        : drone_id(id)
        , target_location(location)
        , operation_type(operation)
        , request_time(std::chrono::system_clock::now())
        , request_source("mavlink") {}
};

} // namespace drone_control