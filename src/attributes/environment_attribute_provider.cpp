/**
 * 环境属性提供者
 * 提供时间、天气等环境属性，支持周期性更新，供策略引擎使用。
 */
#include "attributes/environment_attribute_provider.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <thread>

namespace drone_control {

EnvironmentAttributeProvider::EnvironmentAttributeProvider() 
    : BaseAttributeProvider("environment"), 
      update_interval_(std::chrono::seconds(300)),  // 默认5分钟更新
      weather_service_(std::make_unique<SimpleWeatherService>()),
      time_service_(std::make_unique<SystemTimeService>()),
      running_(false) {
}

EnvironmentAttributeProvider::~EnvironmentAttributeProvider() {
    stopPeriodicUpdate();
}

std::map<std::string, std::string> EnvironmentAttributeProvider::getAttributes(DroneId drone_id) {
    if (!isInitialized()) {
        return {};
    }
    
    std::lock_guard<std::mutex> lock(attributes_mutex_);
    
    // 获取当前环境属性
    std::map<std::string, std::string> attributes;
    
    // 时间属性
    auto time_attrs = time_service_->getCurrentTimeAttributes();
    attributes.insert(time_attrs.begin(), time_attrs.end());
    
    // 天气属性（若存在手动覆盖则优先使用）
    auto weather_attrs = weather_service_->getCurrentWeatherAttributes();
    if (!weather_override_.empty()) {
        weather_attrs["weather_condition"] = weather_override_;
        weather_attrs["weather_condition_source"] = "manual_override";
    }
    attributes.insert(weather_attrs.begin(), weather_attrs.end());
    
    // 系统状态属性
    attributes["environment_update_time"] = getCurrentTimestamp();
    attributes["environment_provider_status"] = "active";
    
    return attributes;
}

bool EnvironmentAttributeProvider::updateAttribute(DroneId drone_id, const std::string& key, const std::string& value) {
    // 环境属性通常是只读的，但可以支持一些配置更新
    if (key == "weather_condition_override") {
        std::lock_guard<std::mutex> lock(attributes_mutex_);
        weather_override_ = value;
        return true;
    }
    
    return false;  // 大部分环境属性不支持手动更新
}

bool EnvironmentAttributeProvider::updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) {
    bool any_updated = false;
    
    for (const auto& [key, value] : attributes) {
        if (updateAttribute(drone_id, key, value)) {
            any_updated = true;
        }
    }
    
    return any_updated;
}

void EnvironmentAttributeProvider::startPeriodicUpdate() {
    if (running_) {
        return;
    }
    
    running_ = true;
    update_thread_ = std::thread([this]() {
        while (running_) {
            try {
                updateEnvironmentData();
            } catch (const std::exception& e) {
                std::cerr << "Error updating environment data: " << e.what() << std::endl;
            }
            
            // 等待更新间隔
            std::unique_lock<std::mutex> lock(update_mutex_);
            update_cv_.wait_for(lock, update_interval_, [this]() { return !running_; });
        }
    });
}

void EnvironmentAttributeProvider::stopPeriodicUpdate() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    update_cv_.notify_all();
    
    if (update_thread_.joinable()) {
        update_thread_.join();
    }
}

void EnvironmentAttributeProvider::setUpdateInterval(std::chrono::seconds interval) {
    update_interval_ = interval;
}

bool EnvironmentAttributeProvider::doInitialize(const std::map<std::string, std::string>& config) {
    // 验证必需配置
    std::vector<std::string> required_keys = {};  // 环境提供者没有必需配置
    if (!validateRequiredConfig(required_keys)) {
        return false;
    }
    
    // 配置更新间隔
    int interval_seconds = getConfigValueInt("update_interval_seconds", 300);
    setUpdateInterval(std::chrono::seconds(interval_seconds));
    
    // 配置天气服务
    std::string weather_api_key = getConfigValue("weather_api_key");
    if (!weather_api_key.empty()) {
        // 如果有API密钥，可以创建真实的天气服务实现
        // 这里暂时使用简单的模拟服务
        weather_service_ = std::make_unique<SimpleWeatherService>();
    }
    
    // 初始化服务
    if (!weather_service_->initialize(config)) {
        std::cerr << "Failed to initialize weather service" << std::endl;
        return false;
    }
    
    if (!time_service_->initialize(config)) {
        std::cerr << "Failed to initialize time service" << std::endl;
        return false;
    }
    
    // 启动定期更新
    startPeriodicUpdate();
    
    return true;
}

void EnvironmentAttributeProvider::updateEnvironmentData() {
    std::lock_guard<std::mutex> lock(attributes_mutex_);
    
    // 更新天气数据
    weather_service_->updateWeatherData();
    
    // 时间服务通常不需要更新，因为它总是返回当前时间
}

std::string EnvironmentAttributeProvider::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

// SystemTimeService 实现
std::map<std::string, std::string> SystemTimeService::getCurrentTimeAttributes() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::gmtime(&time_t);
    
    std::map<std::string, std::string> attributes;
    
    // 基本时间信息
    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    attributes["current_time"] = timestamp_ss.str();
    
    // 时间组件
    attributes["current_year"] = std::to_string(tm.tm_year + 1900);
    attributes["current_month"] = std::to_string(tm.tm_mon + 1);
    attributes["current_day"] = std::to_string(tm.tm_mday);
    attributes["current_hour"] = std::to_string(tm.tm_hour);
    attributes["current_minute"] = std::to_string(tm.tm_min);
    attributes["current_weekday"] = std::to_string(tm.tm_wday);  // 0=Sunday
    
    // 时间段分类
    if (tm.tm_hour >= 6 && tm.tm_hour < 12) {
        attributes["time_period"] = "morning";
    } else if (tm.tm_hour >= 12 && tm.tm_hour < 18) {
        attributes["time_period"] = "afternoon";
    } else if (tm.tm_hour >= 18 && tm.tm_hour < 22) {
        attributes["time_period"] = "evening";
    } else {
        attributes["time_period"] = "night";
    }
    
    // 工作时间判断
    bool is_weekday = (tm.tm_wday >= 1 && tm.tm_wday <= 5);
    bool is_business_hours = (tm.tm_hour >= 9 && tm.tm_hour < 17);
    attributes["is_business_hours"] = (is_weekday && is_business_hours) ? "true" : "false";
    attributes["is_weekday"] = is_weekday ? "true" : "false";
    
    return attributes;
}

bool SystemTimeService::initialize(const std::map<std::string, std::string>& config) {
    // 时间服务不需要特殊初始化
    return true;
}

// SimpleWeatherService 实现
SimpleWeatherService::SimpleWeatherService() : last_update_(std::chrono::steady_clock::time_point::min()) {
    // 初始化模拟天气数据
    current_weather_["weather_condition"] = "clear";
    current_weather_["temperature"] = "20";
    current_weather_["humidity"] = "60";
    current_weather_["wind_speed"] = "5";
    current_weather_["visibility"] = "good";
    current_weather_["precipitation"] = "none";
}

std::map<std::string, std::string> SimpleWeatherService::getCurrentWeatherAttributes() {
    return current_weather_;
}

void SimpleWeatherService::updateWeatherData() {
    auto now = std::chrono::steady_clock::now();
    
    // 模拟天气变化（每次更新时随机改变一些值）
    if (now - last_update_ > std::chrono::minutes(5)) {
        // 简单的天气模拟逻辑
        static int weather_cycle = 0;
        weather_cycle = (weather_cycle + 1) % 4;
        
        switch (weather_cycle) {
            case 0:
                current_weather_["weather_condition"] = "clear";
                current_weather_["visibility"] = "excellent";
                break;
            case 1:
                current_weather_["weather_condition"] = "partly_cloudy";
                current_weather_["visibility"] = "good";
                break;
            case 2:
                current_weather_["weather_condition"] = "cloudy";
                current_weather_["visibility"] = "moderate";
                break;
            case 3:
                current_weather_["weather_condition"] = "overcast";
                current_weather_["visibility"] = "poor";
                break;
        }
        
        last_update_ = now;
    }
}

bool SimpleWeatherService::initialize(const std::map<std::string, std::string>& config) {
    // 简单天气服务不需要特殊初始化
    return true;
}

} // namespace drone_control