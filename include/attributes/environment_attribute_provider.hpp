/**
 * @file environment_attribute_provider.hpp
 * @brief 定义环境属性提供者及其依赖的时间/天气服务，实现访问控制对实时环境上下文的读取与模拟。
 */
#pragma once

#include "base_attribute_provider.hpp"
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace drone_control {

/**
 * 时间服务接口
 */
class TimeService {
public:
    virtual ~TimeService() = default;
    virtual std::map<std::string, std::string> getCurrentTimeAttributes() = 0;
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
};

/**
 * 天气服务接口
 */
class WeatherService {
public:
    virtual ~WeatherService() = default;
    virtual std::map<std::string, std::string> getCurrentWeatherAttributes() = 0;
    virtual void updateWeatherData() = 0;
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
};

/**
 * 系统时间服务实现
 */
class SystemTimeService : public TimeService {
public:
    std::map<std::string, std::string> getCurrentTimeAttributes() override;
    bool initialize(const std::map<std::string, std::string>& config) override;
};

/**
 * 简单天气服务实现（模拟）
 */
class SimpleWeatherService : public WeatherService {
public:
    SimpleWeatherService();
    
    std::map<std::string, std::string> getCurrentWeatherAttributes() override;
    void updateWeatherData() override;
    bool initialize(const std::map<std::string, std::string>& config) override;

private:
    std::map<std::string, std::string> current_weather_;
    std::chrono::steady_clock::time_point last_update_;
};

/**
 * 环境属性提供者
 * 提供时间、天气等环境信息
 */
class EnvironmentAttributeProvider : public BaseAttributeProvider {
public:
    EnvironmentAttributeProvider();
    ~EnvironmentAttributeProvider();
    
    std::map<std::string, std::string> getAttributes(DroneId drone_id) override;
    bool updateAttribute(DroneId drone_id, const std::string& key, const std::string& value) override;
    bool updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) override;
    
    /**
     * 启动定期更新
     */
    void startPeriodicUpdate();
    
    /**
     * 停止定期更新
     */
    void stopPeriodicUpdate();
    
    /**
     * 设置更新间隔
     * @param interval 更新间隔
     */
    void setUpdateInterval(std::chrono::seconds interval);

protected:
    bool doInitialize(const std::map<std::string, std::string>& config) override;

private:
    std::unique_ptr<WeatherService> weather_service_;
    std::unique_ptr<TimeService> time_service_;
    
    std::chrono::seconds update_interval_;
    std::thread update_thread_;
    std::mutex update_mutex_;
    std::condition_variable update_cv_;
    std::mutex attributes_mutex_;
    bool running_;
    
    std::string weather_override_;  // 手动天气覆盖
    
    /**
     * 更新环境数据
     */
    void updateEnvironmentData();
    
    /**
     * 获取当前时间戳
     * @return ISO 8601格式的时间戳
     */
    std::string getCurrentTimestamp();
};

} // namespace drone_control