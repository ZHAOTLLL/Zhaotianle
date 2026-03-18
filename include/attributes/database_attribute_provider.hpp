/**
 * @file database_attribute_provider.hpp
 * @brief 定义基于数据库/内存存储的属性提供者接口与默认实现，用于持久化无人机静态属性及其历史。
 */
#pragma once

#include "base_attribute_provider.hpp"
#include <map>
#include <vector>
#include <mutex>
#include <chrono>
#include <memory>

namespace drone_control {

/**
 * 属性历史记录项
 */
struct AttributeHistoryEntry {
    std::string value;
    std::chrono::system_clock::time_point timestamp;
    std::string updated_by;  // 更新来源
};

/**
 * 数据库接口
 */
class DatabaseInterface {
public:
    virtual ~DatabaseInterface() = default;
    
    /**
     * 初始化数据库连接
     * @param config 配置参数
     * @return 是否初始化成功
     */
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
    
    /**
     * 获取无人机属性
     * @param drone_id 无人机ID
     * @return 属性映射
     */
    virtual std::map<std::string, std::string> getDroneAttributes(DroneId drone_id) = 0;
    
    /**
     * 设置无人机属性
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param value 属性值
     * @param updated_by 更新来源
     * @return 是否设置成功
     */
    virtual bool setDroneAttribute(DroneId drone_id, const std::string& key, 
                                  const std::string& value, const std::string& updated_by = "system") = 0;
    
    /**
     * 批量设置无人机属性
     * @param drone_id 无人机ID
     * @param attributes 属性映射
     * @param updated_by 更新来源
     * @return 是否设置成功
     */
    virtual bool setDroneAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes,
                                   const std::string& updated_by = "system") = 0;
    
    /**
     * 删除无人机属性
     * @param drone_id 无人机ID
     * @param key 属性键
     * @return 是否删除成功
     */
    virtual bool deleteDroneAttribute(DroneId drone_id, const std::string& key) = 0;
    
    /**
     * 获取属性历史记录
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param limit 记录数量限制
     * @return 历史记录列表
     */
    virtual std::vector<AttributeHistoryEntry> getAttributeHistory(DroneId drone_id, 
                                                                  const std::string& key, 
                                                                  int limit = 10) = 0;
    
    /**
     * 清理过期历史记录
     * @param older_than 清理早于此时间的记录
     * @return 清理的记录数量
     */
    virtual int cleanupHistory(std::chrono::system_clock::time_point older_than) = 0;
    
    /**
     * 获取所有已知的无人机ID
     * @return 无人机ID列表
     */
    virtual std::vector<DroneId> getAllDroneIds() = 0;
};

/**
 * 内存数据库实现
 * 用于测试和简单部署场景
 */
class InMemoryDatabase : public DatabaseInterface {
public:
    InMemoryDatabase();
    ~InMemoryDatabase() = default;
    
    bool initialize(const std::map<std::string, std::string>& config) override;
    std::map<std::string, std::string> getDroneAttributes(DroneId drone_id) override;
    bool setDroneAttribute(DroneId drone_id, const std::string& key, 
                          const std::string& value, const std::string& updated_by = "system") override;
    bool setDroneAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes,
                           const std::string& updated_by = "system") override;
    bool deleteDroneAttribute(DroneId drone_id, const std::string& key) override;
    std::vector<AttributeHistoryEntry> getAttributeHistory(DroneId drone_id, 
                                                          const std::string& key, 
                                                          int limit = 10) override;
    int cleanupHistory(std::chrono::system_clock::time_point older_than) override;
    std::vector<DroneId> getAllDroneIds() override;
    
    /**
     * 获取统计信息
     */
    struct Statistics {
        size_t total_drones;
        size_t total_attributes;
        size_t total_history_entries;
    };
    Statistics getStatistics() const;

private:
    // 当前属性存储：drone_id -> {key -> value}
    std::map<DroneId, std::map<std::string, std::string>> current_attributes_;
    
    // 历史记录存储：drone_id -> {key -> [history_entries]}
    std::map<DroneId, std::map<std::string, std::vector<AttributeHistoryEntry>>> attribute_history_;
    
    mutable std::mutex data_mutex_;
    
    int max_history_per_attribute_;  // 每个属性保留的最大历史记录数
    
    /**
     * 添加历史记录
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param value 属性值
     * @param updated_by 更新来源
     */
    void addHistoryEntry(DroneId drone_id, const std::string& key, 
                        const std::string& value, const std::string& updated_by);
    
    /**
     * 限制历史记录数量
     * @param drone_id 无人机ID
     * @param key 属性键
     */
    void limitHistorySize(DroneId drone_id, const std::string& key);
};

/**
 * 数据库属性提供者
 * 负责管理无人机的静态属性存储和检索
 */
class DatabaseAttributeProvider : public BaseAttributeProvider {
public:
    DatabaseAttributeProvider();
    explicit DatabaseAttributeProvider(std::unique_ptr<DatabaseInterface> database);
    ~DatabaseAttributeProvider() = default;
    
    std::map<std::string, std::string> getAttributes(DroneId drone_id) override;
    bool updateAttribute(DroneId drone_id, const std::string& key, const std::string& value) override;
    bool updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) override;
    
    /**
     * 删除属性
     * @param drone_id 无人机ID
     * @param key 属性键
     * @return 是否删除成功
     */
    bool deleteAttribute(DroneId drone_id, const std::string& key);
    
    /**
     * 获取属性历史记录
     * @param drone_id 无人机ID
     * @param key 属性键
     * @param limit 记录数量限制
     * @return 历史记录列表
     */
    std::vector<AttributeHistoryEntry> getAttributeHistory(DroneId drone_id, 
                                                          const std::string& key, 
                                                          int limit = 10);
    
    /**
     * 获取所有已知的无人机ID
     * @return 无人机ID列表
     */
    std::vector<DroneId> getAllDroneIds();
    
    /**
     * 清理过期历史记录
     * @param days_to_keep 保留天数
     * @return 清理的记录数量
     */
    int cleanupOldHistory(int days_to_keep = 30);

protected:
    bool doInitialize(const std::map<std::string, std::string>& config) override;

private:
    std::unique_ptr<DatabaseInterface> database_;
    std::string update_source_;  // 更新来源标识
};

} // namespace drone_control