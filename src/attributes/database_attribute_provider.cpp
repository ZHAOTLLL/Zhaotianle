/**
 * 数据库属性提供者与内存数据库
 * 管理无人机静态属性与历史记录，支持可插拔后端，默认使用内存存储。
 */
#include "attributes/database_attribute_provider.hpp"
#include <iostream>
#include <algorithm>
#include <memory>

namespace drone_control {

// InMemoryDatabase 实现
InMemoryDatabase::InMemoryDatabase() : max_history_per_attribute_(100) {
}

bool InMemoryDatabase::initialize(const std::map<std::string, std::string>& config)
{
    // 配置最大历史记录数
    auto it = config.find("max_history_per_attribute");
    if (it != config.end()) {
        try {
            max_history_per_attribute_ = std::stoi(it->second);
        } catch (const std::exception&) {
            max_history_per_attribute_ = 100;  // 默认值
        }
    }
    
    return true;
}

std::map<std::string, std::string> InMemoryDatabase::getDroneAttributes(DroneId drone_id) // 获取无人机属性
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = current_attributes_.find(drone_id);
    if (it != current_attributes_.end()) {
        return it->second;
    }
    
    return {};  // 返回空映射如果无人机不存在
}

bool InMemoryDatabase::setDroneAttribute(DroneId drone_id, const std::string& key, 
                                        const std::string& value, const std::string& updated_by) // 设置无人机属性
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新当前属性
    current_attributes_[drone_id][key] = value;
    
    // 添加历史记录
    addHistoryEntry(drone_id, key, value, updated_by);
    
    return true;
}

bool InMemoryDatabase::setDroneAttributes(DroneId drone_id, 
                                         const std::map<std::string, std::string>& attributes,
                                         const std::string& updated_by) // 设置无人机属性
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (const auto& [key, value] : attributes) {
        // 更新当前属性
        current_attributes_[drone_id][key] = value;
        
        // 添加历史记录
        addHistoryEntry(drone_id, key, value, updated_by);
    }
    
    return true;
}

bool InMemoryDatabase::deleteDroneAttribute(DroneId drone_id, const std::string& key) // 删除无人机属性
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto drone_it = current_attributes_.find(drone_id);
    if (drone_it != current_attributes_.end()) {
        auto attr_it = drone_it->second.find(key);
        if (attr_it != drone_it->second.end()) {
            drone_it->second.erase(attr_it);
            
            // 添加删除记录到历史
            addHistoryEntry(drone_id, key, "", "system_delete");
            
            return true;
        }
    }
    
    return false;
}

std::vector<AttributeHistoryEntry> InMemoryDatabase::getAttributeHistory(DroneId drone_id, 
                                                                        const std::string& key, 
                                                                        int limit) // 获取属性历史
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto drone_it = attribute_history_.find(drone_id);
    if (drone_it != attribute_history_.end()) {
        auto attr_it = drone_it->second.find(key);
        if (attr_it != drone_it->second.end()) {
            const auto& history = attr_it->second;
            
            // 返回最新的limit条记录
            int start_index = std::max(0, static_cast<int>(history.size()) - limit);
            return std::vector<AttributeHistoryEntry>(history.begin() + start_index, history.end());
        }
    }
    
    return {};
}

int InMemoryDatabase::cleanupHistory(std::chrono::system_clock::time_point older_than) // 清理历史
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    int cleaned_count = 0;
    
    for (auto& [drone_id, drone_history] : attribute_history_) {
        for (auto& [key, history] : drone_history) {
            auto original_size = history.size();
            
            // 移除早于指定时间的记录
            history.erase(
                std::remove_if(history.begin(), history.end(),
                    [older_than](const AttributeHistoryEntry& entry) {
                        return entry.timestamp < older_than;
                    }),
                history.end());
            
            cleaned_count += (original_size - history.size());
        }
    }
    
    return cleaned_count;
}

std::vector<DroneId> InMemoryDatabase::getAllDroneIds() // 获取所有无人机ID
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::vector<DroneId> drone_ids;
    drone_ids.reserve(current_attributes_.size());
    
    for (const auto& [drone_id, attributes] : current_attributes_) {
        drone_ids.push_back(drone_id);
    }
    
    return drone_ids;
}

InMemoryDatabase::Statistics InMemoryDatabase::getStatistics() const // 获取统计信息
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    Statistics stats;
    stats.total_drones = current_attributes_.size();
    stats.total_attributes = 0;
    stats.total_history_entries = 0;
    
    for (const auto& [drone_id, attributes] : current_attributes_) {
        stats.total_attributes += attributes.size();
    }
    
    for (const auto& [drone_id, drone_history] : attribute_history_) {
        for (const auto& [key, history] : drone_history) {
            stats.total_history_entries += history.size();
        }
    }
    
    return stats;
}

void InMemoryDatabase::addHistoryEntry(DroneId drone_id, const std::string& key, 
                                      const std::string& value, const std::string& updated_by) // 添加历史记录
{
    AttributeHistoryEntry entry;
    entry.value = value;
    entry.timestamp = std::chrono::system_clock::now();
    entry.updated_by = updated_by;
    
    attribute_history_[drone_id][key].push_back(entry);
    
    // 限制历史记录大小
    limitHistorySize(drone_id, key);
}

void InMemoryDatabase::limitHistorySize(DroneId drone_id, const std::string& key) // 限制历史记录大小
{
    auto& history = attribute_history_[drone_id][key];
    
    if (static_cast<int>(history.size()) > max_history_per_attribute_) {
        // 保留最新的记录
        int excess = history.size() - max_history_per_attribute_;
        history.erase(history.begin(), history.begin() + excess);
    }
}

// DatabaseAttributeProvider 实现
DatabaseAttributeProvider::DatabaseAttributeProvider() // 数据库属性提供者
    : BaseAttributeProvider("database"), 
      database_(std::make_unique<InMemoryDatabase>()),
      update_source_("database_provider") {
}

DatabaseAttributeProvider::DatabaseAttributeProvider(std::unique_ptr<DatabaseInterface> database)
    : BaseAttributeProvider("database"), 
      database_(std::move(database)),
      update_source_("database_provider") {
}

std::map<std::string, std::string> DatabaseAttributeProvider::getAttributes(DroneId drone_id) {
    if (!isInitialized() || !database_) {
        return {};
    }
    
    return database_->getDroneAttributes(drone_id);
}

bool DatabaseAttributeProvider::updateAttribute(DroneId drone_id, const std::string& key, const std::string& value) {
    if (!isInitialized() || !database_) {
        return false;
    }
    
    return database_->setDroneAttribute(drone_id, key, value, update_source_);
}

bool DatabaseAttributeProvider::updateAttributes(DroneId drone_id, const std::map<std::string, std::string>& attributes) {
    if (!isInitialized() || !database_) {
        return false;
    }
    
    return database_->setDroneAttributes(drone_id, attributes, update_source_);
}

bool DatabaseAttributeProvider::deleteAttribute(DroneId drone_id, const std::string& key) {
    if (!isInitialized() || !database_) {
        return false;
    }
    
    return database_->deleteDroneAttribute(drone_id, key);
}

std::vector<AttributeHistoryEntry> DatabaseAttributeProvider::getAttributeHistory(DroneId drone_id, 
                                                                                 const std::string& key, 
                                                                                 int limit) {
    if (!isInitialized() || !database_) {
        return {};
    }
    
    return database_->getAttributeHistory(drone_id, key, limit);
}

std::vector<DroneId> DatabaseAttributeProvider::getAllDroneIds() {
    if (!isInitialized() || !database_) {
        return {};
    }
    
    return database_->getAllDroneIds();
}

int DatabaseAttributeProvider::cleanupOldHistory(int days_to_keep) {
    if (!isInitialized() || !database_) {
        return 0;
    }
    
    auto cutoff_time = std::chrono::system_clock::now() - std::chrono::hours(24 * days_to_keep);
    return database_->cleanupHistory(cutoff_time);
}

bool DatabaseAttributeProvider::doInitialize(const std::map<std::string, std::string>& config) {
    if (!database_) {
        std::cerr << "Database interface not set" << std::endl;
        return false;
    }
    
    // 设置更新来源
    auto source_it = config.find("update_source");
    if (source_it != config.end()) {
        update_source_ = source_it->second;
    }
    
    // 初始化数据库
    if (!database_->initialize(config)) {
        std::cerr << "Failed to initialize database interface" << std::endl;
        return false;
    }
    
    return true;
}

} // namespace drone_control