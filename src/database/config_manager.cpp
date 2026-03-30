/**
 * 配置文件管理器
 */
#include "database/config_manager.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace drone_control {

ConfigManager::ConfigManager() {
}

ConfigManager::~ConfigManager() {
}

bool ConfigManager::loadConfigFile(const std::string& file_path) {
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        config_files_[file_path] = config;
        std::cout << "[配置管理] 加载配置文件成功: " << file_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[配置管理] 加载配置文件失败: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::saveConfigToDatabase(DatabaseManager& db_manager, const std::string& file_path) {
    auto it = config_files_.find(file_path);
    if (it == config_files_.end()) {
        std::cerr << "[配置管理] 配置文件未加载: " << file_path << std::endl;
        return false;
    }

    // 读取配置文件内容
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "[配置管理] 无法打开配置文件: " << file_path << std::endl;
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // 生成缓存键
    std::string key = "config:" + file_path;
    // 保存到缓存（Redis）
    if (!db_manager.setCache(key, content, 3600)) {
        std::cerr << "[配置管理] 保存配置到缓存失败" << std::endl;
        return false;
    }

    std::cout << "[配置管理] 保存配置到数据库成功: " << file_path << std::endl;
    return true;
}

std::optional<YAML::Node> ConfigManager::getConfig(const std::string& file_path) {
    auto it = config_files_.find(file_path);
    if (it != config_files_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool ConfigManager::loadAllConfigFiles() {
    // 尝试不同的路径
    std::vector<std::string> base_paths = {".", "..", "../build"};
    std::vector<std::string> config_files = {
        "config/main_config.yaml",
        "config/access_control_policies.yaml",
        "config/airspace_config.yaml",
        "config/authentication_config.yaml",
        "config/flight_paths.yaml"
    };

    for (const auto& config_file : config_files) {
        bool loaded = false;
        for (const auto& base_path : base_paths) {
            std::string full_path = base_path + "/" + config_file;
            if (loadConfigFile(full_path)) {
                loaded = true;
                break;
            }
        }
        if (!loaded) {
            std::cerr << "[配置管理] 无法加载配置文件: " << config_file << std::endl;
            return false;
        }
    }

    return true;
}

bool ConfigManager::saveAllConfigsToDatabase(DatabaseManager& db_manager) {
    bool success = true;
    for (const auto& [file_path, _] : config_files_) {
        if (!saveConfigToDatabase(db_manager, file_path)) {
            success = false;
        }
    }
    return success;
}

} // namespace drone_control
