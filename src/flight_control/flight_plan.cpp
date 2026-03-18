/**
 * 飞行计划与路径规划
 * 飞行计划结构、航点与 .plan 文件内容的生成与解析。
 */
#include "flight_control/flight_plan.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace drone_control {

PathPlanner::PathPlanner() : database_loaded_(false) {
}

PathPlanner::~PathPlanner() {
}

bool PathPlanner::loadPathDatabase(const std::string& config_path) {
    try {
        // 先检查文件是否存在，避免输出不必要的错误信息
        std::ifstream test_file(config_path);
        if (!test_file.good()) {
            // 文件不存在，静默返回false（不输出错误信息）
            return false;
        }
        test_file.close();
        
        YAML::Node config = YAML::LoadFile(config_path);
        
        if (!config["flight_paths"]) {
            // 文件存在但格式不正确，输出错误信息
            std::cerr << "Error: 'flight_paths' not found in config file: " << config_path << std::endl;
            return false;
        }
        
        paths_.clear();
        destination_index_.clear();
        
        for (const auto& path_node : config["flight_paths"]) {
            PathData path;
            path.path_id = path_node["path_id"].as<std::string>("");
            path.name = path_node["name"].as<std::string>("");
            path.type = path_node["type"].as<std::string>("");
            path.origin = path_node["origin"].as<std::string>("");
            path.destination = path_node["destination"].as<std::string>("");
            
            // 解析航点
            if (path_node["waypoints"]) {
                for (const auto& wp_node : path_node["waypoints"]) {
                    Waypoint wp;
                    wp.position.latitude = wp_node["lat"].as<double>(0.0);
                    wp.position.longitude = wp_node["lon"].as<double>(0.0);
                    wp.position.altitude = wp_node["alt"].as<double>(0.0);
                    wp.speed_mps = wp_node["speed"].as<double>(5.0);
                    path.waypoints.push_back(wp);
                }
            }
            
            // 解析属性
            if (path_node["attributes"]) {
                const auto& attrs_node = path_node["attributes"];
                if (attrs_node.IsMap()) {
                    for (const auto& it : attrs_node) {
                        std::string key = it.first.as<std::string>();
                        if (it.second.IsScalar()) {
                            path.attributes[key] = it.second.as<std::string>();
                        } else if (it.second.IsSequence()) {
                            // 对于数组类型，转换为逗号分隔的字符串
                            std::string value;
                            for (size_t i = 0; i < it.second.size(); ++i) {
                                if (i > 0) value += ",";
                                value += it.second[i].as<std::string>();
                            }
                            path.attributes[key] = value;
                        } else {
                            path.attributes[key] = "true";  // 布尔值或其他类型
                        }
                    }
                }
            }
            
            paths_[path.path_id] = path;
            
            // 建立目的地索引
            destination_index_[path.destination].push_back(path.path_id);
        }
        
        database_loaded_ = true;
        std::cout << "Path database loaded: " << paths_.size() << " paths" << std::endl;
        return true;
        
    } catch (const YAML::BadFile& e) {
        // 文件不存在，静默返回false（不输出错误信息）
        // 这是正常的，因为我们尝试多个路径
        return false;
    } catch (const std::exception& e) {
        // 其他错误（如文件格式错误），输出错误信息
        std::cerr << "Error loading path database: " << e.what() << " (file: " << config_path << ")" << std::endl;
        return false;
    }
}

std::vector<std::string> PathPlanner::queryPathsByDestination(
    const std::string& target_location,
    const std::string& operation_type) {
    
    std::vector<std::string> candidates;
    
    // 根据目的地查询
    auto it = destination_index_.find(target_location);
    if (it != destination_index_.end()) {
        candidates = it->second;
    }
    
    // 如果指定了操作类型，进行过滤
    if (!operation_type.empty()) {
        std::vector<std::string> filtered;
        for (const auto& path_id : candidates) {
            auto path_it = paths_.find(path_id);
            if (path_it != paths_.end() && path_it->second.type == operation_type) {
                filtered.push_back(path_id);
            }
        }
        candidates = filtered;
    }
    
    return candidates;
}

std::optional<std::string> PathPlanner::selectBestPath(
    const std::vector<std::string>& candidate_paths,
    const Position& current_position,
    const std::map<std::string, std::string>& constraints) {
    
    if (candidate_paths.empty()) {
        return std::nullopt;
    }
    
    // 简单的最近邻选择：选择起点距离当前位置最近的路径
    double min_distance = std::numeric_limits<double>::max();
    std::string best_path_id;
    
    for (const auto& path_id : candidate_paths) {
        auto it = paths_.find(path_id);
        if (it == paths_.end()) {
            continue;
        }
        
        const auto& path = it->second;
        if (path.waypoints.empty()) {
            continue;
        }
        
        // 计算到路径起点的距离
        double distance = calculateDistance(current_position, path.waypoints[0].position);
        
        // 检查约束条件
        bool meets_constraints = true;
        if (!constraints.empty()) {
            // 检查最大高度约束
            auto max_alt_it = constraints.find("max_altitude");
            if (max_alt_it != constraints.end()) {
                try {
                    double max_alt = std::stod(max_alt_it->second);
                    for (const auto& wp : path.waypoints) {
                        if (wp.position.altitude > max_alt) {
                            meets_constraints = false;
                            break;
                        }
                    }
                } catch (...) {
                    // 忽略解析错误
                }
            }
        }
        
        if (meets_constraints && distance < min_distance) {
            min_distance = distance;
            best_path_id = path_id;
        }
    }
    
    if (best_path_id.empty()) {
        return std::nullopt;
    }
    
    return best_path_id;
}

std::optional<FlightPlan> PathPlanner::generatePlanFromPath(
    const std::string& path_id,
    const Position& current_position,
    const std::map<std::string, std::string>& constraints) {
    
    auto it = paths_.find(path_id);
    if (it == paths_.end()) {
        return std::nullopt;
    }
    
    const auto& path_data = it->second;
    if (path_data.waypoints.empty()) {
        return std::nullopt;
    }
    
    FlightPlan plan;
    plan.plan_id = path_id;
    plan.plan_name = path_data.name;
    plan.mode = ControlMode::GUIDED_WAYPOINTS;
    
    // 应用约束条件
    if (!constraints.empty()) {
        auto max_alt_it = constraints.find("max_altitude");
        if (max_alt_it != constraints.end()) {
            try {
                plan.max_altitude = std::stod(max_alt_it->second);
            } catch (...) {
                plan.max_altitude = 120.0;
            }
        }
        
        auto max_speed_it = constraints.find("max_speed");
        if (max_speed_it != constraints.end()) {
            try {
                plan.max_speed = std::stod(max_speed_it->second);
            } catch (...) {
                plan.max_speed = 15.0;
            }
        }
    }
    
    // 生成导引段：从当前位置到路径最近点
    auto guidance_segment = generateGuidanceSegment(current_position, path_data.waypoints);
    plan.waypoints.insert(plan.waypoints.end(), 
                         guidance_segment.begin(), 
                         guidance_segment.end());
    
    // 添加路径航点
    for (const auto& wp : path_data.waypoints) {
        Waypoint plan_wp = wp;
        // 应用速度约束
        if (plan.max_speed > 0 && plan_wp.speed_mps > plan.max_speed) {
            plan_wp.speed_mps = plan.max_speed;
        }
        plan.waypoints.push_back(plan_wp);
    }
    
    return plan;
}

std::optional<FlightPlan> PathPlanner::planPath(
    const std::string& target_location,
    const Position& current_position,
    const std::string& operation_type,
    const std::map<std::string, std::string>& constraints) {
    
    if (!database_loaded_) {
        std::cerr << "Path database not loaded" << std::endl;
        return std::nullopt;
    }
    
    // 1. 查询候选路径
    auto candidates = queryPathsByDestination(target_location, operation_type);
    if (candidates.empty()) {
        std::cerr << "No paths found for destination: " << target_location << std::endl;
        return std::nullopt;
    }
    
    // 2. 选择最佳路径
    auto best_path_id = selectBestPath(candidates, current_position, constraints);
    if (!best_path_id.has_value()) {
        std::cerr << "No suitable path found" << std::endl;
        return std::nullopt;
    }
    
    // 3. 生成飞行计划
    return generatePlanFromPath(best_path_id.value(), current_position, constraints);
}

double PathPlanner::calculateDistance(const Position& p1, const Position& p2) const {
    // 使用Haversine公式计算两点间距离（米）
    const double R = 6371000.0; // 地球半径（米）
    
    double lat1_rad = p1.latitude * M_PI / 180.0;
    double lat2_rad = p2.latitude * M_PI / 180.0;
    double delta_lat = (p2.latitude - p1.latitude) * M_PI / 180.0;
    double delta_lon = (p2.longitude - p1.longitude) * M_PI / 180.0;
    
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return R * c;
}

double PathPlanner::calculateDistanceToPath(
    const Position& point,
    const std::vector<Waypoint>& path) const {
    
    if (path.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& wp : path) {
        double dist = calculateDistance(point, wp.position);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    
    return min_dist;
}

std::vector<Waypoint> PathPlanner::generateGuidanceSegment(
    const Position& current_pos,
    const std::vector<Waypoint>& target_path) const {
    
    std::vector<Waypoint> guidance;
    
    if (target_path.empty()) {
        return guidance;
    }
    
    // 找到路径上距离当前位置最近的航点
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    
    for (size_t i = 0; i < target_path.size(); ++i) {
        double dist = calculateDistance(current_pos, target_path[i].position);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    // 如果距离很近（小于50米），直接使用路径起点
    if (min_dist < 50.0) {
        return guidance;  // 不需要导引段
    }
    
    // 生成从当前位置到最近航点的导引段
    Waypoint guidance_wp;
    guidance_wp.position = target_path[nearest_idx].position;
    guidance_wp.speed_mps = 5.0;  // 导引段使用中等速度
    guidance_wp.action = "guidance";
    guidance.push_back(guidance_wp);
    
    return guidance;
}

} // namespace drone_control

