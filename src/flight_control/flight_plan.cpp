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
#include <stack>
#include <unordered_set>

namespace drone_control {

PathPlanner::PathPlanner() : database_loaded_(false), rtree_root_(nullptr), rng_(std::random_device{}()) {
}

PathPlanner::~PathPlanner() {
    if (rtree_root_) {
        delete rtree_root_;
    }
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

bool PathPlanner::isInObstacle(const Position& position, const std::vector<Obstacle>& obstacles) const {
    for (const auto& obstacle : obstacles) {
        double distance = calculateDistance(position, obstacle.center);
        if (distance <= obstacle.radius && position.altitude <= obstacle.height) {
            return true;
        }
    }
    return false;
}

std::vector<Waypoint> PathPlanner::astarAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    // 定义方向：前后左右上下
    const double step_size = 10.0; // 步长10米
    const std::vector<Position> directions = {
        {0, step_size, 0}, {0, -step_size, 0}, // 前后
        {step_size, 0, 0}, {-step_size, 0, 0}, // 左右
        {0, 0, step_size}, {0, 0, -step_size}  // 上下
    };
    
    // 优先队列，按照f_cost排序
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, std::greater<AStarNode*>> open_set;
    
    // 已访问节点的映射，存储最佳路径
    std::unordered_map<std::string, AStarNode*> closed_set;
    
    // 创建起点节点
    AStarNode* start_node = new AStarNode(start, 0, calculateDistance(start, goal));
    open_set.push(start_node);
    
    // 主循环
    while (!open_set.empty()) {
        AStarNode* current = open_set.top();
        open_set.pop();
        
        // 检查是否到达目标
        if (calculateDistance(current->position, goal) < step_size) {
            // 回溯路径
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 清理内存
            for (auto& pair : closed_set) {
                delete pair.second;
            }
            while (!open_set.empty()) {
                delete open_set.top();
                open_set.pop();
            }
            
            return waypoints;
        }
        
        // 标记为已访问
        std::string key = std::to_string(current->position.latitude) + "," + 
                         std::to_string(current->position.longitude) + "," + 
                         std::to_string(current->position.altitude);
        closed_set[key] = current;
        
        // 探索相邻节点
        for (const auto& dir : directions) {
            Position new_pos = {
                current->position.latitude + dir.latitude,
                current->position.longitude + dir.longitude,
                current->position.altitude + dir.altitude
            };
            
            // 检查是否在障碍物内或超出高度限制
            if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
                continue;
            }
            
            // 检查是否已访问
            std::string new_key = std::to_string(new_pos.latitude) + "," + 
                                 std::to_string(new_pos.longitude) + "," + 
                                 std::to_string(new_pos.altitude);
            if (closed_set.find(new_key) != closed_set.end()) {
                continue;
            }
            
            // 计算代价
            double g_cost = current->g_cost + step_size;
            double h_cost = calculateDistance(new_pos, goal);
            
            // 创建新节点
            AStarNode* new_node = new AStarNode(new_pos, g_cost, h_cost, current);
            open_set.push(new_node);
        }
    }
    
    // 清理内存
    for (auto& pair : closed_set) {
        delete pair.second;
    }
    while (!open_set.empty()) {
        delete open_set.top();
        open_set.pop();
    }
    
    return waypoints; // 未找到路径
}

std::vector<Waypoint> PathPlanner::dijkstraAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    // 定义方向：前后左右上下
    const double step_size = 10.0; // 步长10米
    const std::vector<Position> directions = {
        {0, step_size, 0}, {0, -step_size, 0}, // 前后
        {step_size, 0, 0}, {-step_size, 0, 0}, // 左右
        {0, 0, step_size}, {0, 0, -step_size}  // 上下
    };
    
    // 优先队列，按照g_cost排序
    std::priority_queue<std::pair<double, AStarNode*>, 
                       std::vector<std::pair<double, AStarNode*>>, 
                       std::greater<std::pair<double, AStarNode*>>> open_set;
    
    // 已访问节点的映射，存储最佳路径
    std::unordered_map<std::string, AStarNode*> closed_set;
    
    // 创建起点节点
    AStarNode* start_node = new AStarNode(start, 0, 0);
    open_set.push({0, start_node});
    
    // 主循环
    while (!open_set.empty()) {
        auto current_pair = open_set.top();
        open_set.pop();
        AStarNode* current = current_pair.second;
        
        // 检查是否到达目标
        if (calculateDistance(current->position, goal) < step_size) {
            // 回溯路径
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 清理内存
            for (auto& pair : closed_set) {
                delete pair.second;
            }
            while (!open_set.empty()) {
                delete open_set.top().second;
                open_set.pop();
            }
            
            return waypoints;
        }
        
        // 标记为已访问
        std::string key = std::to_string(current->position.latitude) + "," + 
                         std::to_string(current->position.longitude) + "," + 
                         std::to_string(current->position.altitude);
        closed_set[key] = current;
        
        // 探索相邻节点
        for (const auto& dir : directions) {
            Position new_pos = {
                current->position.latitude + dir.latitude,
                current->position.longitude + dir.longitude,
                current->position.altitude + dir.altitude
            };
            
            // 检查是否在障碍物内或超出高度限制
            if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
                continue;
            }
            
            // 检查是否已访问
            std::string new_key = std::to_string(new_pos.latitude) + "," + 
                                 std::to_string(new_pos.longitude) + "," + 
                                 std::to_string(new_pos.altitude);
            if (closed_set.find(new_key) != closed_set.end()) {
                continue;
            }
            
            // 计算代价
            double g_cost = current->g_cost + step_size;
            
            // 创建新节点
            AStarNode* new_node = new AStarNode(new_pos, g_cost, 0, current);
            open_set.push({g_cost, new_node});
        }
    }
    
    // 清理内存
    for (auto& pair : closed_set) {
        delete pair.second;
    }
    while (!open_set.empty()) {
        delete open_set.top().second;
        open_set.pop();
    }
    
    return waypoints; // 未找到路径
}

std::vector<Waypoint> PathPlanner::rrtAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    const int max_iterations = 1000;
    const double step_size = 10.0;
    const double goal_threshold = 20.0;
    
    std::vector<RRTNode*> nodes;
    RRTNode* start_node = new RRTNode(start);
    nodes.push_back(start_node);
    
    for (int i = 0; i < max_iterations; ++i) {
        // 随机采样一个点
        Position random_pos;
        if (i % 10 == 0) { // 10%的概率直接采样目标点
            random_pos = goal;
        } else {
            // 在起点和目标点周围随机采样
            double lat_range = std::max(std::abs(goal.latitude - start.latitude) * 2, 0.001);
            double lon_range = std::max(std::abs(goal.longitude - start.longitude) * 2, 0.001);
            double alt_range = max_altitude;
            
            std::uniform_real_distribution<double> lat_dist(start.latitude - lat_range, start.latitude + lat_range);
            std::uniform_real_distribution<double> lon_dist(start.longitude - lon_range, start.longitude + lon_range);
            std::uniform_real_distribution<double> alt_dist(0, alt_range);
            
            random_pos.latitude = lat_dist(rng_);
            random_pos.longitude = lon_dist(rng_);
            random_pos.altitude = alt_dist(rng_);
        }
        
        // 找到最近的节点
        RRTNode* nearest_node = nullptr;
        double min_distance = std::numeric_limits<double>::max();
        
        for (RRTNode* node : nodes) {
            double distance = calculateDistance(node->position, random_pos);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_node = node;
            }
        }
        
        if (!nearest_node) continue;
        
        // 向随机点方向移动step_size距离
        double distance = calculateDistance(nearest_node->position, random_pos);
        double ratio = step_size / distance;
        
        Position new_pos;
        new_pos.latitude = nearest_node->position.latitude + (random_pos.latitude - nearest_node->position.latitude) * ratio;
        new_pos.longitude = nearest_node->position.longitude + (random_pos.longitude - nearest_node->position.longitude) * ratio;
        new_pos.altitude = nearest_node->position.altitude + (random_pos.altitude - nearest_node->position.altitude) * ratio;
        
        // 检查新位置是否在障碍物内或超出高度限制
        if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
            continue;
        }
        
        // 检查路径是否与障碍物碰撞
        bool collision = false;
        for (const auto& obstacle : obstacles) {
            // 简化碰撞检测：检查新位置是否在障碍物内
            if (isInObstacle(new_pos, obstacles)) {
                collision = true;
                break;
            }
        }
        
        if (collision) continue;
        
        // 创建新节点
        RRTNode* new_node = new RRTNode(new_pos, nearest_node);
        nodes.push_back(new_node);
        
        // 检查是否到达目标
        if (calculateDistance(new_pos, goal) < goal_threshold) {
            // 回溯路径
            RRTNode* current = new_node;
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 添加目标点
            Waypoint goal_wp;
            goal_wp.position = goal;
            goal_wp.speed_mps = 5.0;
            waypoints.push_back(goal_wp);
            
            // 清理内存
            for (RRTNode* node : nodes) {
                delete node;
            }
            
            return waypoints;
        }
    }
    
    // 清理内存
    for (RRTNode* node : nodes) {
        delete node;
    }
    
    return waypoints; // 未找到路径
}

std::vector<Waypoint> PathPlanner::rrtStarAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    const int max_iterations = 1000;
    const double step_size = 10.0;
    const double goal_threshold = 20.0;
    const double neighbor_radius = 30.0;
    
    std::vector<RRTNode*> nodes;
    RRTNode* start_node = new RRTNode(start);
    nodes.push_back(start_node);
    
    for (int i = 0; i < max_iterations; ++i) {
        // 随机采样一个点
        Position random_pos;
        if (i % 10 == 0) { // 10%的概率直接采样目标点
            random_pos = goal;
        } else {
            // 在起点和目标点周围随机采样
            double lat_range = std::max(std::abs(goal.latitude - start.latitude) * 2, 0.001);
            double lon_range = std::max(std::abs(goal.longitude - start.longitude) * 2, 0.001);
            double alt_range = max_altitude;
            
            std::uniform_real_distribution<double> lat_dist(start.latitude - lat_range, start.latitude + lat_range);
            std::uniform_real_distribution<double> lon_dist(start.longitude - lon_range, start.longitude + lon_range);
            std::uniform_real_distribution<double> alt_dist(0, alt_range);
            
            random_pos.latitude = lat_dist(rng_);
            random_pos.longitude = lon_dist(rng_);
            random_pos.altitude = alt_dist(rng_);
        }
        
        // 找到最近的节点
        RRTNode* nearest_node = nullptr;
        double min_distance = std::numeric_limits<double>::max();
        
        for (RRTNode* node : nodes) {
            double distance = calculateDistance(node->position, random_pos);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_node = node;
            }
        }
        
        if (!nearest_node) continue;
        
        // 向随机点方向移动step_size距离
        double distance = calculateDistance(nearest_node->position, random_pos);
        double ratio = step_size / distance;
        
        Position new_pos;
        new_pos.latitude = nearest_node->position.latitude + (random_pos.latitude - nearest_node->position.latitude) * ratio;
        new_pos.longitude = nearest_node->position.longitude + (random_pos.longitude - nearest_node->position.longitude) * ratio;
        new_pos.altitude = nearest_node->position.altitude + (random_pos.altitude - nearest_node->position.altitude) * ratio;
        
        // 检查新位置是否在障碍物内或超出高度限制
        if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
            continue;
        }
        
        // 找到附近的节点
        std::vector<RRTNode*> neighbors;
        for (RRTNode* node : nodes) {
            if (calculateDistance(node->position, new_pos) < neighbor_radius) {
                neighbors.push_back(node);
            }
        }
        
        // 选择最优父节点
        RRTNode* best_parent = nearest_node;
        double best_cost = calculateDistance(start, nearest_node->position) + step_size;
        
        for (RRTNode* neighbor : neighbors) {
            double cost = calculateDistance(start, neighbor->position) + calculateDistance(neighbor->position, new_pos);
            if (cost < best_cost) {
                // 检查路径是否与障碍物碰撞
                bool collision = false;
                if (isInObstacle(new_pos, obstacles)) {
                    collision = true;
                }
                if (!collision) {
                    best_parent = neighbor;
                    best_cost = cost;
                }
            }
        }
        
        // 创建新节点
        RRTNode* new_node = new RRTNode(new_pos, best_parent);
        nodes.push_back(new_node);
        
        // 重新连接附近的节点
        for (RRTNode* neighbor : neighbors) {
            double cost = calculateDistance(start, new_node->position) + calculateDistance(new_node->position, neighbor->position);
            double current_cost = calculateDistance(start, neighbor->position);
            
            if (cost < current_cost) {
                // 检查路径是否与障碍物碰撞
                bool collision = false;
                if (isInObstacle(neighbor->position, obstacles)) {
                    collision = true;
                }
                if (!collision) {
                    neighbor->parent = new_node;
                }
            }
        }
        
        // 检查是否到达目标
        if (calculateDistance(new_pos, goal) < goal_threshold) {
            // 回溯路径
            RRTNode* current = new_node;
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 添加目标点
            Waypoint goal_wp;
            goal_wp.position = goal;
            goal_wp.speed_mps = 5.0;
            waypoints.push_back(goal_wp);
            
            // 清理内存
            for (RRTNode* node : nodes) {
                delete node;
            }
            
            return waypoints;
        }
    }
    
    // 清理内存
    for (RRTNode* node : nodes) {
        delete node;
    }
    
    return waypoints; // 未找到路径
}

std::vector<Waypoint> PathPlanner::breadthFirstAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    // 定义方向：前后左右上下
    const double step_size = 10.0; // 步长10米
    const std::vector<Position> directions = {
        {0, step_size, 0}, {0, -step_size, 0}, // 前后
        {step_size, 0, 0}, {-step_size, 0, 0}, // 左右
        {0, 0, step_size}, {0, 0, -step_size}  // 上下
    };
    
    // 队列用于广度优先搜索
    std::queue<AStarNode*> queue;
    
    // 已访问节点的映射，存储父节点
    std::unordered_map<std::string, AStarNode*> visited;
    
    // 创建起点节点
    AStarNode* start_node = new AStarNode(start, 0, 0);
    queue.push(start_node);
    visited[std::to_string(start.latitude) + "," + std::to_string(start.longitude) + "," + std::to_string(start.altitude)] = start_node;
    
    // 主循环
    while (!queue.empty()) {
        AStarNode* current = queue.front();
        queue.pop();
        
        // 检查是否到达目标
        if (calculateDistance(current->position, goal) < step_size) {
            // 回溯路径
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 清理内存
            for (auto& pair : visited) {
                delete pair.second;
            }
            
            return waypoints;
        }
        
        // 探索相邻节点
        for (const auto& dir : directions) {
            Position new_pos = {
                current->position.latitude + dir.latitude,
                current->position.longitude + dir.longitude,
                current->position.altitude + dir.altitude
            };
            
            // 检查是否在障碍物内或超出高度限制
            if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
                continue;
            }
            
            // 检查是否已访问
            std::string key = std::to_string(new_pos.latitude) + "," + 
                             std::to_string(new_pos.longitude) + "," + 
                             std::to_string(new_pos.altitude);
            if (visited.find(key) != visited.end()) {
                continue;
            }
            
            // 创建新节点
            AStarNode* new_node = new AStarNode(new_pos, 0, 0, current);
            queue.push(new_node);
            visited[key] = new_node;
        }
    }
    
    // 清理内存
    for (auto& pair : visited) {
        delete pair.second;
    }
    
    return waypoints; // 未找到路径
}

std::vector<Waypoint> PathPlanner::depthFirstAlgorithm(
    const Position& start, const Position& goal,
    const std::vector<Obstacle>& obstacles, double max_altitude) {
    std::vector<Waypoint> waypoints;
    
    // 定义方向：前后左右上下
    const double step_size = 10.0; // 步长10米
    const std::vector<Position> directions = {
        {0, step_size, 0}, {0, -step_size, 0}, // 前后
        {step_size, 0, 0}, {-step_size, 0, 0}, // 左右
        {0, 0, step_size}, {0, 0, -step_size}  // 上下
    };
    
    // 栈用于深度优先搜索
    std::stack<AStarNode*> dfs_stack;
    
    // 已访问节点的集合
    std::unordered_set<std::string> visited;
    
    // 创建起点节点
    AStarNode* start_node = new AStarNode(start, 0, 0);
    dfs_stack.push(start_node);
    visited.insert(std::to_string(start.latitude) + "," + std::to_string(start.longitude) + "," + std::to_string(start.altitude));
    
    // 主循环
    while (!dfs_stack.empty()) {
        AStarNode* current = dfs_stack.top();
        dfs_stack.pop();
        
        // 检查是否到达目标
        if (calculateDistance(current->position, goal) < step_size) {
            // 回溯路径
            while (current) {
                Waypoint wp;
                wp.position = current->position;
                wp.speed_mps = 5.0;
                waypoints.push_back(wp);
                current = current->parent;
            }
            std::reverse(waypoints.begin(), waypoints.end());
            
            // 清理内存
            // 注意：这里简化处理，实际应该清理所有创建的节点
            return waypoints;
        }
        
        // 探索相邻节点（逆序压入栈，保证顺序）
        for (auto it = directions.rbegin(); it != directions.rend(); ++it) {
            const auto& dir = *it;
            Position new_pos = {
                current->position.latitude + dir.latitude,
                current->position.longitude + dir.longitude,
                current->position.altitude + dir.altitude
            };
            
            // 检查是否在障碍物内或超出高度限制
            if (isInObstacle(new_pos, obstacles) || new_pos.altitude > max_altitude || new_pos.altitude < 0) {
                continue;
            }
            
            // 检查是否已访问
            std::string key = std::to_string(new_pos.latitude) + "," + 
                             std::to_string(new_pos.longitude) + "," + 
                             std::to_string(new_pos.altitude);
            if (visited.find(key) != visited.end()) {
                continue;
            }
            
            // 创建新节点
            AStarNode* new_node = new AStarNode(new_pos, 0, 0, current);
            dfs_stack.push(new_node);
            visited.insert(key);
        }
    }
    
    return waypoints; // 未找到路径
}

std::optional<FlightPlan> PathPlanner::planPathWithAlgorithm(
    const Position& start_position,
    const Position& goal_position,
    const std::vector<Obstacle>& obstacles,
    PathPlanningAlgorithm algorithm,
    const std::map<std::string, std::string>& constraints) {
    FlightPlan plan;
    plan.algorithm = algorithm;
    plan.obstacles = obstacles;
    
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
    
    // 根据算法类型执行路径规划
    std::vector<Waypoint> waypoints;
    switch (algorithm) {
        case PathPlanningAlgorithm::ASTAR:
            waypoints = astarAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        case PathPlanningAlgorithm::DIJKSTRA:
            waypoints = dijkstraAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        case PathPlanningAlgorithm::RRT:
            waypoints = rrtAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        case PathPlanningAlgorithm::RRT_STAR:
            waypoints = rrtStarAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        case PathPlanningAlgorithm::BREADTH_FIRST:
            waypoints = breadthFirstAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        case PathPlanningAlgorithm::DEPTH_FIRST:
            waypoints = depthFirstAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
        default:
            waypoints = astarAlgorithm(start_position, goal_position, obstacles, plan.max_altitude);
            break;
    }
    
    if (waypoints.empty()) {
        return std::nullopt;
    }
    
    plan.waypoints = waypoints;
    
    // 计算总距离和预计飞行时间
    double total_distance = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        total_distance += calculateDistance(waypoints[i-1].position, waypoints[i].position);
    }
    
    double estimated_time = total_distance / plan.max_speed;
    plan.parameters["total_distance"] = std::to_string(total_distance);
    plan.parameters["estimated_time"] = std::to_string(estimated_time);
    
    return plan;
}

bool PathPlanner::storeFlightTrajectory(const FlightTrajectory& trajectory) {
    trajectories_.push_back(trajectory);
    updateRTreeIndex();
    return true;
}

std::vector<FlightTrajectory> PathPlanner::retrieveTrajectories(
    const Position& start_position,
    const Position& goal_position,
    double max_distance) {
    std::vector<FlightTrajectory> result;
    if (rtree_root_) {
        retrieveFromRTree(rtree_root_, start_position, goal_position, max_distance, result);
    }
    
    // 对结果进行排序，优先选择距离最近的轨迹
    std::sort(result.begin(), result.end(), [&](const FlightTrajectory& a, const FlightTrajectory& b) {
        double dist_a = calculateDistance(a.start_position, start_position) + 
                       calculateDistance(a.end_position, goal_position);
        double dist_b = calculateDistance(b.start_position, start_position) + 
                       calculateDistance(b.end_position, goal_position);
        return dist_a < dist_b;
    });
    
    return result;
}

void PathPlanner::updateRTreeIndex() {
    if (rtree_root_) {
        delete rtree_root_;
    }
    buildRTree();
}

void PathPlanner::buildRTree() {
    rtree_root_ = new RTreeNode();
    rtree_root_->is_leaf = true;
    
    for (size_t i = 0; i < trajectories_.size(); ++i) {
        insertTrajectoryToRTree(rtree_root_, &trajectories_[i], 0);
    }
}

void PathPlanner::insertTrajectoryToRTree(RTreeNode* node, FlightTrajectory* trajectory, int depth) {
    if (node->is_leaf) {
        node->trajectories.push_back(trajectory);
        
        // 如果节点已满，需要分裂
        if (node->trajectories.size() > 4) { // 简单的分裂阈值
            // 简化处理，实际R*树有更复杂的分裂策略
            RTreeNode* new_node = new RTreeNode();
            new_node->is_leaf = true;
            
            // 将一半轨迹移到新节点
            size_t mid = node->trajectories.size() / 2;
            for (size_t i = mid; i < node->trajectories.size(); ++i) {
                new_node->trajectories.push_back(node->trajectories[i]);
            }
            node->trajectories.resize(mid);
            
            // 如果是根节点，创建新的根节点
            if (node == rtree_root_) {
                RTreeNode* new_root = new RTreeNode();
                new_root->is_leaf = false;
                new_root->children.push_back(node);
                new_root->children.push_back(new_node);
                rtree_root_ = new_root;
            } else {
                // 实际应该通知父节点进行分裂，这里简化处理
            }
        }
    } else {
        // 选择最合适的子节点
        RTreeNode* best_child = nullptr;
        double min_enlargement = std::numeric_limits<double>::max();
        
        for (auto child : node->children) {
            // 简化处理，实际R*树有更复杂的选择策略
            double enlargement = 0; // 计算边界扩大的代价
            if (enlargement < min_enlargement) {
                min_enlargement = enlargement;
                best_child = child;
            }
        }
        
        if (best_child) {
            insertTrajectoryToRTree(best_child, trajectory, depth + 1);
        }
    }
}

void PathPlanner::retrieveFromRTree(RTreeNode* node, const Position& start, const Position& goal, 
                                  double max_distance, std::vector<FlightTrajectory>& result) {
    if (node->is_leaf) {
        for (auto trajectory : node->trajectories) {
            double start_dist = calculateDistance(trajectory->start_position, start);
            double end_dist = calculateDistance(trajectory->end_position, goal);
            if (start_dist + end_dist < max_distance) {
                result.push_back(*trajectory);
            }
        }
    } else {
        for (auto child : node->children) {
            // 简化处理，实际R*树需要检查边界是否与查询区域相交

            retrieveFromRTree(child, start, goal, max_distance, result);
        }
    }
}

} // namespace drone_control

