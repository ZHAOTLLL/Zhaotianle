#pragma once

#include "common/types.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <functional>
#include <queue>
#include <set>
#include <unordered_map>
#include <random>
#include <chrono>

namespace drone_control {

/**
 * 路径规划算法类型
 */
enum class PathPlanningAlgorithm {
    ASTAR,          // A*算法
    DIJKSTRA,       // Dijkstra算法
    RRT,            // 快速随机树算法
    RRT_STAR,       // RRT*算法
    BREADTH_FIRST,  // 广度优先搜索
    DEPTH_FIRST     // 深度优先搜索
};

/**
 * 障碍物类型
 */
enum class ObstacleType {
    STATIC,         // 静态障碍物
    DYNAMIC,        // 动态障碍物
    PRIVACY_ZONE,   // 隐私区域
    NO_FLY_ZONE     // 禁飞区域
};

/**
 * 障碍物结构体
 */
struct Obstacle {
    ObstacleType type;           // 障碍物类型
    Position center;             // 中心点位置
    double radius;               // 半径（米）
    double height;               // 高度（米）
    std::string name;            // 障碍物名称
    
    Obstacle(ObstacleType t, const Position& pos, double r, double h, const std::string& n = "")
        : type(t), center(pos), radius(r), height(h), name(n) {}
};

/**
 * 飞行轨迹结构体
 */
struct FlightTrajectory {
    std::string trajectory_id;           // 轨迹ID
    Position start_position;             // 起始位置
    Position end_position;               // 结束位置
    std::vector<Waypoint> waypoints;     // 航点列表
    double total_distance;               // 总距离（米）
    double estimated_time;               // 预计飞行时间（秒）
    std::map<std::string, std::string> metadata; // 元数据（天气、无人机型号等）
    Timestamp created_time;              // 创建时间
    
    FlightTrajectory() 
        : total_distance(0.0), estimated_time(0.0), created_time(std::chrono::system_clock::now()) {}
};

/**
 * R*树节点结构体
 */
struct RTreeNode {
    std::vector<Position> bounds;        // 节点边界
    std::vector<RTreeNode*> children;    // 子节点
    std::vector<FlightTrajectory*> trajectories; // 轨迹数据
    bool is_leaf;                        // 是否为叶节点
    
    RTreeNode() : is_leaf(false) {}
    ~RTreeNode() {
        for (auto child : children) {
            delete child;
        }
    }
};

/**
 * 飞行计划结构体
 * 定义无人机的飞行路径和控制方式
 */
struct FlightPlan {
    ControlMode mode;                               // 控制模式
    std::vector<Waypoint> waypoints;               // 航点列表
    std::string plan_file_content;                 // .plan文件内容
    std::map<std::string, std::string> parameters; // 飞行参数
    
    // 计划信息
    std::string plan_id;                           // 计划ID
    std::string plan_name;                         // 计划名称
    Timestamp created_time;                        // 创建时间
    
    // 安全约束
    double max_altitude;                           // 最大高度
    double max_speed;                             // 最大速度
    std::vector<std::string> restricted_zones;     // 禁飞区域
    
    // 路径规划相关
    PathPlanningAlgorithm algorithm;               // 使用的路径规划算法
    std::vector<Obstacle> obstacles;               // 路径上的障碍物
    
    FlightPlan() 
        : mode(ControlMode::GUIDED_WAYPOINTS)
        , created_time(std::chrono::system_clock::now())
        , max_altitude(120.0)  // 默认120米
        , max_speed(15.0)      // 默认15m/s
        , algorithm(PathPlanningAlgorithm::ASTAR) {}   // 默认使用A*算法
};

/**
 * 飞行任务状态信息
 */
struct FlightMissionStatus {
    DroneId drone_id;
    std::string status;                            // pending/active/completed/aborted
    std::string current_waypoint;                  // 当前航点
    double progress_percentage;                    // 进度百分比
    
    Timestamp started_at;
    Timestamp estimated_completion;
    std::optional<Timestamp> completed_at;
    
    // 实时信息
    Position current_position;
    double current_speed;
    std::string last_command;
    
    FlightMissionStatus() 
        : drone_id(0)
        , status("pending")
        , progress_percentage(0.0)
        , current_speed(0.0) {}
};

/**
 * 路径规划器
 * 根据认证结果和目标位置从预设路径库中查询和选择路径
 * 对应草稿中"根据其当前位置和目标别名，在数据库中查询与目的地关联的候选路径集合"
 */
class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner();
    
    /**
     * 加载路径数据库（从flight_paths.yaml）
     * @param config_path 配置文件路径
     * @return 是否加载成功
     */
    bool loadPathDatabase(const std::string& config_path);
    
    /**
     * 根据目标位置查询候选路径
     * 对应草稿：根据目标别名查询与目的地关联的候选路径集合
     * @param target_location 目标位置（建筑物代码或别名）
     * @param operation_type 操作类型（delivery, inspection, monitoring等）
     * @return 候选路径ID列表
     */
    std::vector<std::string> queryPathsByDestination(
        const std::string& target_location,
        const std::string& operation_type = "");
    
    /**
     * 选择最匹配的路径
     * 对应草稿：采用基于空间最近邻的查询算子选取一条与当前状态最匹配的路径
     * @param candidate_paths 候选路径ID列表
     * @param current_position 当前位置
     * @param constraints 约束条件（max_altitude, max_speed等）
     * @return 选中的路径ID，如果未找到则返回空
     */
    std::optional<std::string> selectBestPath(
        const std::vector<std::string>& candidate_paths,
        const Position& current_position,
        const std::map<std::string, std::string>& constraints = {});
    
    /**
     * 根据路径ID生成飞行计划
     * 对应草稿：计算出无人机当前位置到该预设路径上最近点的导引段
     * @param path_id 路径ID
     * @param current_position 当前位置
     * @param constraints 约束条件
     * @return 飞行计划，如果路径不存在则返回空
     */
    std::optional<FlightPlan> generatePlanFromPath(
        const std::string& path_id,
        const Position& current_position,
        const std::map<std::string, std::string>& constraints = {});
    
    /**
     * 综合查询：根据目标位置和当前位置直接生成飞行计划
     * 这是最常用的接口，整合了查询、选择和生成三个步骤
     * @param target_location 目标位置
     * @param current_position 当前位置
     * @param operation_type 操作类型
     * @param constraints 约束条件
     * @return 飞行计划
     */
    std::optional<FlightPlan> planPath(
        const std::string& target_location,
        const Position& current_position,
        const std::string& operation_type = "",
        const std::map<std::string, std::string>& constraints = {});
    
    /**
     * 使用指定算法进行路径规划
     * @param start_position 起始位置
     * @param goal_position 目标位置
     * @param obstacles 障碍物列表
     * @param algorithm 路径规划算法
     * @param constraints 约束条件
     * @return 飞行计划
     */
    std::optional<FlightPlan> planPathWithAlgorithm(
        const Position& start_position,
        const Position& goal_position,
        const std::vector<Obstacle>& obstacles,
        PathPlanningAlgorithm algorithm,
        const std::map<std::string, std::string>& constraints = {});
    
    /**
     * 存储飞行轨迹
     * @param trajectory 飞行轨迹
     * @return 是否存储成功
     */
    bool storeFlightTrajectory(const FlightTrajectory& trajectory);
    
    /**
     * 使用R*树检索可用飞行轨迹
     * @param start_position 起始位置
     * @param goal_position 目标位置
     * @param max_distance 最大距离（米）
     * @return 匹配的轨迹列表
     */
    std::vector<FlightTrajectory> retrieveTrajectories(
        const Position& start_position,
        const Position& goal_position,
        double max_distance = 1000.0);
    
    /**
     * 更新R*树索引
     */
    void updateRTreeIndex();

private:
    // 路径数据结构
    struct PathData {
        std::string path_id;
        std::string name;
        std::string type;
        std::string origin;
        std::string destination;
        std::vector<Waypoint> waypoints;
        std::map<std::string, std::string> attributes;
    };
    
    // A*算法节点
    struct AStarNode {
        Position position;
        double g_cost; // 从起点到当前节点的代价
        double h_cost; // 从当前节点到目标的启发式代价
        double f_cost; // 总代价
        AStarNode* parent;
        
        AStarNode(const Position& pos, double g, double h, AStarNode* p = nullptr)
            : position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
        
        bool operator>(const AStarNode& other) const {
            return f_cost > other.f_cost;
        }
    };
    
    // RRT节点
    struct RRTNode {
        Position position;
        RRTNode* parent;
        
        RRTNode(const Position& pos, RRTNode* p = nullptr)
            : position(pos), parent(p) {}
    };
    
    std::map<std::string, PathData> paths_;  // 路径ID -> 路径数据
    std::map<std::string, std::vector<std::string>> destination_index_;  // 目的地 -> 路径ID列表
    std::vector<FlightTrajectory> trajectories_;  // 存储的飞行轨迹
    RTreeNode* rtree_root_;  // R*树根节点
    bool database_loaded_;
    std::mt19937 rng_;  // 随机数生成器
    
    /**
     * 计算两点之间的距离（米）
     */
    double calculateDistance(const Position& p1, const Position& p2) const;
    
    /**
     * 计算点到路径的最近距离
     */
    double calculateDistanceToPath(const Position& point, const std::vector<Waypoint>& path) const;
    
    /**
     * 生成从当前位置到路径的导引段
     */
    std::vector<Waypoint> generateGuidanceSegment(
        const Position& current_pos,
        const std::vector<Waypoint>& target_path) const;
    
    /**
     * 检查位置是否在障碍物内
     */
    bool isInObstacle(const Position& position, const std::vector<Obstacle>& obstacles) const;
    
    /**
     * A*算法实现
     */
    std::vector<Waypoint> astarAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * Dijkstra算法实现
     */
    std::vector<Waypoint> dijkstraAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * RRT算法实现
     */
    std::vector<Waypoint> rrtAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * RRT*算法实现
     */
    std::vector<Waypoint> rrtStarAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * 广度优先搜索算法实现
     */
    std::vector<Waypoint> breadthFirstAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * 深度优先搜索算法实现
     */
    std::vector<Waypoint> depthFirstAlgorithm(
        const Position& start, const Position& goal,
        const std::vector<Obstacle>& obstacles, double max_altitude);
    
    /**
     * 构建R*树
     */
    void buildRTree();
    
    /**
     * 插入轨迹到R*树
     */
    void insertTrajectoryToRTree(RTreeNode* node, FlightTrajectory* trajectory, int depth);
    
    /**
     * 从R*树检索轨迹
     */
    void retrieveFromRTree(RTreeNode* node, const Position& start, const Position& goal, 
                          double max_distance, std::vector<FlightTrajectory>& result);
};

} // namespace drone_control