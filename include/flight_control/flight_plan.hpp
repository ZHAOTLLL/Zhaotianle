#pragma once

#include "common/types.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <optional>

namespace drone_control {

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
    
    FlightPlan() 
        : mode(ControlMode::GUIDED_WAYPOINTS)
        , created_time(std::chrono::system_clock::now())
        , max_altitude(120.0)  // 默认120米
        , max_speed(15.0) {}   // 默认15m/s
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
    
    std::map<std::string, PathData> paths_;  // 路径ID -> 路径数据
    std::map<std::string, std::vector<std::string>> destination_index_;  // 目的地 -> 路径ID列表
    bool database_loaded_;
    
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
};

} // namespace drone_control