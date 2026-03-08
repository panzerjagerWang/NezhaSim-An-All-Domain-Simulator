#ifndef NEZHA_WHEEL_EFFICIENCY_PLUGIN_H
#define NEZHA_WHEEL_EFFICIENCY_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <memory>

namespace gazebo {

/// \brief 详细的地形状态
enum class DetailedTerrainState {
    LAND_GROUNDED,      // 在陆地上且接地
    LAND_AIRBORNE,      // 在陆地上方但悬空
    SEA_SURFACE,        // 在海面上
    UNDERWATER,         // 在水下
    SEA_ABOVE,          // 在海面上方
    UNKNOWN             // 未知状态
};

/// \brief 轮子效率模式
enum class WheelEfficiencyMode {
    NORMAL,             // 正常效率（陆地接地）
    WATER,              // 水中低效率
    AIRBORNE,           // 悬空（无效率）
    DISABLED            // 禁用
};

/// \brief 轮子效率控制插件
/// 订阅地形检测插件的话题，根据详细的地形状态调整轮子效率
class WheelEfficiencyPlugin : public ModelPlugin {
public:
    WheelEfficiencyPlugin();
    virtual ~WheelEfficiencyPlugin();
    
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
    /// \brief 世界更新回调
    void OnUpdate(const common::UpdateInfo& _info);
    
    /// \brief 地形状态回调
    void TerrainCallback(const std_msgs::String::ConstPtr& msg);
    
    /// \brief 解析地形状态
    DetailedTerrainState ParseTerrainState(const std::string& terrain_name) const;
    
    /// \brief 根据地形状态决定效率模式
    WheelEfficiencyMode DetermineEfficiencyMode(DetailedTerrainState state) const;
    
    /// \brief 更新轮子效率
    void UpdateWheelEfficiency();
    
    /// \brief 检测是否接地
    bool CheckGroundContact();
    
    /// \brief 设置轮子摩擦力
    bool SetWheelFriction(physics::LinkPtr wheel_link, 
                          double mu1, double mu2, 
                          double slip1 = 0.0, double slip2 = 0.0);
    
    /// \brief 施加阻力
    void ApplyResistanceForces();
    
    /// \brief 初始化轮子链接
    void InitializeWheelLinks();
    
    /// \brief 打印状态信息
    void PrintStatusInfo();

private:
    // Gazebo 相关
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr base_link_;
    event::ConnectionPtr update_connection_;
    
    // 轮子链接
    std::vector<physics::LinkPtr> wheel_links_;
    std::vector<std::string> wheel_link_names_;
    
    // ROS 相关
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Subscriber terrain_sub_;
    ros::Publisher status_pub_;  // 发布当前效率状态
    std::string namespace_;
    std::string terrain_topic_;
    std::string status_topic_;
    
    // 地形状态
    std::string current_terrain_name_;
    DetailedTerrainState current_terrain_state_;
    DetailedTerrainState last_terrain_state_;
    
    // 效率模式
    WheelEfficiencyMode current_mode_;
    WheelEfficiencyMode last_mode_;
    
    // 接地检测
    bool use_ground_contact_check_;
    double ground_contact_threshold_;  // 接地检测阈值（距离）
    bool is_grounded_;
    
    // 摩擦力参数
    struct FrictionParams {
        double mu1;
        double mu2;
        double slip1;
        double slip2;
    };
    FrictionParams normal_friction_;    // 正常效率（陆地接地）
    FrictionParams water_friction_;     // 水中效率
    FrictionParams airborne_friction_;  // 悬空效率
    
    // 阻尼和阻力参数
    struct DampingParams {
        double linear_damping;
        double angular_damping;
        double resistance_force;
    };
    DampingParams normal_damping_;
    DampingParams water_damping_;
    DampingParams airborne_damping_;
    
    // 更新控制
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;
    
    // 调试
    bool debug_mode_;
    int debug_counter_;
    double status_print_interval_;
    common::Time last_status_print_time_;
    
    // 控制模式
    enum ControlMode {
        FRICTION_ONLY,
        DAMPING_ONLY,
        COMBINED
    };
    ControlMode control_mode_;
    
    // 渐变过渡
    bool enable_smooth_transition_;
    double transition_duration_;
    common::Time transition_start_time_;
    FrictionParams transition_start_friction_;
    FrictionParams transition_target_friction_;
};

} // namespace gazebo

#endif // NEZHA_WHEEL_EFFICIENCY_PLUGIN_H


