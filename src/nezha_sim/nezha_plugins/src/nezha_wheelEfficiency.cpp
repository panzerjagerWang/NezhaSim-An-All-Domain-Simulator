#include "nezha_wheelEfficiency.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo {

WheelEfficiencyPlugin::WheelEfficiencyPlugin()
  : ModelPlugin()
  , model_(nullptr)
  , world_(nullptr)
  , base_link_(nullptr)
  , update_connection_(nullptr)
  , ros_node_(nullptr)
  , namespace_("")
  , terrain_topic_("terrain_status")
  , status_topic_("wheel_efficiency_status")
  , current_terrain_name_("unknown")
  , current_terrain_state_(DetailedTerrainState::UNKNOWN)
  , last_terrain_state_(DetailedTerrainState::UNKNOWN)
  , current_mode_(WheelEfficiencyMode::DISABLED)
  , last_mode_(WheelEfficiencyMode::DISABLED)
  , use_ground_contact_check_(true)
  , ground_contact_threshold_(0.1)
  , is_grounded_(false)
  , update_rate_(50.0)
  , update_period_(1.0/50.0)
  , last_update_time_(0)
  , debug_mode_(true)
  , debug_counter_(0)
  , status_print_interval_(2.0)
  , last_status_print_time_(0)
  , control_mode_(ControlMode::COMBINED)
  , enable_smooth_transition_(false)
  , transition_duration_(0.5)
  , transition_start_time_(0)
{
    // 正常效率参数（陆地接地）
    normal_friction_.mu1 = 1.0;
    normal_friction_.mu2 = 1.0;
    normal_friction_.slip1 = 0.0;
    normal_friction_.slip2 = 0.0;
    
    normal_damping_.linear_damping = 0.01;
    normal_damping_.angular_damping = 0.01;
    normal_damping_.resistance_force = 0.0;
    
    // 水中效率参数
    water_friction_.mu1 = 0.02;
    water_friction_.mu2 = 0.02;
    water_friction_.slip1 = 2.0;
    water_friction_.slip2 = 2.0;
    
    water_damping_.linear_damping = 0.9;
    water_damping_.angular_damping = 0.9;
    water_damping_.resistance_force = 80.0;
    
    // 悬空效率参数
    airborne_friction_.mu1 = 0.0;
    airborne_friction_.mu2 = 0.0;
    airborne_friction_.slip1 = 5.0;
    airborne_friction_.slip2 = 5.0;
    
    airborne_damping_.linear_damping = 0.05;
    airborne_damping_.angular_damping = 0.05;
    airborne_damping_.resistance_force = 5.0;
}

WheelEfficiencyPlugin::~WheelEfficiencyPlugin() {
    if (update_connection_) {
        update_connection_.reset();
    }
    if (ros_node_) {
        ros_node_->shutdown();
    }
}

void WheelEfficiencyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    gzmsg << "═══════════════════════════════════════════════════" << std::endl;
    gzmsg << "[wheel_efficiency] Loading plugin..." << std::endl;
    
    model_ = _model;
    world_ = model_->GetWorld();
    
    // 读取配置参数
    if (_sdf->HasElement("robotNamespace")) {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    
    if (_sdf->HasElement("terrainTopic")) {
        terrain_topic_ = _sdf->GetElement("terrainTopic")->Get<std::string>();
    }
    
    if (_sdf->HasElement("statusTopic")) {
        status_topic_ = _sdf->GetElement("statusTopic")->Get<std::string>();
    }
    
    if (_sdf->HasElement("updateRate")) {
        update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
        update_period_ = 1.0 / update_rate_;
    }
    
    if (_sdf->HasElement("debugMode")) {
        debug_mode_ = _sdf->GetElement("debugMode")->Get<bool>();
    }
    
    if (_sdf->HasElement("useGroundContactCheck")) {
        use_ground_contact_check_ = _sdf->GetElement("useGroundContactCheck")->Get<bool>();
    }
    
    if (_sdf->HasElement("groundContactThreshold")) {
        ground_contact_threshold_ = _sdf->GetElement("groundContactThreshold")->Get<double>();
    }
    
    // 读取正常效率参数
    if (_sdf->HasElement("normalFrictionMu1")) {
        normal_friction_.mu1 = _sdf->GetElement("normalFrictionMu1")->Get<double>();
    }
    if (_sdf->HasElement("normalFrictionMu2")) {
        normal_friction_.mu2 = _sdf->GetElement("normalFrictionMu2")->Get<double>();
    }
    
    // 读取水中效率参数
    if (_sdf->HasElement("waterFrictionMu1")) {
        water_friction_.mu1 = _sdf->GetElement("waterFrictionMu1")->Get<double>();
    }
    if (_sdf->HasElement("waterFrictionMu2")) {
        water_friction_.mu2 = _sdf->GetElement("waterFrictionMu2")->Get<double>();
    }
    if (_sdf->HasElement("waterSlip1")) {
        water_friction_.slip1 = _sdf->GetElement("waterSlip1")->Get<double>();
    }
    if (_sdf->HasElement("waterSlip2")) {
        water_friction_.slip2 = _sdf->GetElement("waterSlip2")->Get<double>();
    }
    if (_sdf->HasElement("waterLinearDamping")) {
        water_damping_.linear_damping = _sdf->GetElement("waterLinearDamping")->Get<double>();
    }
    if (_sdf->HasElement("waterAngularDamping")) {
        water_damping_.angular_damping = _sdf->GetElement("waterAngularDamping")->Get<double>();
    }
    if (_sdf->HasElement("waterResistanceForce")) {
        water_damping_.resistance_force = _sdf->GetElement("waterResistanceForce")->Get<double>();
    }
    
    // 读取悬空效率参数
    if (_sdf->HasElement("airborneFrictionMu1")) {
        airborne_friction_.mu1 = _sdf->GetElement("airborneFrictionMu1")->Get<double>();
    }
    if (_sdf->HasElement("airborneFrictionMu2")) {
        airborne_friction_.mu2 = _sdf->GetElement("airborneFrictionMu2")->Get<double>();
    }
    if (_sdf->HasElement("airborneSlip1")) {
        airborne_friction_.slip1 = _sdf->GetElement("airborneSlip1")->Get<double>();
    }
    
    // 读取控制模式
    if (_sdf->HasElement("controlMode")) {
        std::string mode_str = _sdf->GetElement("controlMode")->Get<std::string>();
        if (mode_str == "friction") {
            control_mode_ = ControlMode::FRICTION_ONLY;
        } else if (mode_str == "damping") {
            control_mode_ = ControlMode::DAMPING_ONLY;
        } else {
            control_mode_ = ControlMode::COMBINED;
        }
    }
    
    if (_sdf->HasElement("enableSmoothTransition")) {
        enable_smooth_transition_ = _sdf->GetElement("enableSmoothTransition")->Get<bool>();
    }
    
    if (_sdf->HasElement("transitionDuration")) {
        transition_duration_ = _sdf->GetElement("transitionDuration")->Get<double>();
    }
    
    // 获取 base_link
    base_link_ = model_->GetLink("base_link");
    if (!base_link_) {
        // 尝试获取第一个链接
        auto links = model_->GetLinks();
        if (!links.empty()) {
            base_link_ = links.front();
        }
    }
    
    // 初始化轮子链接
    InitializeWheelLinks();
    
    // 初始化 ROS
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "wheel_efficiency_plugin", ros::init_options::NoSigintHandler);
    }
    
    std::string node_namespace = namespace_;
    if (!node_namespace.empty() && node_namespace[0] != '/') {
        node_namespace = "/" + node_namespace;
    }
    ros_node_.reset(new ros::NodeHandle(node_namespace));
    
    // 订阅地形状态话题
    terrain_sub_ = ros_node_->subscribe(terrain_topic_, 10, 
                                        &WheelEfficiencyPlugin::TerrainCallback, this);
    
    // 发布效率状态话题
    status_pub_ = ros_node_->advertise<std_msgs::String>(status_topic_, 10);
    
    // 连接世界更新事件
    last_update_time_ = world_->SimTime();
    last_status_print_time_ = world_->SimTime();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&WheelEfficiencyPlugin::OnUpdate, this, _1));
    
    gzmsg << "[wheel_efficiency] ✓ Plugin loaded successfully!" << std::endl;
    gzmsg << "[wheel_efficiency] Model: " << model_->GetName() << std::endl;
    gzmsg << "[wheel_efficiency] Namespace: " << node_namespace << std::endl;
    gzmsg << "[wheel_efficiency] Terrain topic: " << terrain_topic_ << std::endl;
    gzmsg << "[wheel_efficiency] Status topic: " << status_topic_ << std::endl;
    gzmsg << "[wheel_efficiency] Wheel count: " << wheel_links_.size() << std::endl;
    gzmsg << "[wheel_efficiency] Ground contact check: " << (use_ground_contact_check_ ? "ENABLED" : "DISABLED") << std::endl;
    gzmsg << "[wheel_efficiency] Control mode: " << control_mode_ << std::endl;
    gzmsg << "═══════════════════════════════════════════════════" << std::endl;
}

void WheelEfficiencyPlugin::InitializeWheelLinks() {
    // 常见的轮子命名模式
    std::vector<std::string> common_patterns = {
        "front_left_wheel", "front_right_wheel", 
        "rear_left_wheel", "rear_right_wheel",
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
        "left_wheel", "right_wheel",
        "wheel_left", "wheel_right"
    };
    
    auto links = model_->GetLinks();
    for (const auto& link : links) {
        std::string link_name = link->GetName();
        std::string link_name_lower = link_name;
        std::transform(link_name_lower.begin(), link_name_lower.end(), 
                      link_name_lower.begin(), ::tolower);
        
        // 检查是否包含 "wheel" 关键词
        if (link_name_lower.find("wheel") != std::string::npos) {
            wheel_links_.push_back(link);
            wheel_link_names_.push_back(link_name);
            gzmsg << "[wheel_efficiency] ✓ Found wheel: " << link_name << std::endl;
        }
    }
    
    if (wheel_links_.empty()) {
        gzerr << "[wheel_efficiency] ✗ No wheel links found!" << std::endl;
    }
}

void WheelEfficiencyPlugin::TerrainCallback(const std_msgs::String::ConstPtr& msg) {
    current_terrain_name_ = msg->data;
    current_terrain_state_ = ParseTerrainState(current_terrain_name_);
    
    if (debug_mode_ && current_terrain_state_ != last_terrain_state_) {
        gzmsg << "[wheel_efficiency] Terrain changed: \"" << current_terrain_name_ 
              << "\" -> State: " << static_cast<int>(current_terrain_state_) << std::endl;
    }
}

DetailedTerrainState WheelEfficiencyPlugin::ParseTerrainState(
    const std::string& terrain_name) const {
    
    if (terrain_name == "land") {
        return DetailedTerrainState::LAND_GROUNDED;
    } 
    else if (terrain_name == "sea_surface") {
        return DetailedTerrainState::SEA_SURFACE;
    } 
    else if (terrain_name == "underwater") {
        return DetailedTerrainState::UNDERWATER;
    } 
    else if (terrain_name == "sea_above") {
        return DetailedTerrainState::SEA_ABOVE;
    } 
    else if (terrain_name == "unknown") {
        return DetailedTerrainState::UNKNOWN;
    }
    else {
        return DetailedTerrainState::UNKNOWN;
    }
}

WheelEfficiencyMode WheelEfficiencyPlugin::DetermineEfficiencyMode(
    DetailedTerrainState state) const {
    
    switch (state) {
        case DetailedTerrainState::LAND_GROUNDED:
            return WheelEfficiencyMode::NORMAL;
            
        case DetailedTerrainState::LAND_AIRBORNE:
            return WheelEfficiencyMode::AIRBORNE;
            
        case DetailedTerrainState::SEA_SURFACE:
        case DetailedTerrainState::UNDERWATER:
        case DetailedTerrainState::SEA_ABOVE:
            return WheelEfficiencyMode::WATER;
            
        case DetailedTerrainState::UNKNOWN:
        default:
            return WheelEfficiencyMode::AIRBORNE;
    }
}


bool WheelEfficiencyPlugin::CheckGroundContact() {
    // 不再需要复杂的接触检测
    // 直接返回 terrain_status 是否为 LAND_GROUNDED
    return (current_terrain_state_ == DetailedTerrainState::LAND_GROUNDED);
}


void WheelEfficiencyPlugin::OnUpdate(const common::UpdateInfo& _info) {
    common::Time current_time = _info.simTime;
    
    // 控制更新频率
    if ((current_time - last_update_time_).Double() < update_period_) {
        return;
    }
    last_update_time_ = current_time;
    
    // 直接根据地形状态决定效率模式（不需要额外的接地检测）
    WheelEfficiencyMode target_mode = DetermineEfficiencyMode(current_terrain_state_);
    
    // 如果模式改变，更新轮子效率
    if (target_mode != current_mode_) {
        current_mode_ = target_mode;
        UpdateWheelEfficiency();
        
        // 发布状态变化
        std_msgs::String status_msg;
        switch (current_mode_) {
            case WheelEfficiencyMode::NORMAL:
                status_msg.data = "normal";
                break;
            case WheelEfficiencyMode::WATER:
                status_msg.data = "water";
                break;
            case WheelEfficiencyMode::AIRBORNE:
                status_msg.data = "airborne";
                break;
            default:
                status_msg.data = "disabled";
        }
        status_pub_.publish(status_msg);
    }
    
    // 持续施加阻力
    ApplyResistanceForces();
    
    // 定期打印状态
    if (debug_mode_ && (current_time - last_status_print_time_).Double() > status_print_interval_) {
        PrintStatusInfo();
        last_status_print_time_ = current_time;
    }
    
    last_terrain_state_ = current_terrain_state_;
    last_mode_ = current_mode_;
    debug_counter_++;
}


void WheelEfficiencyPlugin::UpdateWheelEfficiency() {
    FrictionParams target_friction;
    DampingParams target_damping;
    std::string mode_name;
    
    switch (current_mode_) {
        case WheelEfficiencyMode::NORMAL:
            target_friction = normal_friction_;
            target_damping = normal_damping_;
            mode_name = "NORMAL (Land Grounded)";
            gzmsg << "[wheel_efficiency] ✓✓✓ Switching to NORMAL mode - Full efficiency!" << std::endl;
            break;
            
        case WheelEfficiencyMode::WATER:
            target_friction = water_friction_;
            target_damping = water_damping_;
            mode_name = "WATER";
            gzwarn << "[wheel_efficiency] ≈≈≈ Switching to WATER mode - Low efficiency!" << std::endl;
            break;
            
        case WheelEfficiencyMode::AIRBORNE:
            target_friction = airborne_friction_;
            target_damping = airborne_damping_;
            mode_name = "AIRBORNE";
            gzwarn << "[wheel_efficiency] ✗✗✗ Switching to AIRBORNE mode - No traction!" << std::endl;
            break;
            
        default:
            gzerr << "[wheel_efficiency] Unknown mode!" << std::endl;
            return;
    }
    
    // 更新所有轮子
    for (auto& wheel_link : wheel_links_) {
        // 设置摩擦力
        if (control_mode_ == ControlMode::FRICTION_ONLY || 
            control_mode_ == ControlMode::COMBINED) {
            SetWheelFriction(wheel_link, 
                           target_friction.mu1, 
                           target_friction.mu2,
                           target_friction.slip1,
                           target_friction.slip2);
        }
        
        // 设置阻尼
        if (control_mode_ == ControlMode::DAMPING_ONLY || 
            control_mode_ == ControlMode::COMBINED) {
            wheel_link->SetLinearDamping(target_damping.linear_damping);
            wheel_link->SetAngularDamping(target_damping.angular_damping);
        }
    }
    
    gzmsg << "[wheel_efficiency] Mode: " << mode_name 
          << " | μ=" << target_friction.mu1 
          << " | damping=" << target_damping.linear_damping << std::endl;
}

bool WheelEfficiencyPlugin::SetWheelFriction(physics::LinkPtr wheel_link, 
                                             double mu1, double mu2, 
                                             double slip1, double slip2) {
    if (!wheel_link) {
        return false;
    }
    
    auto collisions = wheel_link->GetCollisions();
    for (auto& collision : collisions) {
        auto surface = collision->GetSurface();
        if (surface && surface->FrictionPyramid()) {
            // 只设置摩擦系数
            surface->FrictionPyramid()->SetMuPrimary(mu1);
            surface->FrictionPyramid()->SetMuSecondary(mu2);
        }
    }
    
    return true;
}


void WheelEfficiencyPlugin::ApplyResistanceForces() {
    DampingParams* damping = nullptr;
    
    switch (current_mode_) {
        case WheelEfficiencyMode::NORMAL:
            damping = &normal_damping_;
            break;
        case WheelEfficiencyMode::WATER:
            damping = &water_damping_;
            break;
        case WheelEfficiencyMode::AIRBORNE:
            damping = &airborne_damping_;
            break;
        default:
            return;
    }
    
    if (damping->resistance_force <= 0.0) {
        return;
    }
    
    for (auto& wheel_link : wheel_links_) {
        ignition::math::Vector3d linear_vel = wheel_link->WorldLinearVel();
        ignition::math::Vector3d angular_vel = wheel_link->WorldAngularVel();
        
        double speed = linear_vel.Length();
        if (speed > 0.01) {
            ignition::math::Vector3d resistance_force = 
                -linear_vel.Normalized() * damping->resistance_force * speed;
            wheel_link->AddForce(resistance_force);
        }
        
        double angular_speed = angular_vel.Length();
        if (angular_speed > 0.01) {
            ignition::math::Vector3d resistance_torque = 
                -angular_vel.Normalized() * damping->resistance_force * 0.1 * angular_speed;
            wheel_link->AddTorque(resistance_torque);
        }
    }
}

void WheelEfficiencyPlugin::PrintStatusInfo() {
    std::string mode_str;
    switch (current_mode_) {
        case WheelEfficiencyMode::NORMAL:
            mode_str = "NORMAL";
            break;
        case WheelEfficiencyMode::WATER:
            mode_str = "WATER";
            break;
        case WheelEfficiencyMode::AIRBORNE:
            mode_str = "AIRBORNE";
            break;
        default:
            mode_str = "DISABLED";
    }
}


GZ_REGISTER_MODEL_PLUGIN(WheelEfficiencyPlugin)

} // namespace gazebo


