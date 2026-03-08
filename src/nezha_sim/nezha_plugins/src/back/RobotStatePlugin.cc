#include "RobotStatePlugin.hh"

namespace gazebo {

void RobotStatePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // 初始化Gazebo核心组件
  model_ = _parent;
  world_ = model_->GetWorld();
  lastUpdateTime_ = world_->SimTime();

  // ① 创建 transport node（在 world_ 已经拿到后）
  gzNode_.reset(new gazebo::transport::Node());
  gzNode_->Init(world_->Name());

  // ② 话题名 = /wave/<boat_name>/surface_z
  std::string topic = "/wave/" + model_->GetName() + "/surface_z";
  surfaceSub_ = gzNode_->Subscribe(
      topic,
      &RobotStatePlugin::OnSurfaceZ,   // 回调
      this);

  gzdbg << "[RobotStatePlugin] Subscribing to " << topic << std::endl;

  // ROS初始化（支持无ROS环境）
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "gazebo_robot_state_plugin",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  nh_ = std::make_unique<ros::NodeHandle>();

  // 初始化参数和服务客户端
  InitSDFParameters(_sdf);
  InitServiceClients();
  
  const auto linkPtrs        = model_->GetLinks();                 
  const auto collisionMeshes = ::asv::CreateCollisionMeshes(linkPtrs); 

  links_.reserve(linkPtrs.size());
  for (size_t i = 0; i < linkPtrs.size(); ++i)                      
  {
    LinkHydroData data;
    data.link        = linkPtrs[i];
    auto surf = asv::TrianglesToCGALMesh(collisionMeshes[i]);
    data.initMeshes.push_back(surf);
    data.worldMeshes.push_back(std::make_shared<::asv::Mesh>(*surf));
    links_.push_back(data);  // 添加这行
  }

  // 注册Gazebo更新回调
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    [this](const common::UpdateInfo&) { this->OnUpdate(); }
  );

  ROS_INFO_STREAM("[OptimizedRobotStatePlugin] Loaded for " << model_->GetName());
}

// 初始化SDF参数
void RobotStatePlugin::InitSDFParameters(sdf::ElementPtr _sdf)
{
  auto getParam = [&](const char* name, auto def) {
    return _sdf->HasElement(name) ? _sdf->Get<double>(name) : def;
  };

  default_fluid_density_    = getParam("default_fluid_density", 1028.0);
  default_volume_scaling_   = getParam("default_volume_scaling", 1.0);
  default_added_mass_scaling_ = getParam("default_added_mass_scaling", 1.0);
  default_damping_scaling_  = getParam("default_damping_scaling", 1.0);
  water_surface_height_     = getParam("water_surface_height", 0.0);
  default_current_velocity_ = getParam("default_current_velocity", 1.0);
  updateInterval_           = getParam("update_interval", 0.1);
  epsDry_ = _sdf->HasElement("epsilon_dry") ? _sdf->Get<double>("epsilon_dry") : 0.02;
  epsWet_ = _sdf->HasElement("epsilon_wet") ? _sdf->Get<double>("epsilon_wet") : 0.98;
}

// 初始化服务客户端
void RobotStatePlugin::InitServiceClients()
{
  std::string ns = "/" + model_->GetName();

  fluid_density_client_ = CreateClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(ns + "/set_fluid_density");
  volume_scaling_client_  = CreateClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(ns + "/set_volume_scaling");
  added_mass_scaling_client_ = CreateClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(ns + "/set_added_mass_scaling");
  damping_scaling_client_ = CreateClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(ns + "/set_damping_scaling");

  for (int i = 0; i < 3; ++i) {
    thruster_clients_[i] = CreateClient<uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency>(
        ns + "/thrusters/" + std::to_string(i) + "/set_thrust_force_efficiency");
  }

  set_current_velocity_client_ = CreateClient<uuv_world_ros_plugins_msgs::SetCurrentVelocity>("/set_current_velocity");
}

void RobotStatePlugin::OnUpdate()
{
  //-------------------------------
  // 0) 频率控制
  //-------------------------------
  common::Time now = world_->SimTime();
  if ((now - lastUpdateTime_).Double() < updateInterval_)
    return;
  if (!hasSurface_)
    return;
    
  //-------------------------------
  // 1) 计算全模型平均浸没率 ζ
  //-------------------------------
  double globalZeta = 0.0;
  size_t counted    = 0;

  if (!links_.empty())                    // 波场 & 网格均有效
  {
    for (auto& lh : links_)
    {
      // a. 更新碰撞网格到世界坐标
      ignition::math::Pose3d pose = lh.link->WorldPose();
      for (size_t i = 0; i < lh.initMeshes.size(); ++i)
        ApplyPoseToMesh(pose, lh.initMeshes[i], lh.worldMeshes[i]);

      // b. 计算单 link ζ
      double zeta = ComputeImmersion(lh.worldMeshes, surfaceZ_);   // 新写法

      lh.immersion = zeta;

      globalZeta += zeta;
      ++counted;
    }
    if (counted > 0)
      globalZeta /= static_cast<double>(counted);      // 求平均
  }
  else
  {
    globalZeta = 0.0;                                  // 退化到"完全出水"视角
  }

  //-------------------------------
  // 2) 判定干 / 湿 状态
  //-------------------------------
  bool isDry = (globalZeta <= epsDry_);
  bool isWet = (globalZeta >= epsWet_);

  // 若无法计算 ζ，则用旧的质心高度判断兜底
  if (links_.empty())
  {
    isDry =  CheckWaterSurfaceState();  // 质心高于水面 → dry
    isWet = !isDry;                     // 其余视作 wet
  }

  //-------------------------------
  // 3) 状态机：仅在 Dry ↔ Wet 切换时执行服务调用
  //-------------------------------
  // lastState_ 语义：true  = Dry(出水)；false = Wet(完全浸没)
  if (isDry && lastState_ == false)                     // Wet → Dry
  {
    SetUnderwaterParams(0.0);                           // 关 UUV 水动力
    SetThrusterEfficiency(0.0);                         // 推进器停
    SetCurrentVelocity(0.0, 0.0, 0.0);                  // 洋流=0
    lastState_ = true;
  }
  else if (isWet && lastState_ == true)                 // Dry → Wet
  {
    SetUnderwaterParams(1.0);                           // 开 UUV 水动力
    SetThrusterEfficiency(1.0);                         // 推进器恢复
    SetCurrentVelocity(default_current_velocity_, 0.0, 0.0);
    lastState_ = false;
  }
  /* 0.02 < ζ < 0.98 : 半浸区间，保持当前设置，由 ASV 网格浮力接管 */

  //-------------------------------
  // 4) 更新时间戳
  //-------------------------------
  lastUpdateTime_ = now;
}

// 检查水面状态
bool RobotStatePlugin::CheckWaterSurfaceState()
{
#if GAZEBO_MAJOR_VERSION >= 9
  return model_->WorldPose().Pos().Z() > surfaceZ_;
#else
  return model_->GetWorldPose().pos.z > surfaceZ_;
#endif
}

// 状态切换处理
void RobotStatePlugin::HandleStateTransition(bool isAboveWater)
{
  if (isAboveWater) {
    SetUnderwaterParams(0.0);
    SetThrusterEfficiency(0.0);
    SetCurrentVelocity(0.0, 0.0, 0.0);
  } else {
    SetUnderwaterParams(1.0);
    SetThrusterEfficiency(1.0);
    SetCurrentVelocity(default_current_velocity_, 0.0, 0.0);
  }
}

// 修复消息回调函数
void RobotStatePlugin::OnSurfaceZ(const boost::shared_ptr<const gazebo::msgs::Vector3d> &_msg)
{
  surfaceZ_   = _msg->z();
  hasSurface_ = true;
}

// 设置水下参数
void RobotStatePlugin::SetUnderwaterParams(double scale)
{
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request srv_req;
  srv_req.data = scale * default_fluid_density_;
  AsyncServiceCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(fluid_density_client_, srv_req);

  srv_req.data = scale * default_volume_scaling_;
  AsyncServiceCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(volume_scaling_client_, srv_req);

  srv_req.data = scale * default_added_mass_scaling_;
  AsyncServiceCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(added_mass_scaling_client_, srv_req);

  srv_req.data = scale * default_damping_scaling_;
  AsyncServiceCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(damping_scaling_client_, srv_req);
}

// 设置推进器效率
void RobotStatePlugin::SetThrusterEfficiency(double eff)
{
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request srv_req;
  srv_req.efficiency = eff;
  for (auto& client : thruster_clients_) {
    AsyncServiceCall<uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency>(client, srv_req);
  }
}

// 设置洋流参数
void RobotStatePlugin::SetCurrentVelocity(double vel, double horz, double vert)
{
  uuv_world_ros_plugins_msgs::SetCurrentVelocity::Request srv_req;
  srv_req.velocity = vel;
  srv_req.horizontal_angle = horz;
  srv_req.vertical_angle = vert;
  AsyncServiceCall<uuv_world_ros_plugins_msgs::SetCurrentVelocity>(set_current_velocity_client_, srv_req);
}

// 将局部网格顶点变换到世界坐标
void ApplyPoseToMesh(const ignition::math::Pose3d& pose,
                     const std::shared_ptr<::asv::Mesh>& src,
                     std::shared_ptr<::asv::Mesh>& dst)
{
  namespace im = ignition::math;
  dst->clear();  // 简化处理：重新生成顶点；面拓扑沿用
  for (auto v: src->vertices())
  {
    auto p  = src->point(v);
    im::Vector3d lp(p.x(), p.y(), p.z());
    im::Vector3d wp = pose.CoordPositionAdd(lp);
    dst->add_vertex(::asv::Point3(wp.X(), wp.Y(), wp.Z()));  // 使用 Point3
  }
}


double RobotStatePlugin::ComputeImmersion(const std::vector<std::shared_ptr<::asv::Mesh>>& meshes, double waterZ)
{
    std::size_t wet = 0, total = 0;

    for (const auto& m : meshes)
    {
        for (auto v : m->vertices())
        {
            const auto& p = m->point(v);
            ++total;
            if (p.z() <= waterZ) ++wet;
        }
    }
    return total ? static_cast<double>(wet)/total : 0.0;
}


GZ_REGISTER_MODEL_PLUGIN(RobotStatePlugin)

} // namespace gazebo

