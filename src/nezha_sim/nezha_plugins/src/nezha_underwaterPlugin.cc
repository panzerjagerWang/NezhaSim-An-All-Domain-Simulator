
#include "nezha_underwaterPlugin.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <uuv_gazebo_plugins/Def.hh>

#define MAX_FORCE 1e6
#define MAX_TORQUE 1e6
namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(UnderwaterObjectPlugin)

UnderwaterObjectPlugin::UnderwaterObjectPlugin() : useGlobalCurrent(true)
{
}

UnderwaterObjectPlugin::~UnderwaterObjectPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
}

void UnderwaterObjectPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Invalid model pointer");
  GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");

  this->model = _model;
  this->world = _model->GetWorld();

  this->node = transport::NodePtr(new transport::Node());
  std::string worldName;
#if GAZEBO_MAJOR_VERSION >= 8
  worldName = this->world->Name();
#else
  worldName = this->world->GetName();
#endif
  this->node->Init(worldName);

  if (_sdf->HasElement("flow_velocity_topic"))
  {
    std::string flowTopic = _sdf->Get<std::string>("flow_velocity_topic");
    GZ_ASSERT(!flowTopic.empty(),
              "Fluid velocity topic tag cannot be empty");

    gzmsg << "Subscribing to current velocity topic: " << flowTopic
        << std::endl;
    this->flowSubscriber = this->node->Subscribe(flowTopic,
      &UnderwaterObjectPlugin::UpdateFlowVelocity, this);
  }

  double fluidDensity = 1028.0;

  if (_sdf->HasElement("fluid_density"))
    fluidDensity = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("use_global_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_current");

  bool debugFlag = false;
  if (_sdf->HasElement("debug"))
    debugFlag = static_cast<bool>(_sdf->Get<int>("debug"));

  ignition::math::Vector3d cob;

  double gAcc;
#if GAZEBO_MAJOR_VERSION >= 8
  gAcc = std::abs(this->world->Gravity().Z());
#else
  gAcc = std::abs(this->world->GetPhysicsEngine()->GetGravity().z);
#endif
  
  this->baseLinkName = std::string();
  
  if (_sdf->HasElement("link"))
  {
    for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      physics::LinkPtr link;
      std::string linkName = "";

      if (linkElem->HasAttribute("name"))
      {
        linkName = linkElem->Get<std::string>("name");
        std::size_t found = linkName.find("base_link");
        if (found != std::string::npos)
        {
          this->baseLinkName = linkName;
          gzmsg << "Name of the BASE_LINK: " << this->baseLinkName << std::endl;
        }

        link = this->model->GetLink(linkName);
        if (!link)
        {
          gzwarn << "Specified link [" << linkName << "] not found."
                 << std::endl;
          continue;
        }
      }
      else
      {
        gzwarn << "Attribute name missing from link [" << linkName
               << "]" << std::endl;
        continue;
      }

      HydrodynamicModelPtr hydro;
      hydro.reset(
        HydrodynamicModelFactory::GetInstance().CreateHydrodynamicModel(
        linkElem, link));
      
      hydro->SetFluidDensity(fluidDensity);
      hydro->SetGravity(gAcc);
      
{
    auto& registry = HydrodynamicModelRegistry::GetInstance();
    gzmsg << "[UnderwaterObjectPlugin] Model for link: " 
          << link->GetName() 
          << " | Total models: " << registry.Size() << std::endl;
}

      hydro->SetEnabled(true);
      
      if (debugFlag)
        this->InitDebug(link, hydro);

      this->models[link] = hydro;
      this->models[link]->Print("all");
      
    }  
    
  }  

  gzmsg << "════════════════════════════════════════" << std::endl;
  gzmsg << "UnderwaterObjectPlugin::Load 完成" << std::endl;
  gzmsg << "  this->models.size() = " << this->models.size() << std::endl;
  
 if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "underwater_object_plugin",
              ros::init_options::NoSigintHandler);
  }
  
  this->rosNode.reset(new ros::NodeHandle(""));
  
  // ✅ 为每个 link 注册 ROS 服务
  for (auto& pair : this->models)
  {
    physics::LinkPtr link = pair.first;
    std::string linkName = link->GetName();
    
    // 创建服务命名空间
    std::string ns = this->model->GetName() + "/" + linkName;
    
    gzmsg << "Registering ROS services for link: " << ns << std::endl;
    
    // 注册流体密度服务
    this->services[linkName + "/get_fluid_density"] = 
      this->rosNode->advertiseService(
        ns + "/get_fluid_density",
        &UnderwaterObjectPlugin::GetFluidDensity, this);
    
    this->services[linkName + "/set_fluid_density"] = 
      this->rosNode->advertiseService(
        ns + "/set_fluid_density",
        &UnderwaterObjectPlugin::SetFluidDensity, this);
    
    // 注册体积缩放服务
    this->services[linkName + "/get_volume_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/get_volume_scaling",
        &UnderwaterObjectPlugin::GetVolumeScaling, this);
    
    this->services[linkName + "/set_volume_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/set_volume_scaling",
        &UnderwaterObjectPlugin::SetVolumeScaling, this);
    
    // 注册附加质量缩放服务
    this->services[linkName + "/get_added_mass_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/get_added_mass_scaling",
        &UnderwaterObjectPlugin::GetAddedMassScaling, this);
    
    this->services[linkName + "/set_added_mass_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/set_added_mass_scaling",
        &UnderwaterObjectPlugin::SetAddedMassScaling, this);
    
    // 注册阻尼缩放服务
    this->services[linkName + "/get_damping_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/get_damping_scaling",
        &UnderwaterObjectPlugin::GetDampingScaling, this);
    
    this->services[linkName + "/set_damping_scaling"] = 
      this->rosNode->advertiseService(
        ns + "/set_damping_scaling",
        &UnderwaterObjectPlugin::SetDampingScaling, this);
  }
  
  gzmsg << "ROS services registered successfully" << std::endl;
{
    auto& registry = HydrodynamicModelRegistry::GetInstance();
    auto models = registry.GetModels();
    
    gzmsg << "  单例注册表大小 = " << models.size() << std::endl;
    
    if (models.empty()) {
        gzerr << "✗✗✗ 警告：单例注册表为空！" << std::endl;
    } else {
        gzmsg << "  注册表内容:" << std::endl;
        for (size_t i = 0; i < models.size(); ++i) {
            auto* model = models[i];
            if (model && model->GetLink()) {
                gzmsg << "    [" << i << "] " 
                      << model->GetLink()->GetScopedName() 
                      << " @ " << model << std::endl;
            } else {
                gzmsg << "    [" << i << "] NULL" << std::endl;
            }
        }
    }
}

  
{
    auto& registry = HydrodynamicModelRegistry::GetInstance();
    auto models = registry.GetModels();
    
    for (auto& pair : this->models) {
        auto* rawPtr = pair.second.get();
        auto it = std::find(models.begin(), models.end(), rawPtr);
        
        if (it == models.end()) {
            gzerr << "✗ 模型 " << pair.first->GetScopedName() 
                  << " 不在单例注册表中！" << std::endl;
        } else {
            gzmsg << "✓ 模型 " << pair.first->GetScopedName() 
                  << " 已在单例注册表中" << std::endl;
        }
    }
}
  
  gzmsg << "════════════════════════════════════════" << std::endl;

  this->Connect();
}

void UnderwaterObjectPlugin::InitDebug(physics::LinkPtr _link,
  HydrodynamicModelPtr _hydro)
{

  std::string rootTopic = "/debug/forces/" + _link->GetName() + "/";
  std::vector<std::string> topics {"restoring", "damping", "added_mass",
    "added_coriolis"};
  for (auto topic : topics)
  {
    this->hydroPub[_link->GetName() + "/" + topic] =
      this->node->Advertise<msgs::WrenchStamped>(rootTopic + topic);
  }

  _hydro->SetDebugFlag(true);
  _hydro->SetStoreVector(RESTORING_FORCE);
  _hydro->SetStoreVector(UUV_DAMPING_FORCE);
  _hydro->SetStoreVector(UUV_DAMPING_TORQUE);
  _hydro->SetStoreVector(UUV_ADDED_CORIOLIS_FORCE);
  _hydro->SetStoreVector(UUV_ADDED_CORIOLIS_TORQUE);
  _hydro->SetStoreVector(UUV_ADDED_MASS_FORCE);
  _hydro->SetStoreVector(UUV_ADDED_MASS_TORQUE);
}

void UnderwaterObjectPlugin::Init()
{

}

void UnderwaterObjectPlugin::Update(const common::UpdateInfo &_info)
{
  auto clampValue = [](double val, double maxVal) {
    return std::max(std::min(val, maxVal), -maxVal);
  };
  
  auto clampVec = [&](const ignition::math::Vector3d& v, double maxVal) {
    return ignition::math::Vector3d(
      clampValue(v.X(), maxVal),
      clampValue(v.Y(), maxVal),
      clampValue(v.Z(), maxVal)
    );
  };
  
  auto isValidVec = [](const ignition::math::Vector3d& v) {
    return std::isfinite(v.X()) && std::isfinite(v.Y()) && std::isfinite(v.Z());
  };

  double time = _info.simTime.Double();
  
  // ✅ 添加：定期打印的时间控制（每1秒打印一次）
  static double lastPrintTime = 0.0;
  const double printInterval = 1.0;  // 打印间隔（秒）
  bool shouldPrint = (time - lastPrintTime) >= printInterval;
  
  if (shouldPrint) {
    lastPrintTime = time;
  }

  for (std::map<gazebo::physics::LinkPtr,
       HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
  {
    physics::LinkPtr link = it->first;
    HydrodynamicModelPtr hydro = it->second;
    
    // ✅ 修改：只处理 base_link
    std::string linkName = link->GetName();
    bool isBaseLink = (linkName.find("base_link") != std::string::npos);
    
    // ✅ 添加：获取 link 的位置信息（只在是 base_link 时）
    ignition::math::Pose3d linkPose;
    ignition::math::Vector3d linkVel;
    if (isBaseLink) {
#if GAZEBO_MAJOR_VERSION >= 8
      linkPose = link->WorldPose();
      linkVel = link->RelativeLinearVel();
#else
      linkPose = link->GetWorldPose();
      linkVel = link->GetRelativeLinearVel();
#endif
    }
    
    if (!hydro->IsEnabled()) {
      continue;
    }
    
    double linearAccel, angularAccel;
#if GAZEBO_MAJOR_VERSION >= 8
    linearAccel = link->RelativeLinearAccel().Length();
    angularAccel = link->RelativeAngularAccel().Length();
#else
    linearAccel = link->GetRelativeLinearAccel().GetLength();
    angularAccel = link->GetRelativeAngularAccel().GetLength();
#endif

    GZ_ASSERT(!std::isnan(linearAccel) && !std::isnan(angularAccel),
      "Linear or angular accelerations are invalid.");
    (void)linearAccel;
    (void)angularAccel;
    
    // ✅ 修改：只在是 base_link 时获取 ForceReport
    HydrodynamicModel::ForceReport report;
    if (isBaseLink) {
      report = hydro->UpdateForces(time, this->flowVelocity);
    }
    
    // ✅ 修改：只打印 base_link 的信息
    if (shouldPrint && isBaseLink) {
      gzmsg << "\n========== Underwater Forces Report (t=" << time << "s) ==========" << std::endl;
      
      gzmsg << "[" << linkName << "] Position: " 
            << "x=" << std::fixed << std::setprecision(3) << linkPose.Pos().X() 
            << ", y=" << linkPose.Pos().Y() 
            << ", z=" << linkPose.Pos().Z() << " m" << std::endl;
      
      gzmsg << "  Velocity: " 
            << "vx=" << linkVel.X() 
            << ", vy=" << linkVel.Y() 
            << ", vz=" << linkVel.Z() << " m/s" << std::endl;
      
      gzmsg << "  Total Force:  [" 
            << report.totalForce.X() << ", " 
            << report.totalForce.Y() << ", " 
            << report.totalForce.Z() << "] N" << std::endl;
      
      gzmsg << "  Total Torque: [" 
            << report.totalTorque.X() << ", " 
            << report.totalTorque.Y() << ", " 
            << report.totalTorque.Z() << "] Nm" << std::endl;
      
      gzmsg << "  Damping Force:  [" 
            << report.dampingForce.X() << ", " 
            << report.dampingForce.Y() << ", " 
            << report.dampingForce.Z() << "] N" << std::endl;
      
      gzmsg << "  Damping Torque: [" 
            << report.dampingTorque.X() << ", " 
            << report.dampingTorque.Y() << ", " 
            << report.dampingTorque.Z() << "] Nm" << std::endl;
      
      gzmsg << "  Added Mass Force:  [" 
            << report.addedMassForce.X() << ", " 
            << report.addedMassForce.Y() << ", " 
            << report.addedMassForce.Z() << "] N" << std::endl;
      
      gzmsg << "  Buoyancy Force: [" 
            << report.buoyancyForce.X() << ", " 
            << report.buoyancyForce.Y() << ", " 
            << report.buoyancyForce.Z() << "] N" << std::endl;
      
      // 计算力的总大小
      double totalForceMag = report.totalForce.Length();
      double dampingForceMag = report.dampingForce.Length();
      double buoyancyForceMag = report.buoyancyForce.Length();
      
      gzmsg << "  Force Magnitudes: Total=" << totalForceMag 
            << " N, Damping=" << dampingForceMag 
            << " N, Buoyancy=" << buoyancyForceMag << " N" << std::endl;
      
      gzmsg << "============================================================\n" << std::endl;
    }
    
    hydro->ApplyHydrodynamicForces(time, this->flowVelocity);

    ignition::math::Vector3d force = clampVec(
      hydro->GetStoredVector(UUV_ADDED_MASS_FORCE), MAX_FORCE);
    ignition::math::Vector3d torque = clampVec(
      hydro->GetStoredVector(UUV_ADDED_MASS_TORQUE), MAX_TORQUE);

    if (!isValidVec(force) || !isValidVec(torque)) {
      gzerr << "Detected invalid force/torque, skipping this update." << std::endl;
      continue;
    }
    
    this->PublishRestoringForce(link);
    this->PublishHydrodynamicWrenches(link);
    this->PublishCurrentVelocityMarker();
    this->PublishIsSubmerged();
  }
}



void UnderwaterObjectPlugin::Connect()
{

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&UnderwaterObjectPlugin::Update,
                    this, _1));
}

void UnderwaterObjectPlugin::PublishCurrentVelocityMarker()
{

  return;
}

void UnderwaterObjectPlugin::PublishIsSubmerged()
{

  return;
}

void UnderwaterObjectPlugin::UpdateFlowVelocity(ConstVector3dPtr &_msg)
{
  if (this->useGlobalCurrent)
  {
    double x = _msg->x();
    double y = _msg->y();
    double z = _msg->z();

    if (std::isnan(x) || std::isnan(y) || std::isnan(z))
    {
      gzerr << "Invalid flow velocity (NaN detected). Ignoring update." << std::endl;
      return;
    }
    this->flowVelocity.X() = x;
    this->flowVelocity.Y() = y;
    this->flowVelocity.Z() = z;
  }
}

void UnderwaterObjectPlugin::PublishRestoringForce(
  physics::LinkPtr _link)
{
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;

    ignition::math::Vector3d restoring = this->models[_link]->GetStoredVector(
      RESTORING_FORCE);

    msgs::WrenchStamped msg;
    this->GenWrenchMsg(restoring, ignition::math::Vector3d(0, 0, 0), msg);
    this->hydroPub[_link->GetName() + "/restoring"]->Publish(msg);
  }
}

void UnderwaterObjectPlugin::PublishHydrodynamicWrenches(
  physics::LinkPtr _link)
{
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;
    msgs::WrenchStamped msg;
    ignition::math::Vector3d force, torque;

    force = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->hydroPub[_link->GetName() + "/added_mass"]->Publish(msg);

    force = this->models[_link]->GetStoredVector(UUV_DAMPING_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_DAMPING_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->hydroPub[_link->GetName() + "/damping"]->Publish(msg);

    force = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->hydroPub[_link->GetName() + "/added_coriolis"]->Publish(msg);
  }
}

void UnderwaterObjectPlugin::GenWrenchMsg(ignition::math::Vector3d _force,
  ignition::math::Vector3d _torque, gazebo::msgs::WrenchStamped &_output)
{
  common::Time curTime;
#if GAZEBO_MAJOR_VERSION >= 8
  curTime = this->world->SimTime();
#else
  curTime = this->world->GetSimTime();
#endif

  msgs::Wrench * wrench = _output.mutable_wrench();
  msgs::Time * t = _output.mutable_time();
  msgs::Vector3d * msgForce = wrench->mutable_force();
  msgs::Vector3d * msgTorque = wrench->mutable_torque();

  msgs::Set(msgTorque,
    ignition::math::Vector3d(_torque.X(), _torque.Y(), _torque.Z()));
  msgs::Set(msgForce,
    ignition::math::Vector3d(_force.X(), _force.Y(), _force.Z()));

  t->set_sec(curTime.sec);
  t->set_nsec(curTime.nsec);
}
// ✅ 在 namespace gazebo 的 } 之前添加这些函数

// ✅ 修正后的 ROS 服务回调函数

bool UnderwaterObjectPlugin::GetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty())
  {
    res.data = 0.0;  // ✅ GetFloat 只有 data 字段
    return false;     // ✅ 返回 false 表示服务调用失败
  }
  
  res.data = this->models.begin()->second->GetFluidDensity();
  return true;  // ✅ 返回 true 表示服务调用成功
}

bool UnderwaterObjectPlugin::SetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data <= 0.0)
  {
    gzerr << "Invalid fluid density: " << req.data << std::endl;
    res.success = false;  // ✅ SetFloat 有 success 字段
    return true;
  }
  
  for (auto& pair : this->models)
  {
    pair.second->SetFluidDensity(req.data);
  }
  
  gzmsg << "Fluid density set to: " << req.data << std::endl;
  res.success = true;
  return true;
}

bool UnderwaterObjectPlugin::GetVolumeScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty())
  {
    res.data = 0.0;  // ✅ 只设置 data
    return false;
  }
  
  double value;
  if (this->models.begin()->second->GetParam("scaling_volume", value))
  {
    res.data = value;
    return true;  // ✅ 返回 true 表示成功
  }
  else
  {
    res.data = 0.0;
    return false;  // ✅ 返回 false 表示失败
  }
}

bool UnderwaterObjectPlugin::SetVolumeScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0)
  {
    gzerr << "Invalid volume scaling: " << req.data << std::endl;
    res.success = false;
    return true;
  }
  
  for (auto& pair : this->models)
  {
    pair.second->SetParam("scaling_volume", req.data);
  }
  
  gzmsg << "Volume scaling set to: " << req.data << std::endl;
  res.success = true;
  return true;
}

bool UnderwaterObjectPlugin::GetAddedMassScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty())
  {
    res.data = 0.0;
    return false;
  }
  
  double value;
  if (this->models.begin()->second->GetParam("scaling_added_mass", value))
  {
    res.data = value;
    return true;
  }
  else
  {
    res.data = 0.0;
    return false;
  }
}

bool UnderwaterObjectPlugin::SetAddedMassScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0)
  {
    gzerr << "Invalid added mass scaling: " << req.data << std::endl;
    res.success = false;
    return true;
  }
  
  for (auto& pair : this->models)
  {
    pair.second->SetParam("scaling_added_mass", req.data);
  }
  
  gzmsg << "Added mass scaling set to: " << req.data << std::endl;
  res.success = true;
  return true;
}

bool UnderwaterObjectPlugin::GetDampingScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty())
  {
    res.data = 0.0;
    return false;
  }
  
  double value;
  if (this->models.begin()->second->GetParam("scaling_damping", value))
  {
    res.data = value;
    return true;
  }
  else
  {
    res.data = 0.0;
    return false;
  }
}

bool UnderwaterObjectPlugin::SetDampingScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0)
  {
    gzerr << "Invalid damping scaling: " << req.data << std::endl;
    res.success = false;
    return true;
  }
  
  for (auto& pair : this->models)
  {
    pair.second->SetParam("scaling_damping", req.data);
  }
  
  gzmsg << "Damping scaling set to: " << req.data << std::endl;
  res.success = true;
  return true;
}


}  // namespace gazebo

