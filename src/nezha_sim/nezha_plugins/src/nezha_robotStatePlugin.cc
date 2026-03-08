#include "nezha_robotStatePlugin.hh"
#include "nezha_getWavefield.hh"

#include <nezha_plugins/GetPhaseSample.h> 

// Gazebo physics headers
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <thread>



static bool WaitForClientExistence(ros::ServiceClient &client, const std::string &name, double timeout_s = 2.0) {
  double waited = 0.0;
  const double step = 0.1;
  while (waited < timeout_s) {
    if (client && client.exists()) return true;
    ros::Duration(step).sleep();
    waited += step;
  }
  ROS_WARN_STREAM("[RobotStatePlugin] Service " << name << " did not appear after " << timeout_s << "s");
  return false;
}


template<typename Srv, typename Req>
static void SafeAsyncCall(ros::ServiceClient &client, const std::string &name, const Req &req) {
  if (!client) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[RobotStatePlugin] client uninitialized for " << name);
    return;
  }
  if (!client.exists()) {
    ROS_WARN_STREAM_THROTTLE(5.0, "[RobotStatePlugin] service " << name << " not available, skipping call");
    return;
  }


  Srv srv;
  srv.request = req; 

  std::thread([client, srv, name]() mutable {
    bool ok = false;
    try {
      ok = client.call(srv);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("[RobotStatePlugin] exception when calling service " << name << ": " << e.what());
    }
    if (!ok) {
      ROS_WARN_STREAM_THROTTLE(5.0, "[RobotStatePlugin] async call to " << name << " returned false");
    }
  }).detach();
}



namespace gazebo {
double RobotStatePlugin::QuerySurfaceZ(double x, double y) const
{
  if (!world_) {
    ROS_WARN_THROTTLE(5.0, "[RobotStatePlugin] Invalid world pointer!");
    return 0.0;
  }

  double simTime = world_->SimTime().Double();

  // 使用全局 GetWavefield
  if (auto wf = asv::GetWavefield(world_)) {
    // 使用 WavefieldSampler 的静态方法
    auto params = wf->GetParameters();
    if (params) {
      asv::Point3 point(x, y, 0.0);
      double depth = asv::WavefieldSampler::ComputeDepthDirectly(*params, point, simTime);
      
      // depth 是从水面到点的深度，如果点在水面上方则为负
      // 水面高度 = point.z + depth
      double surfaceZ = 0.0 - depth;  // 假设参考点 z=0
      
      static double lastPrintTime = 0.0;
      if (simTime - lastPrintTime > 1.0) {
        ROS_DEBUG("[RobotStatePlugin] QuerySurfaceZ t=%.2f pos=(%.2f, %.2f) z=%.3f",
                  simTime, x, y, surfaceZ);
        lastPrintTime = simTime;
      }
      
      return surfaceZ;
    }
  }

  static bool warnedOnce = false;
  if (!warnedOnce) {
    ROS_WARN("[RobotStatePlugin] No wavefield available! Returning z=0.0");
    warnedOnce = true;
  }
  return 0.0;
}


bool RobotStatePlugin::BindWavefieldOnce()
{
  if (!world_) return false;
  if (waveBound_) return true;

#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::physics::ModelPtr waveModel = world_->ModelByName(wave_model_name_);
#else
  gazebo::physics::ModelPtr waveModel;
  {
    auto models = world_->GetModels();
    for (auto const& m : models) {
      if (m->GetName() == "ocean_waves") { 
        waveModel = m; 
        break; 
      }
    }
  }
#endif

  // 如果没找到，尝试模糊匹配
  if (!waveModel) {
#if GAZEBO_MAJOR_VERSION >= 9
    for (auto const& m : world_->Models()) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || 
          n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#else
    auto models = world_->GetModels();
    for (auto const& m : models) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || 
          n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#endif
  }

  if (!waveModel) {
    const auto now = world_->SimTime();
    if ((now - lastPrint_).Double() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[RobotStatePlugin] Cannot bind wavefield: "
                             "model 'ocean_waves' not found.");
      lastPrint_ = now;
    }
    return false;
  }

  // 查找 link
  gazebo::physics::LinkPtr waveLink;
  auto links = waveModel->GetLinks();
  for (auto const& l : links) {
    if (l->GetName() == wave_link_name_) { 
      waveLink = l; 
      break; 
    }
  }
  if (!waveLink && !links.empty()) waveLink = links.front();

  waveModel_ = waveModel;
  waveLink_  = waveLink;
  waveBound_ = true;

  ROS_INFO_STREAM("[RobotStatePlugin] ✓ Bound to '" 
                  << waveModel_->GetName() << "' / '" 
                  << (waveLink_ ? waveLink_->GetName() : "(none)") << "'");
  return true;
}


void RobotStatePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  world_ = model_->GetWorld();
  lastUpdateTime_ = world_->SimTime();

  gzNode_.reset(new gazebo::transport::Node());
  gzNode_->Init(world_->Name());

  std::string topic = "/wave/" + model_->GetName() + "/surface_z";

  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "gazebo_robot_state_plugin",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }


  std::string ns;
  if (_sdf->HasElement("namespace")) {
    ns = _sdf->Get<std::string>("namespace");
  }
  if (ns.empty()) {

    ns = "/" + model_->GetName();
  }

  if (!ns.empty() && ns[0] != '/') ns = "/" + ns;

  nh_ = std::make_unique<ros::NodeHandle>(ns);
  ROS_INFO_STREAM("[RobotStatePlugin] ROS namespace set to " << nh_->getNamespace());

  InitSDFParameters(_sdf);
  InitServiceClients();
  // 初始化 wavefield 绑定
  lastPrint_ = world_->SimTime();
  
  // 尝试立即绑定
  if (!BindWavefieldOnce()) {
    ROS_INFO("[RobotStatePlugin] WavefieldEntity not yet available. "
             "Will retry in OnUpdate...");
  } else {
    ROS_INFO("[RobotStatePlugin] ✓ Wavefield bound immediately on Load.");
  }
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    [this](const common::UpdateInfo&) { this->OnUpdate(); }
  );

  ROS_INFO_STREAM("[OptimizedRobotStatePlugin] Loaded for " << model_->GetName());
}



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
    if (_sdf->HasElement("wave_model_name")) {
    wave_model_name_ = _sdf->Get<std::string>("wave_model_name");
  } else {
    wave_model_name_ = "ocean_waves";  // 默认值
  }
  
  if (_sdf->HasElement("wave_link_name")) {
    wave_link_name_ = _sdf->Get<std::string>("wave_link_name");
  } else {
    wave_link_name_ = "ocean_waves_link";  // 默认值
  }
  
  ROS_INFO_STREAM("[RobotStatePlugin] Wave model: " << wave_model_name_ 
                  << ", link: " << wave_link_name_);
  ROS_INFO_STREAM("[RobotStatePlugin] params: model_ns=/" << model_->GetName()
                  << " default_current_velocity=" << default_current_velocity_
                  << " updateInterval=" << updateInterval_);
}
ros::ServiceClient get_phase_client_;

void RobotStatePlugin::InitServiceClients()
{

  auto tryServiceNames = [this](const std::vector<std::string> &candidates) -> std::string {
    for (const auto &name : candidates) {
      std::string resolved = nh_->resolveName(name);
      if (ros::service::exists(resolved, false)) {  
        ROS_INFO_STREAM("[RobotStatePlugin] ✅ Found service: " << resolved);
        return resolved;
      }
    }
    return "";  
  };


  std::string model_ns = "/" + model_->GetName();
  

  {
    std::vector<std::string> candidates = {
      model_ns + "/set_fluid_density",
      "/hydrodynamics/set_fluid_density",
      "set_fluid_density"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      fluid_density_client_ = nh_->serviceClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ set_fluid_density service not found");
    }
  }


  {
    std::vector<std::string> candidates = {
      model_ns + "/set_volume_scaling",
      "/hydrodynamics/set_volume_scaling",
      "set_volume_scaling"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      volume_scaling_client_ = nh_->serviceClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ set_volume_scaling service not found");
    }
  }


  {
    std::vector<std::string> candidates = {
      model_ns + "/set_added_mass_scaling",
      "/hydrodynamics/set_added_mass_scaling",
      "set_added_mass_scaling"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      added_mass_scaling_client_ = nh_->serviceClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ set_added_mass_scaling service not found");
    }
  }


  {
    std::vector<std::string> candidates = {
      model_ns + "/set_damping_scaling",
      "/hydrodynamics/set_damping_scaling",
      "set_damping_scaling"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      damping_scaling_client_ = nh_->serviceClient<uuv_gazebo_ros_plugins_msgs::SetFloat>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ set_damping_scaling service not found");
    }
  }

  {
    size_t thruster_count = sizeof(this->thruster_clients_) / sizeof(this->thruster_clients_[0]);
    for (size_t i = 0; i < thruster_count; ++i) {
      std::vector<std::string> candidates = {
        model_ns + "/thrusters/" + std::to_string(i) + "/set_thrust_force_efficiency",
        "/hydrodynamics/thrusters/" + std::to_string(i) + "/set_thrust_force_efficiency",
        "thrusters/" + std::to_string(i) + "/set_thrust_force_efficiency"
      };
      std::string found = tryServiceNames(candidates);
      if (!found.empty()) {
        thruster_clients_[i] = nh_->serviceClient<uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency>(found);
      } else {
        ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ thruster " << i << " service not found");
      }
    }
  }

  {
    std::vector<std::string> candidates = {
      model_ns + "/set_current_velocity",
      "/hydrodynamics/set_current_velocity",
      "set_current_velocity"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      set_current_velocity_client_ = nh_->serviceClient<uuv_world_ros_plugins_msgs::SetCurrentVelocity>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ set_current_velocity service not found");
    }
  }


  {
    std::vector<std::string> candidates = {
      "/transmedia/get_phase_sample",
      "transmedia/get_phase_sample",
      "get_phase_sample"
    };
    std::string found = tryServiceNames(candidates);
    if (!found.empty()) {
      get_phase_client_ = nh_->serviceClient<nezha_plugins::GetPhaseSample>(found);
    } else {
      ROS_WARN_STREAM("[RobotStatePlugin] ⚠️ get_phase_sample service not found (will retry in OnUpdate)");
    }
  }

  ROS_INFO_STREAM("[RobotStatePlugin] ✅ InitServiceClients finished in <50ms");
}




bool QueryWaterPhase(std::string& phase, double& surfaceZ)
{
  if (!get_phase_client_) {
    ROS_WARN_THROTTLE(5.0, "[RobotStatePlugin] get_phase_client_ uninitialized");
    return false;
  }
  if (!get_phase_client_.exists()) {
    ROS_WARN_THROTTLE(5.0, "[RobotStatePlugin] get_phase_client_ not available");
    return false;
  }

  nezha_plugins::GetPhaseSample srv;
  if (!get_phase_client_.call(srv)) {
    ROS_WARN_THROTTLE(5.0, "[RobotStatePlugin] call to get_phase_client_ failed");
    return false;
  }

  phase    = srv.response.phase_name;
  surfaceZ = srv.response.surface_z;
  return true;
}

void RobotStatePlugin::OnUpdate()
{
  if (!waveBound_) {
    if (BindWavefieldOnce()) {
      ROS_INFO("[RobotStatePlugin] ✓ Wavefield bound successfully in OnUpdate.");
    } else {
      const auto now = world_->SimTime();
      if ((now - lastPrint_).Double() > 1.0) {
        ROS_WARN("[RobotStatePlugin] ⏳ Waiting for wave model 'nezha_ocean_waves'...");
        lastPrint_ = now;
      }
    }
  }

  common::Time now = world_->SimTime();
  if ((now - lastUpdateTime_).Double() < updateInterval_) return;

  auto pose = model_->WorldPose();
  double x = pose.Pos().X();
  double y = pose.Pos().Y();

  double eta = QuerySurfaceZ(x, y);
  surfaceZ_ = eta;

  double z = pose.Pos().Z();
  

  bool targetDry = (z > eta );  

  if (targetDry != lastState_) {
    if (targetDry) {
      SetUnderwaterParams(0.0);
      SetThrusterEfficiency(0.0);
      SetCurrentVelocity(0.0, 0.0, 0.0);
      lastState_ = true;
      ROS_INFO("[RobotStatePlugin] → Switched to DRY state");
    } else {
      SetUnderwaterParams(1.0);
      SetThrusterEfficiency(1.0);
      SetCurrentVelocity(default_current_velocity_, 0.0, 0.0);
      lastState_ = false;
      ROS_INFO("[RobotStatePlugin] → Switched to UNDERWATER state");
    }
  }

  lastUpdateTime_ = now;
}




void RobotStatePlugin::SetUnderwaterParams(double scale)
{
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request srv_req;
  srv_req.data = scale * default_fluid_density_;
  SafeAsyncCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(fluid_density_client_, nh_->resolveName("set_fluid_density"), srv_req);

  srv_req.data = scale * default_volume_scaling_;
  SafeAsyncCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(volume_scaling_client_, nh_->resolveName("set_volume_scaling"), srv_req);

  srv_req.data = scale * default_added_mass_scaling_;
  SafeAsyncCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(added_mass_scaling_client_, nh_->resolveName("set_added_mass_scaling"), srv_req);

  srv_req.data = scale * default_damping_scaling_;
  SafeAsyncCall<uuv_gazebo_ros_plugins_msgs::SetFloat>(damping_scaling_client_, nh_->resolveName("set_damping_scaling"), srv_req);
}



void RobotStatePlugin::SetThrusterEfficiency(double eff)
{
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request srv_req;
  srv_req.efficiency = eff;

  size_t thruster_count = sizeof(this->thruster_clients_) / sizeof(this->thruster_clients_[0]);
  for (size_t i = 0; i < thruster_count; ++i) {
    std::string name = nh_->resolveName("thrusters/" + std::to_string(i) + "/set_thrust_force_efficiency");
    SafeAsyncCall<uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency>(thruster_clients_[i], name, srv_req);
  }
}


void RobotStatePlugin::SetCurrentVelocity(double vel, double horz, double vert)
{
  uuv_world_ros_plugins_msgs::SetCurrentVelocity::Request srv_req;
  srv_req.velocity = vel;
  srv_req.horizontal_angle = horz;
  srv_req.vertical_angle = vert;
SafeAsyncCall<uuv_world_ros_plugins_msgs::SetCurrentVelocity>(set_current_velocity_client_, nh_->resolveName("set_current_velocity"), srv_req);

}


GZ_REGISTER_MODEL_PLUGIN(RobotStatePlugin)

} 

