//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
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

GZ_REGISTER_MODEL_PLUGIN(NezhaUnderwaterObjectPlugin)

NezhaUnderwaterObjectPlugin::NezhaUnderwaterObjectPlugin() : useGlobalCurrent(true)
{
}

NezhaUnderwaterObjectPlugin::~NezhaUnderwaterObjectPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
}

void NezhaUnderwaterObjectPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf)
{
  // !!! DEBUG PRINT START !!!
  gzmsg << "════════════════════════════════════════" << std::endl;
  gzmsg << "[NezhaUnderwater] Load() STARTED for model: " << (_model ? _model->GetName() : "NULL") << std::endl;

  GZ_ASSERT(_model != NULL, "Invalid model pointer");
  GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");

  this->model = _model;
  
  std::string modelName = this->model->GetName(); 
  if (modelName.find("nezha") == std::string::npos && modelName.find("Nezha") == std::string::npos) {
      gzmsg << "[NezhaUnderwater] Ignoring non-Nezha model: " << modelName << std::endl;
      return; 
  }
  
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
    GZ_ASSERT(!flowTopic.empty(), "Fluid velocity topic tag cannot be empty");
    this->flowSubscriber = this->node->Subscribe(flowTopic,
      &NezhaUnderwaterObjectPlugin::UpdateFlowVelocity, this);
  }

  double fluidDensity = 1028.0;
  if (_sdf->HasElement("fluid_density"))
    fluidDensity = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("use_global_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_current");

  bool debugFlag = false;
  if (_sdf->HasElement("debug"))
    debugFlag = static_cast<bool>(_sdf->Get<int>("debug"));

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
        if (linkName.find("base_link") != std::string::npos)
          this->baseLinkName = linkName;

        link = this->model->GetLink(linkName);
        if (!link)
        {
          gzwarn << "[NezhaUnderwater] Specified link [" << linkName << "] not found." << std::endl;
          continue;
        }
      }
      else
      {
        gzwarn << "[NezhaUnderwater] Attribute name missing from link" << std::endl;
        continue;
      }

      // !!! CRITICAL SECTION START !!!
      gzmsg << "[NezhaUnderwater] Creating HydrodynamicModel for: " << linkName << std::endl;

      // FIXED: Added nezha:: namespace
      nezha::HydrodynamicModelPtr hydro;
      try {
          // FIXED: Added nezha:: namespace
          hydro.reset(
            nezha::HydrodynamicModelFactory::GetInstance().CreateHydrodynamicModel(
            linkElem, link));
      } catch (const std::exception& e) {
          gzerr << "[NezhaUnderwater] EXCEPTION in Factory: " << e.what() << std::endl;
          continue;
      } catch (...) {
          gzerr << "[NezhaUnderwater] UNKNOWN EXCEPTION in Factory." << std::endl;
          continue;
      }
      
      // !!! SAFETY CHECK !!!
      if (!hydro) {
          gzerr << "[NezhaUnderwater] ✗ Factory returned NULL for link: " << linkName 
                << ". Check if SDF parameters match the Factory expectations." << std::endl;
          continue;
      }

      hydro->SetFluidDensity(fluidDensity);
      hydro->SetGravity(gAcc);
      hydro->SetEnabled(true);
      
      if (debugFlag)
        this->InitDebug(link, hydro);

      this->models[link] = hydro;
      
      // Only print if valid
      if (this->models[link]) {
          this->models[link]->Print("all");
          gzmsg << "[NezhaUnderwater] ✓ Successfully loaded model for: " << linkName << std::endl;
      }
      // !!! CRITICAL SECTION END !!!
    }  
  }  

  gzmsg << "════════════════════════════════════════" << std::endl;
  gzmsg << "[NezhaUnderwater] Load() COMPLETE. Models loaded: " << this->models.size() << std::endl;
  
 if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "nezha_underwater_object_plugin",
              ros::init_options::NoSigintHandler);
  }
  
  this->rosNode.reset(new ros::NodeHandle(""));
  
  for (auto& pair : this->models)
  {
    physics::LinkPtr link = pair.first;
    std::string linkName = link->GetName();
    std::string ns = this->model->GetName() + "/" + linkName;
    
    this->services[linkName + "/get_fluid_density"] = 
      this->rosNode->advertiseService(ns + "/get_fluid_density", &NezhaUnderwaterObjectPlugin::GetFluidDensity, this);
    this->services[linkName + "/set_fluid_density"] = 
      this->rosNode->advertiseService(ns + "/set_fluid_density", &NezhaUnderwaterObjectPlugin::SetFluidDensity, this);
    this->services[linkName + "/get_volume_scaling"] = 
      this->rosNode->advertiseService(ns + "/get_volume_scaling", &NezhaUnderwaterObjectPlugin::GetVolumeScaling, this);
    this->services[linkName + "/set_volume_scaling"] = 
      this->rosNode->advertiseService(ns + "/set_volume_scaling", &NezhaUnderwaterObjectPlugin::SetVolumeScaling, this);
    this->services[linkName + "/get_added_mass_scaling"] = 
      this->rosNode->advertiseService(ns + "/get_added_mass_scaling", &NezhaUnderwaterObjectPlugin::GetAddedMassScaling, this);
    this->services[linkName + "/set_added_mass_scaling"] = 
      this->rosNode->advertiseService(ns + "/set_added_mass_scaling", &NezhaUnderwaterObjectPlugin::SetAddedMassScaling, this);
    this->services[linkName + "/get_damping_scaling"] = 
      this->rosNode->advertiseService(ns + "/get_damping_scaling", &NezhaUnderwaterObjectPlugin::GetDampingScaling, this);
    this->services[linkName + "/set_damping_scaling"] = 
      this->rosNode->advertiseService(ns + "/set_damping_scaling", &NezhaUnderwaterObjectPlugin::SetDampingScaling, this);
  }
  
  this->Connect();
}

// FIXED: Added nezha:: namespace to argument type
void NezhaUnderwaterObjectPlugin::InitDebug(physics::LinkPtr _link,
  nezha::HydrodynamicModelPtr _hydro)
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

void NezhaUnderwaterObjectPlugin::Init()
{
}

void NezhaUnderwaterObjectPlugin::Update(const common::UpdateInfo &_info)
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
  
  static double lastPrintTime = 0.0;
  const double printInterval = 1.0; 
  bool shouldPrint = (time - lastPrintTime) >= printInterval;
  
  if (shouldPrint) {
    lastPrintTime = time;
  }

  for (auto it = models.begin(); it != models.end(); ++it)
  {
    physics::LinkPtr link = it->first;
    // FIXED: Added nezha:: namespace
    nezha::HydrodynamicModelPtr hydro = it->second;
    
    // Safety check for null hydro
    if (!hydro) continue;

    std::string linkName = link->GetName();
    bool isBaseLink = (linkName.find("base_link") != std::string::npos);
    
    // Calculate forces
    // FIXED: Added nezha:: namespace
    nezha::HydrodynamicModel::ForceReport report;
    if (isBaseLink) {
      report = hydro->UpdateForces(time, this->flowVelocity);
    }
    
    
    hydro->ApplyHydrodynamicForces(time, this->flowVelocity);

    ignition::math::Vector3d force = clampVec(
      hydro->GetStoredVector(UUV_ADDED_MASS_FORCE), MAX_FORCE);
    ignition::math::Vector3d torque = clampVec(
      hydro->GetStoredVector(UUV_ADDED_MASS_TORQUE), MAX_TORQUE);

    if (!isValidVec(force) || !isValidVec(torque)) {
      continue;
    }
    
    this->PublishRestoringForce(link);
    this->PublishHydrodynamicWrenches(link);
    this->PublishCurrentVelocityMarker();
    this->PublishIsSubmerged();
  }
}

void NezhaUnderwaterObjectPlugin::Connect()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&NezhaUnderwaterObjectPlugin::Update,
                    this, _1));
}

void NezhaUnderwaterObjectPlugin::PublishCurrentVelocityMarker() {}
void NezhaUnderwaterObjectPlugin::PublishIsSubmerged() {}

void NezhaUnderwaterObjectPlugin::UpdateFlowVelocity(ConstVector3dPtr &_msg)
{
  if (this->useGlobalCurrent)
  {
    double x = _msg->x();
    double y = _msg->y();
    double z = _msg->z();
    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
        this->flowVelocity.X() = x;
        this->flowVelocity.Y() = y;
        this->flowVelocity.Z() = z;
    }
  }
}

void NezhaUnderwaterObjectPlugin::PublishRestoringForce(physics::LinkPtr _link)
{
  if (this->models.count(_link) && this->models[_link] && this->models[_link]->GetDebugFlag())
  {
    ignition::math::Vector3d restoring = this->models[_link]->GetStoredVector(RESTORING_FORCE);
    msgs::WrenchStamped msg;
    this->GenWrenchMsg(restoring, ignition::math::Vector3d(0, 0, 0), msg);
    this->hydroPub[_link->GetName() + "/restoring"]->Publish(msg);
  }
}

void NezhaUnderwaterObjectPlugin::PublishHydrodynamicWrenches(physics::LinkPtr _link)
{
  if (this->models.count(_link) && this->models[_link] && this->models[_link]->GetDebugFlag())
  {
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

void NezhaUnderwaterObjectPlugin::GenWrenchMsg(ignition::math::Vector3d _force,
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

  msgs::Set(msgTorque, ignition::math::Vector3d(_torque.X(), _torque.Y(), _torque.Z()));
  msgs::Set(msgForce, ignition::math::Vector3d(_force.X(), _force.Y(), _force.Z()));

  t->set_sec(curTime.sec);
  t->set_nsec(curTime.nsec);
}

// ROS Service Callbacks
bool NezhaUnderwaterObjectPlugin::GetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty() || !this->models.begin()->second) { res.data = 0.0; return false; }
  res.data = this->models.begin()->second->GetFluidDensity();
  return true;
}

bool NezhaUnderwaterObjectPlugin::SetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data <= 0.0) { res.success = false; return true; }
  for (auto& pair : this->models) { if(pair.second) pair.second->SetFluidDensity(req.data); }
  res.success = true;
  return true;
}

bool NezhaUnderwaterObjectPlugin::GetVolumeScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty() || !this->models.begin()->second) { res.data = 0.0; return false; }
  double value;
  if (this->models.begin()->second->GetParam("scaling_volume", value)) { res.data = value; return true; }
  return false;
}

bool NezhaUnderwaterObjectPlugin::SetVolumeScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0) { res.success = false; return true; }
  for (auto& pair : this->models) { if(pair.second) pair.second->SetParam("scaling_volume", req.data); }
  res.success = true; return true;
}

bool NezhaUnderwaterObjectPlugin::GetAddedMassScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty() || !this->models.begin()->second) { res.data = 0.0; return false; }
  double value;
  if (this->models.begin()->second->GetParam("scaling_added_mass", value)) { res.data = value; return true; }
  return false;
}

bool NezhaUnderwaterObjectPlugin::SetAddedMassScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0) { res.success = false; return true; }
  for (auto& pair : this->models) { if(pair.second) pair.second->SetParam("scaling_added_mass", req.data); }
  res.success = true; return true;
}

bool NezhaUnderwaterObjectPlugin::GetDampingScaling(
    uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res)
{
  if (this->models.empty() || !this->models.begin()->second) { res.data = 0.0; return false; }
  double value;
  if (this->models.begin()->second->GetParam("scaling_damping", value)) { res.data = value; return true; }
  return false;
}

bool NezhaUnderwaterObjectPlugin::SetDampingScaling(
    uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
    uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res)
{
  if (req.data < 0.0) { res.success = false; return true; }
  for (auto& pair : this->models) { if(pair.second) pair.second->SetParam("scaling_damping", req.data); }
  res.success = true; return true;
}

}  // namespace gazebo

