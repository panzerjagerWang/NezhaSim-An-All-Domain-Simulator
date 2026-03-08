
#include <ros/package.h>
#include <cmath>  
#include <algorithm> 
#include <array>  
#if defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wreorder"
#endif
#include "nrotorS_gazeboMotorModel.h"
#if defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include <ros/ros.h>
#include "Float32.pb.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>   
#include <cmath>
#include <gazebo/transport/transport.hh>   
#include <sdf/sdf.hh>                      
namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  if (publish_speed_) {
    turning_velocity_msg_.set_data(joint_->GetVelocity(0));
    motor_velocity_pub_->Publish(turning_velocity_msg_);
  }
  if (publish_position_) {
    position_msg_.set_data(joint_->Position(0));
    motor_position_pub_->Publish(position_msg_);
  }
  if (publish_force_) {
    force_msg_.set_data(joint_->GetForce(0));
    motor_force_pub_->Publish(force_msg_);
  }
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  model_ = _model;

  namespace_.clear();

if (_sdf->HasElement("ktIgeGainPubTopic")) {
  kt_ige_gain_pub_topic_ = _sdf->Get<std::string>("ktIgeGainPubTopic");
}
if (_sdf->HasElement("kqIgeGainPubTopic")) {
  kq_ige_gain_pub_topic_ = _sdf->Get<std::string>("kqIgeGainPubTopic");
}

if (_sdf->HasElement("ktGainPubTopic")) {
  kt_gain_pub_topic_ = _sdf->Get<std::string>("ktGainPubTopic");
}
if (_sdf->HasElement("kqGainPubTopic")) {
  kq_gain_pub_topic_ = _sdf->Get<std::string>("kqGainPubTopic");
}


  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());


  node_handle_->Init();

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor "
             "is attached.\n";

  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow(
        "[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_
                                                                << "\".");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow(
        "[gazebo_motor_model] Couldn't find specified link \"" << link_name_
                                                               << "\".");

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction =
        _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as "
               "turningDirection.\n";
  } else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or "
             "'ccw').\n";

  if (_sdf->HasElement("motorType")) {
    std::string motor_type = _sdf->GetElement("motorType")->Get<std::string>();
    if (motor_type == "velocity")
      motor_type_ = MotorType::kVelocity;
    else if (motor_type == "position")
      motor_type_ = MotorType::kPosition;
    else if (motor_type == "force") {
      motor_type_ = MotorType::kForce;
    } else
      gzerr << "[gazebo_motor_model] Please only use 'velocity', 'position' or "
               "'force' as motorType.\n";
  } else {
    gzwarn << "[gazebo_motor_model] motorType not specified, using velocity.\n";
    motor_type_ = MotorType::kVelocity;
  }

  if (motor_type_ == MotorType::kPosition) {
    if (_sdf->HasElement("joint_control_pid")) {
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 0;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = 0;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      pids_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    } else {
      pids_.Init(0, 0, 0, 0, 0, 0, 0);
      gzerr << "[gazebo_motor_model] PID values not found, Setting all values "
               "to zero!\n";
    }
  } else {
    pids_.Init(0, 0, 0, 0, 0, 0, 0);
  }

  getSdfParam<std::string>(
      _sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
      motor_speed_pub_topic_);

  if (_sdf->HasElement("motorPositionPubTopic")) {
    publish_position_ = true;
    motor_position_pub_topic_ =
        _sdf->GetElement("motorPositionPubTopic")->Get<std::string>();
  }
  if (_sdf->HasElement("motorForcePubTopic")) {
    publish_force_ = true;
    motor_force_pub_topic_ =
        _sdf->GetElement("motorForcePubTopic")->Get<std::string>();
  }

  getSdfParam<double>(
      _sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
      rotor_drag_coefficient_);
  getSdfParam<double>(
      _sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
      rolling_moment_coefficient_);
  getSdfParam<double>(
      _sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<std::string>(_sdf, "bladeType",  blade_type_,  blade_type_);
  getSdfParam<double>     (_sdf, "bladeRadius",blade_radius_,blade_radius_);
getSdfParam<bool>   (_sdf, "waterAnim",        water_anim_enabled_, water_anim_enabled_);
getSdfParam<double> (_sdf, "splashThreshold",  dR_splash_threshold_, dR_splash_threshold_);
getSdfParam<std::string>(_sdf, "rippleModelPath",  ripple_model_path_,  ripple_model_path_);
getSdfParam<std::string>(_sdf, "rippleModelName",  ripple_model_name_,  ripple_model_name_);

std::string data_dir  = ros::package::getPath("nezha_gazebo") + "/prop_data/";
std::string json_file = data_dir + "NWE_"+blade_type_ + ".json";
ROS_INFO_STREAM("[gazebo_motor_model] Using prop data file: " << json_file);
gzmsg << "[gazebo_motor_model] Using prop data file: " << json_file << std::endl;


std::ifstream jf(json_file);
if (!jf.is_open()) {
  gzerr << "[gazebo_motor_model] cannot open " << json_file << "\n";
} else {
  nlohmann::json j; jf >> j;


  if (j["KT"].contains("sr_formula")
      && j["KT"]["sr_formula"] == "compiled") {

    use_sr_model_   = true;
    kt_sr_.theta    = j["KT"]["theta"].get<std::vector<double>>();
    kt_sr_.boundary = j["KT"]["boundary"].get<double>();
    kq_sr_.theta    = j["KQ"]["theta"].get<std::vector<double>>();
    kq_sr_.boundary = j["KQ"]["boundary"].get<double>();

    gzmsg << "[gazebo_motor_model] Loaded SR gain model: "
          << blade_type_ << "\n";
  }

  else {
    d_ratio_grid_   = j["d_over_R"].get<std::vector<double>>();
    rpm_ratio_grid_ = j["rpm_ratio"].get<std::vector<double>>();
    kt_gain_table_  = j["KT"].get<std::vector<std::vector<double>>>();
    kq_gain_table_  = j["KQ"].get<std::vector<std::vector<double>>>();
    table_loaded_   = true;

    gzmsg << "[gazebo_motor_model] Loaded gain table: "
          << blade_type_ << "\n";
  }
}

  getSdfParam<double>(
      _sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(
      _sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(
      _sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(
      _sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);


  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMotorModel::OnUpdate, this, _1));


  rotor_velocity_filter_.reset(
      new FirstOrderFilter<double>(
          time_constant_up_, time_constant_down_, ref_motor_input_));
          
  ros::NodeHandle nh(namespace_);                 
  kt_gain_pub_ = nh.advertise<std_msgs::Float32>(kt_gain_pub_topic_, 10);
  kq_gain_pub_ = nh.advertise<std_msgs::Float32>(kq_gain_pub_topic_, 10);
  

std::string ige_json_file = data_dir + "IGE.json";
ROS_INFO_STREAM("[gazebo_motor_model] Loading IGE data file: " << ige_json_file);
gzmsg << "[gazebo_motor_model] Loading IGE data file: " << ige_json_file << std::endl;

std::ifstream ige_jf(ige_json_file);
if (!ige_jf.is_open()) {
  gzerr << "[gazebo_motor_model] cannot open " << ige_json_file << "\n";
} else {
  nlohmann::json ige_j; 
  ige_jf >> ige_j;
  
  if (ige_j["KT"].contains("sr_formula") && ige_j["KT"]["sr_formula"] == "compiled") {
    use_ige_model_ = true;
    kt_ige_.theta = ige_j["KT"]["theta"].get<std::vector<double>>();
    kt_ige_.boundary = ige_j["KT"]["boundary"].get<double>();
    kq_ige_.theta = ige_j["KQ"]["theta"].get<std::vector<double>>();
    kq_ige_.boundary = ige_j["KQ"]["boundary"].get<double>();
    ige_loaded_ = true;
    
    gzmsg << "[gazebo_motor_model] Loaded IGE SR gain model\n";
  }
}

kt_ige_gain_pub_ = nh.advertise<std_msgs::Float32>(kt_ige_gain_pub_topic_, 10);
kq_ige_gain_pub_ = nh.advertise<std_msgs::Float32>(kq_ige_gain_pub_topic_, 10);

                                  
std::string terrain_topic = "terrain_status";
if (!namespace_.empty()) {

  terrain_status_sub_ = nh.subscribe("/" + namespace_ + "/" + terrain_topic, 1, 
                                    &GazeboMotorModel::TerrainStatusCallback, this);
  gzmsg << "[gazebo_motor_model] Subscribing to terrain status on: /" 
        << namespace_ << "/" << terrain_topic << std::endl;
} else {
  terrain_status_sub_ = nh.subscribe(terrain_topic, 1, 
                                    &GazeboMotorModel::TerrainStatusCallback, this);
  gzmsg << "[gazebo_motor_model] Subscribing to terrain status on: " 
        << terrain_topic << std::endl;
}
  lastPrint_ = model_->GetWorld()->SimTime();
  
  if (!BindWavefieldOnce()) {
    gzmsg << "[gazebo_motor_model] WavefieldEntity not yet available. "
          << "Will retry in OnUpdate..." << std::endl;
  } else {
    gzmsg << "[gazebo_motor_model] ✓ Wavefield bound immediately on Load." << std::endl;
  }
}

double GazeboMotorModel::QuerySurfaceZ(double x, double y) const
{
  if (!model_ || !model_->GetWorld()) {
    gzwarn << "[gazebo_motor_model] Invalid world pointer!" << std::endl;
    return 0.0;
  }

  auto world = model_->GetWorld();
  double simTime = world->SimTime().Double();

  if (auto wf = asv::GetWavefield(world)) {
    auto params = wf->GetParameters();
    if (params) {
      asv::Point3 point(x, y, 0.0);
      double depth = asv::WavefieldSampler::ComputeDepthDirectly(*params, point, simTime);
      
      double surfaceZ = 0.0 - depth;
      
      static double lastPrintTime = 0.0;
      if (simTime - lastPrintTime > 1.0) {
        gzdbg << "[gazebo_motor_model] QuerySurfaceZ t=" << simTime 
              << " pos=(" << x << ", " << y << ") z=" << surfaceZ << std::endl;
        lastPrintTime = simTime;
      }
      
      return surfaceZ;
    }
  }

  static bool warnedOnce = false;
  if (!warnedOnce) {
    gzwarn << "[gazebo_motor_model] No wavefield available! Returning z=0.0" << std::endl;
    warnedOnce = true;
  }
  return 0.0;
}

bool GazeboMotorModel::BindWavefieldOnce()
{
  if (!model_ || !model_->GetWorld()) return false;
  if (waveBound_) return true;

  auto world = model_->GetWorld();

#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::physics::ModelPtr waveModel = world->ModelByName("nezha_ocean_waves");
#else
  gazebo::physics::ModelPtr waveModel;
  {
    auto models = world->GetModels();
    for (auto const& m : models) {
      if (m->GetName() == "nezha_ocean_waves") { 
        waveModel = m; 
        break; 
      }
    }
  }
#endif

  if (!waveModel) {
#if GAZEBO_MAJOR_VERSION >= 9
    for (auto const& m : world->Models()) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || 
          n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#else
    auto models = world->GetModels();
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
    const auto now = world->SimTime();
    if ((now - lastPrint_).Double() > 1.0) {
      gzerr << "[gazebo_motor_model] Cannot bind wavefield: "
            << "model 'nezha_ocean_waves' not found." << std::endl;
      lastPrint_ = now;
    }
    return false;
  }

  // 查找 link
  gazebo::physics::LinkPtr waveLink;
  auto links = waveModel->GetLinks();
  for (auto const& l : links) {
    if (l->GetName() == "ocean_waves_link") { 
      waveLink = l; 
      break; 
    }
  }
  if (!waveLink && !links.empty()) waveLink = links.front();

  waveModel_ = waveModel;
  waveLink_  = waveLink;
  waveBound_ = (waveModel_ != nullptr);

  if (waveBound_) {
    gzmsg << "[gazebo_motor_model] ✓ Bound wavefield via model '"
          << waveModel_->GetName() << "' link '"
          << (waveLink_ ? waveLink_->GetName() : "(none)") << "'" << std::endl;
  }

  return waveBound_;
}

void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {



  
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;


  if (publish_speed_) {
    motor_velocity_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_speed_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_speed_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_speed_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }


  if (publish_position_) {
    motor_position_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_position_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_position_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_position_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }


  if (publish_force_) {
    motor_force_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
        "~/" + namespace_ + "/" + motor_force_pub_topic_, 1);

    connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
        "~/" + namespace_ + "/" + motor_force_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(
        namespace_ + "/" + motor_force_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    gz_connect_gazebo_to_ros_topic_pub->Publish(
        connect_gazebo_to_ros_topic_msg, true);
  }

  command_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_sub_topic_,
      &GazeboMotorModel::ControlCommandCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(
      namespace_ + "/" + command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);


  wind_speed_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + wind_speed_sub_topic_,
      &GazeboMotorModel::WindSpeedCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(
      namespace_ + "/" + wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);
      
   
 
}

void GazeboMotorModel::ControlCommandCallback(
    GzCommandMotorInputMsgPtr& command_motor_input_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (motor_number_ > command_motor_input_msg->motor_speed_size() - 1) {
    gzerr << "You tried to access index " << motor_number_
          << " of the MotorSpeed message array which is of size "
          << command_motor_input_msg->motor_speed_size();
  }

  if (motor_type_ == MotorType::kVelocity) {
    ref_motor_input_ = std::min(
        static_cast<double>(
            command_motor_input_msg->motor_speed(motor_number_)),
        static_cast<double>(max_rot_velocity_));
  } else if (motor_type_ == MotorType::kPosition) {
    ref_motor_input_ = command_motor_input_msg->motor_speed(motor_number_);
  } else {  
    ref_motor_input_ = std::min(
        static_cast<double>(
            command_motor_input_msg->motor_speed(motor_number_)),
        static_cast<double>(max_force_));
  }
}

void GazeboMotorModel::WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }


  wind_speed_W_.X() = wind_speed_msg->velocity().x();
  wind_speed_W_.Y() = wind_speed_msg->velocity().y();
  wind_speed_W_.Z() = wind_speed_msg->velocity().z();
}

double GazeboMotorModel::NormalizeAngle(double input){

      double wrapped = std::fmod(std::abs(input), 2*M_PI);
      wrapped = std::copysign(wrapped, input);

     if(std::abs(wrapped - 2*M_PI) < 1e-8){
       wrapped = 0;
     }


     if(wrapped < 0){
        wrapped += 2*M_PI;
     }

     return wrapped;
}
void GazeboMotorModel::SpawnRippleOnce() {
  if (ripple_spawned_) return;        
  ripple_spawned_  = true;

  gzmsg << "[gazebo_motor_model] Spawning ripple model at motor " 
        << motor_number_ << std::endl;
  
  sdf::SDFPtr ripple_sdf(new sdf::SDF());
  sdf::init(ripple_sdf);
  
  if (!sdf::readFile(ripple_model_path_, ripple_sdf)) {
    gzerr << "[gazebo_motor_model] Failed to read ripple SDF: " 
          << ripple_model_path_ << std::endl;
    return;
  }
  

  sdf::ElementPtr model_elem = ripple_sdf->Root()->GetElement("model");
  if (!model_elem) {
    gzerr << "[gazebo_motor_model] No model element in ripple SDF" << std::endl;
    return;
  }
  

  std::string unique_name = "surface_ripple_motor_" + std::to_string(motor_number_);
  gzmsg << "[gazebo_motor_model] Spawning ripple model: " << unique_name << std::endl;

  model_elem->GetAttribute("name")->Set(unique_name);


  ignition::math::Pose3d pose(
      link_->WorldPose().Pos().X(),
      link_->WorldPose().Pos().Y(),
      0.0, 0, 0, 0);
  model_elem->GetElement("pose")->Set(pose);


  model_->GetWorld()->InsertModelSDF(*ripple_sdf);


  gazebo::event::Events::ConnectWorldUpdateBegin([this, unique_name] (const common::UpdateInfo&)
  {
    if (!ripple_model_)
      ripple_model_ = model_->GetWorld()->ModelByName(unique_name);


  });
}
void GazeboMotorModel::UpdateRipplePose() {
  if (!ripple_model_) return;

  ignition::math::Pose3d lp = link_->WorldPose();
  ignition::math::Pose3d rp(lp.Pos().X(), lp.Pos().Y(), 0.0, 0, 0, 0);
  ripple_model_->SetWorldPose(rp);
}

void GazeboMotorModel::HideRipple()
{
  if (!ripple_model_) return;


  ignition::math::Pose3d pose = ripple_model_->WorldPose();
  pose.Pos().Z(-1000.0);
  ripple_model_->SetWorldPose(pose);

  ripple_shown_ = false;
}

void GazeboMotorModel::ShowRipple()
{

  ripple_shown_ = true;
}
void GazeboMotorModel::UpdateForcesAndMoments() {
  switch (motor_type_) {
    case (MotorType::kPosition): {
      double err = NormalizeAngle(joint_->Position(0)) - NormalizeAngle(ref_motor_input_);

      if (err > M_PI){
        err -= 2*M_PI;
      }
      if (err < -M_PI){
        err += 2*M_PI;
      }
      if(std::abs(err - M_PI) < 1e-8){
        err = -M_PI;
      }

      double force = pids_.Update(err, sampling_time_);
      joint_->SetForce(0, force);
      break;
    }
    case (MotorType::kForce): {
      joint_->SetForce(0, ref_motor_input_);
      break;
    }
    default:  // MotorType::kVelocity
    {
      motor_rot_vel_ = joint_->GetVelocity(0);
      if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
        gzerr << "Aliasing on motor [" << motor_number_
              << "] might occur. Consider making smaller simulation time "
                 "steps or raising the rotor_velocity_slowdown_sim_ param.\n";
      }
      double real_motor_velocity =
          motor_rot_vel_ * rotor_velocity_slowdown_sim_;

      int real_motor_velocity_sign =
          (real_motor_velocity > 0) - (real_motor_velocity < 0);

      if (!waveBound_) {
        BindWavefieldOnce();
      }
      
      ignition::math::Pose3d pose = link_->WorldPose();
      double x = pose.Pos().X();
      double y = pose.Pos().Y();
      double z = pose.Pos().Z();
      
      double surfaceZ = QuerySurfaceZ(x, y);
      
      double d_over_R;
      double relative_height = z - surfaceZ;

      if (is_underwater_) {
        d_over_R = 1.0;  // 水下直接当作无IGE
      } else {
        double d_over_R_raw = std::clamp(relative_height / blade_radius_, 0.0, 1.0);

        // 对接近表面的值进行抬升，避免0附近导致增益猛增
        double min_dr = std::clamp(surface_blend_window_R_, 0.0, 0.5); // 限制在0~0.5R
        d_over_R = min_dr + (1.0 - min_dr) * d_over_R_raw; // 重映射到[min_dr, 1.0]
      }

      if (water_anim_enabled_ && is_over_water_) {
        if (!ripple_spawned_)  SpawnRippleOnce();
        
        bool should_show = (d_over_R_raw < dR_splash_threshold_);

        if (should_show && !ripple_shown_)       ShowRipple();
        if (!should_show && ripple_shown_)       HideRipple();

        if (ripple_shown_)
          UpdateRipplePose();      
      }

      double rpm_ratio = std::clamp(std::abs(real_motor_velocity) / max_rot_velocity_,
                                    0.0, 1.5);
      kt_gain_ = 1.0;  
      kq_gain_ = 1.0;
      kt_ige_gain_ = 1.0;
      kq_ige_gain_ = 1.0;

      if (is_over_water_ && !is_underwater_) {
        if (use_sr_model_) {
          kt_gain_ = srGain(kt_sr_, d_over_R, rpm_ratio, blade_radius_);
          kq_gain_ = srGain(kq_sr_, d_over_R, rpm_ratio, blade_radius_);
        } else if (table_loaded_) {
          kt_gain_ = std::clamp(bilerp(d_ratio_grid_, rpm_ratio_grid_,
                                       kt_gain_table_, d_over_R, rpm_ratio), 0.9, 1.4);
          kq_gain_ = std::clamp(bilerp(d_ratio_grid_, rpm_ratio_grid_,
                                       kq_gain_table_, d_over_R, rpm_ratio), 0.9, 1.4);
        }
      }

      if (is_over_land_ && !is_underwater_ && ige_loaded_) {
        const double ige_blade_radius = 0.25;
        double ige_d_over_R = std::clamp(z / ige_blade_radius, 0.0, 1.0);
        
        if (use_ige_model_) {
          kt_ige_gain_ = srGain(kt_ige_, ige_d_over_R, rpm_ratio, ige_blade_radius);
          kq_ige_gain_ = srGain(kq_ige_, ige_d_over_R, rpm_ratio, ige_blade_radius);

          kt_gain_ = kt_ige_gain_;
          kq_gain_ = kq_ige_gain_;
        }
      }

      // 水下短路提前：跳过所有 IGE/NWE 增益与水面动画
      if (is_underwater_) {
        kt_gain_ = 1.0;
        kq_gain_ = 1.0;
        kt_ige_gain_ = 1.0;
        kq_ige_gain_ = 1.0;

        // 推力/力矩计算（按常规推进，无IGE）
        double thrust = turning_direction_ * real_motor_velocity_sign *
                        motor_constant_ * 1.0 *
                        real_motor_velocity * real_motor_velocity;

        double motor_torque = turning_direction_ *
                              moment_constant_ * 1.0 *
                              real_motor_velocity * real_motor_velocity;

        // 可选：若你已实现推力/力矩硬限幅，这里也一并使用
        // thrust = std::clamp(thrust, -thrust_max_abs_, thrust_max_abs_);
        // motor_torque = std::clamp(motor_torque, -torque_max_abs_, torque_max_abs_);

        link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

        // 拖矩与滚转惯例处理
        ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
        ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
        ignition::math::Vector3d relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
        ignition::math::Vector3d body_velocity_perpendicular =
            relative_wind_velocity_W -
            (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);

        ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) *
                                 rotor_drag_coefficient_ *
                                 body_velocity_perpendicular;
        link_->AddForce(air_drag);

        physics::Link_V parent_links = link_->GetParentJointsLinks();
        ignition::math::Pose3d pose_difference =
            link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
        ignition::math::Vector3d drag_torque(
            0, 0, -turning_direction_ * real_motor_velocity_sign * motor_torque);
        ignition::math::Vector3d drag_torque_parent_frame =
            pose_difference.Rot().RotateVector(drag_torque);
        parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

        ignition::math::Vector3d rolling_moment =
            -std::abs(real_motor_velocity) *
            rolling_moment_coefficient_ *
            body_velocity_perpendicular;
        parent_links.at(0)->AddTorque(rolling_moment);

        // 关节速度
        double ref_motor_rot_vel =
            rotor_velocity_filter_->updateFilter(ref_motor_input_, sampling_time_);
        joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel /
                                 rotor_velocity_slowdown_sim_);

        // 发布增益（此时全为1.0）
        std_msgs::Float32 gmsg;
        gmsg.data = static_cast<float>(kt_gain_);
        kt_gain_pub_.publish(gmsg);
        gmsg.data = static_cast<float>(kq_gain_);
        kq_gain_pub_.publish(gmsg);
        gmsg.data = static_cast<float>(kt_ige_gain_);
        kt_ige_gain_pub_.publish(gmsg);
        gmsg.data = static_cast<float>(kq_ige_gain_);
        kq_ige_gain_pub_.publish(gmsg);

        return; // 提前返回，跳过水面/地面增益与动画
      }
      auto clamp_gain = [this](double g){
        return std::clamp(g, kt_kq_gain_min_, kt_kq_gain_max_);
      };

      auto rate_limit = [this](double prev, double cur, double dt){
        if (gain_rate_limit_ <= 0.0 || dt <= 0.0) return cur;
        double max_step = gain_rate_limit_ * dt;
        double delta = cur - prev;
        if (delta >  max_step) return prev + max_step;
        if (delta < -max_step) return prev - max_step;
        return cur;
      };

      // 先夹紧
      kt_gain_ = clamp_gain(kt_gain_);
      kq_gain_ = clamp_gain(kq_gain_);

      // 再限制变化速率
      kt_gain_ = rate_limit(prev_kt_gain_, kt_gain_, sampling_time_);
      kq_gain_ = rate_limit(prev_kq_gain_, kq_gain_, sampling_time_);

      prev_kt_gain_ = kt_gain_;
      prev_kq_gain_ = kq_gain_;
      // 安全兜底：若出现 NaN/Inf，回退到 1.0
      if (!std::isfinite(kt_gain_)) kt_gain_ = 1.0;
      if (!std::isfinite(kq_gain_)) kq_gain_ = 1.0;
      if (!std::isfinite(kt_ige_gain_)) kt_ige_gain_ = 1.0;
      if (!std::isfinite(kq_ige_gain_)) kq_ige_gain_ = 1.0;

      double thrust = turning_direction_ * real_motor_velocity_sign *
                      motor_constant_ * kt_gain_ *
                      real_motor_velocity * real_motor_velocity;

      double motor_torque = turning_direction_ *
                            moment_constant_ * kq_gain_ *
                            real_motor_velocity * real_motor_velocity;

      link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

      ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
      ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
      ignition::math::Vector3d relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
      ignition::math::Vector3d body_velocity_perpendicular =
          relative_wind_velocity_W -
          (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) *
                               rotor_drag_coefficient_ *
                               body_velocity_perpendicular;

      link_->AddForce(air_drag);

      physics::Link_V parent_links = link_->GetParentJointsLinks();

      ignition::math::Pose3d pose_difference =
          link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
      ignition::math::Vector3d drag_torque(
          0, 0, -turning_direction_ * real_motor_velocity_sign * motor_torque);

      ignition::math::Vector3d drag_torque_parent_frame =
          pose_difference.Rot().RotateVector(drag_torque);
      parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

      ignition::math::Vector3d rolling_moment;
      rolling_moment = -std::abs(real_motor_velocity) *
                       rolling_moment_coefficient_ *
                       body_velocity_perpendicular;
      parent_links.at(0)->AddTorque(rolling_moment);

      double ref_motor_rot_vel;
      ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(
          ref_motor_input_, sampling_time_);

      joint_->SetVelocity(
          0, turning_direction_ * ref_motor_rot_vel /
                 rotor_velocity_slowdown_sim_);
    }
  }
  
  std_msgs::Float32 gmsg;
  gmsg.data = static_cast<float>(kt_gain_);
  kt_gain_pub_.publish(gmsg);
  gmsg.data = static_cast<float>(kq_gain_);
  kq_gain_pub_.publish(gmsg);

  gmsg.data = static_cast<float>(kt_ige_gain_);
  kt_ige_gain_pub_.publish(gmsg);
  gmsg.data = static_cast<float>(kq_ige_gain_);
  kq_ige_gain_pub_.publish(gmsg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel)


double bilerp(const std::vector<double>& xs,
                     const std::vector<double>& ys,
                     const std::vector<std::vector<double>>& f,
                     double x, double y) {

  auto it_x = std::upper_bound(xs.begin(), xs.end(), x);
  auto it_y = std::upper_bound(ys.begin(), ys.end(), y);
  

  if (it_x == xs.begin()) it_x++;
  if (it_y == ys.begin()) it_y++;
  if (it_x == xs.end()) it_x--;
  if (it_y == ys.end()) it_y--;
  

  size_t i1 = std::distance(xs.begin(), it_x) - 1;
  size_t i2 = std::distance(xs.begin(), it_x);
  size_t j1 = std::distance(ys.begin(), it_y) - 1;
  size_t j2 = std::distance(ys.begin(), it_y);
  

  double x1 = xs[i1], x2 = xs[i2];
  double y1 = ys[j1], y2 = ys[j2];
  double t = (x - x1) / (x2 - x1);
  double u = (y - y1) / (y2 - y1);
  

  double f11 = f[i1][j1], f12 = f[i1][j2];
  double f21 = f[i2][j1], f22 = f[i2][j2];
  
  return (1-t)*(1-u)*f11 + t*(1-u)*f21 + (1-t)*u*f12 + t*u*f22;
}

inline double phys3(const double* th,
                    double h, double p, double r)
{

  auto A1 = th[0] + th[1]*p + th[2]*r;
  auto A2 = th[3] + th[4]*p + th[5]*r;
  auto A3 = th[6] + th[7]*p + th[8]*r;
  auto B1 = th[9] + th[10]*p + th[11]*r;
  auto B2 = th[12]+ th[13]*p + th[14]*r;
  auto n  = th[15];
  return 1.0 + A1*std::exp(-B1*h) + A2*std::exp(-B2*h)
             + A3/std::pow(1.0+h, n);
}

double srGain(const SRModel& m,
              double h, double p, double r)
{
  const double* th = m.theta.data();          
  const double* seg = (h <= m.boundary) ? th : th + 16;  
  double g = phys3(seg, h, p, r);
  return std::clamp(g, 0.9, 1.4);
}
void GazeboMotorModel::TerrainStatusCallback(const std_msgs::String::ConstPtr& msg) {
  terrain_status_ = msg->data;

  is_over_land_ = (terrain_status_ == "land");           
  is_over_water_ = (terrain_status_ == "sea_surface" || 
                    terrain_status_ == "sea_above");
  is_underwater_ = (terrain_status_ == "underwater");   
  

  static int callback_counter = 0;
  if (callback_counter % 50 == 0) {
    gzdbg << "[gazebo_motor_model] Motor " << motor_number_ 
          << " - Terrain status updated: " << terrain_status_ 
          << ", Over land: " << is_over_land_
          << ", Over water: " << is_over_water_
          << ", Underwater: " << is_underwater_ << std::endl;
  }
  callback_counter++;
}


}









