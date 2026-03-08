#include <cmath>  // 为 std::pow 函数
#include <algorithm> // 为 std::upper_bound, std::copy_n, std::min 函数
#include <array>  // 为 std::array 类型
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "nezha_motorModel.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>   // for std::clamp / std::upper_bound
#include <cmath>
#include <gazebo/transport/transport.hh>   // ★ for Node / Publisher
#include <sdf/sdf.hh>                      // ★ for reading ripple SDF
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
if (_sdf->HasElement("ktGainPubTopic"))
  kt_gain_pub_topic_ = _sdf->Get<std::string>("ktGainPubTopic");
if (_sdf->HasElement("kqGainPubTopic"))
  kq_gain_pub_topic_ = _sdf->Get<std::string>("kqGainPubTopic");

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor "
             "is attached.\n";

  // Get the pointer to the joint.
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

  // Set up joint control PID to control joint.
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

  // Only publish position and force messages if a topic is specified in sdf.
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
  /* -------- 近水动画 SDF 参数 -------- */
getSdfParam<bool>   (_sdf, "waterAnim",        water_anim_enabled_, water_anim_enabled_);
getSdfParam<double> (_sdf, "splashThreshold",  dR_splash_threshold_, dR_splash_threshold_);
getSdfParam<std::string>(_sdf, "rippleModelPath",  ripple_model_path_,  ripple_model_path_);
getSdfParam<std::string>(_sdf, "rippleModelName",  ripple_model_name_,  ripple_model_name_);

/* -------- 创建地面射线，用于判定海 / 陆 -------- */
ground_ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
                model_->GetWorld()->Physics()->CreateShape(
                  "ray", physics::CollisionPtr()));
//---------------------------------- 读增益文件 ---------------------------------
std::string data_dir  = ros::package::getPath("nezha_gazebo") + "/prop_data/";
std::string json_file = data_dir + "NWE_"+blade_type_ + ".json";
ROS_INFO_STREAM("[gazebo_motor_model] Using prop data file: " << json_file);
gzmsg << "[gazebo_motor_model] Using prop data file: " << json_file << std::endl;


std::ifstream jf(json_file);
if (!jf.is_open()) {
  gzerr << "[gazebo_motor_model] cannot open " << json_file << "\n";
} else {
  nlohmann::json j; jf >> j;

  // ── A. SR 公式格式 ─────────────────────────────────────
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
  // ── B. 表格增益格式 (留作兜底) ───────────────────────────
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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  // Create the first order filter.
  rotor_velocity_filter_.reset(
      new FirstOrderFilter<double>(
          time_constant_up_, time_constant_down_, ref_motor_input_));
          
   // ---------- 新增：ROS 话题发布器 ----------
  ros::NodeHandle nh(namespace_);                 // <ns>/kt_gain
  kt_gain_pub_ = nh.advertise<std_msgs::Float32>(kt_gain_pub_topic_, 10);
  kq_gain_pub_ = nh.advertise<std_msgs::Float32>(kq_gain_pub_topic_, 10);
  
  // [NEW] Setup Motor Stop Subscriber
  // Topic will be: /<robotNamespace>/motor_stop_cmd
  // Send "true" to this topic to kill the motor.
  motor_stop_sub_ = nh.subscribe(motor_stop_topic_, 1, 
                                 &GazeboMotorModel::StopCommandCallback, this);

  // [NEW] Setup Link Pose Publisher
  // Topic will be: /<robotNamespace>/motor_link_pose
  link_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(link_pose_pub_topic_, 10);
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  /* === 0. 先用射线检测正下方命中物体名称 === */
  {
    ignition::math::Vector3d pos = link_->WorldPose().Pos();
    ground_ray_->SetPoints(pos, pos - ignition::math::Vector3d(0,0,100));
    std::string entity; double dist;
    ground_ray_->GetIntersection(dist, entity);
    over_sea_ = (entity.find("ocean") != std::string::npos ||
                 entity.find("water") != std::string::npos);
  }
  
  double current_time = _info.simTime.Double();
  if (current_time - last_pose_publish_time_ > 0.05) { // 0.05s = 20Hz
      ignition::math::Pose3d pose = link_->WorldPose();
      
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "world"; // Or "map" depending on your setup
      
      pose_msg.pose.position.x = pose.Pos().X();
      pose_msg.pose.position.y = pose.Pos().Y();
      pose_msg.pose.position.z = pose.Pos().Z();
      
      pose_msg.pose.orientation.w = pose.Rot().W();
      pose_msg.pose.orientation.x = pose.Rot().X();
      pose_msg.pose.orientation.y = pose.Rot().Y();
      pose_msg.pose.orientation.z = pose.Rot().Z();
      
      link_pose_pub_.publish(pose_msg);
      last_pose_publish_time_ = current_time;
  }
  
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

  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  //  ACTUAL MOTOR SPEED MSG SETUP (GAZEBO->ROS)  //
  // ============================================ //
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

  // =============================================== //
  //  ACTUAL MOTOR POSITION MSG SETUP (GAZEBO->ROS)  //
  // =============================================== //

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

  // ============================================ //
  //  ACTUAL MOTOR FORCE MSG SETUP (GAZEBO->ROS)  //
  // ============================================ //

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

  // ============================================ //
  // = CONTROL COMMAND MSG SETUP (ROS->GAZEBO) = //
  // ============================================ //

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

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  /// TODO(gbmhunter): Do we need this? There is a separate Gazebo wind plugin.
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
  } else {  // if (motor_type_ == MotorType::kForce) {
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

  // TODO(burrimi): Transform velocity to world frame if frame_id is set to
  // something else.
  wind_speed_W_.X() = wind_speed_msg->velocity().x();
  wind_speed_W_.Y() = wind_speed_msg->velocity().y();
  wind_speed_W_.Z() = wind_speed_msg->velocity().z();
}

double GazeboMotorModel::NormalizeAngle(double input){
      // Constrain magnitude to be max 2*M_PI.
      double wrapped = std::fmod(std::abs(input), 2*M_PI);
      wrapped = std::copysign(wrapped, input);

     // Constrain result to be element of [0, 2*pi).
     // Set angle to zero if sufficiently close to 2*pi.
     if(std::abs(wrapped - 2*M_PI) < 1e-8){
       wrapped = 0;
     }

     // Ensure angle is positive.
     if(wrapped < 0){
        wrapped += 2*M_PI;
     }

     return wrapped;
}
void GazeboMotorModel::StopCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
  motor_stopped_ = msg->data;
  if (motor_stopped_) {
     // Optional: Print a warning when stop is triggered
     // gzdbg << "Motor " << motor_number_ << " EMERGENCY STOP triggered." << std::endl;
  }
}
void GazeboMotorModel::UpdateForcesAndMoments() {
  // [NEW] Check for Emergency Stop
  if (motor_stopped_) {
      ref_motor_input_ = 0.0;
      // If using PID (Position control), you might want to reset errors here too
      // pids_.Reset(); 
  }
  switch (motor_type_) {
    case (MotorType::kPosition): {
      double err = NormalizeAngle(joint_->Position(0)) - NormalizeAngle(ref_motor_input_);

      // Angles are element of [0..2pi).
      // Constrain difference of angles to be in [-pi..pi).
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
      // Get the direction of the rotor rotation.
      int real_motor_velocity_sign =
          (real_motor_velocity > 0) - (real_motor_velocity < 0);
      // Assuming symmetric propellers (or rotors) for the thrust calculation.
// ---------- 计算 d/R 和 rpm 比 ----------
 ignition::math::Pose3d pose = link_->WorldPose();
 double z         = pose.Pos().Z();          // 以水面 z=0 为基准，向上为 +
 bool   underwater = (z <= 0.0);

 double d_over_R;
 if (underwater) {
   d_over_R = 1.0;                           // 占位，无实际意义
 } else {
   d_over_R = std::clamp(z / blade_radius_, 0.0, 1.0);
 }
/* === 近水动画：海面才触发 === */
if (water_anim_enabled_ && over_sea_) {
  if (d_over_R < dR_splash_threshold_) {
    if (!ripple_model_ && !waiting_for_spawn_)   SpawnRippleOnce();
  } else {                                       // 离水远或水下
    if (ripple_model_) model_->GetWorld()->RemoveModel(ripple_model_);
    ripple_model_.reset();  waiting_for_spawn_ = false;
  }
  if (ripple_model_)    UpdateRipplePose();
}
double rpm_ratio = std::clamp(std::abs(real_motor_velocity) / max_rot_velocity_,
                              0.0, 1.4);

// ---------- 取增益 (写入成员) ----------
kt_gain_ = 1.0;  kq_gain_ = 1.0;
if (use_sr_model_) {
kt_gain_ = srGain(kt_sr_, d_over_R, rpm_ratio, blade_radius_);
kq_gain_ = srGain(kq_sr_, d_over_R, rpm_ratio, blade_radius_);

} else if (table_loaded_) {
  kt_gain_ = std::clamp(bilerp(d_ratio_grid_, rpm_ratio_grid_,
                               kt_gain_table_, d_over_R, rpm_ratio), 0.9, 1.4);
  kq_gain_ = std::clamp(bilerp(d_ratio_grid_, rpm_ratio_grid_,
                               kq_gain_table_, d_over_R, rpm_ratio), 0.9, 1.4);
}
 // -------- 水下直接关闭近水修正 --------
  if (underwater) {
    // 现修改为 0.3
    kt_gain_ = 0.7; 
    kq_gain_ = 0.7;
  }
// ---------- 推力 / 扭矩 ----------
double thrust = turning_direction_ * real_motor_velocity_sign *
                motor_constant_ * kt_gain_ *
                 real_motor_velocity * real_motor_velocity;

double motor_torque = turning_direction_ *
                      moment_constant_ * kq_gain_ *
                       real_motor_velocity * real_motor_velocity;

// ---------- 施加到机体 ----------
link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

// 下面原有 air_drag / rolling_moment / parent_links 加力代码 **全部保留**，
// 只把之前用到 old 'thrust' 变量的地方替换成新的 `thrust`.
      // Forces from Philppe Martin's and Erwan Salaün's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
      ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
      ignition::math::Vector3d relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
      ignition::math::Vector3d body_velocity_perpendicular =
          relative_wind_velocity_W -
          (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) *
                               rotor_drag_coefficient_ *
                               body_velocity_perpendicular;

      // Apply air_drag to link.
      link_->AddForce(air_drag);
      // Moments get the parent link, such that the resulting torques can be
      // applied.
      physics::Link_V parent_links = link_->GetParentJointsLinks();
      // The tansformation from the parent_link to the link_.
      ignition::math::Pose3d pose_difference =
          link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
      ignition::math::Vector3d drag_torque(
          0, 0, -turning_direction_ * thrust * moment_constant_);
      // Transforming the drag torque into the parent frame to handle
      // arbitrary rotor orientations.
      ignition::math::Vector3d drag_torque_parent_frame =
          pose_difference.Rot().RotateVector(drag_torque);
      parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

      ignition::math::Vector3d rolling_moment;
      // - \omega * \mu_1 * V_A^{\perp}
      rolling_moment = -std::abs(real_motor_velocity) *
                       rolling_moment_coefficient_ *
                       body_velocity_perpendicular;
      parent_links.at(0)->AddTorque(rolling_moment);
      // Apply the filter on the motor's velocity.
      double ref_motor_rot_vel;
      ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(
          ref_motor_input_, sampling_time_);

      // Make sure max force is set, as it may be reset to 0 by a world reset any
      // time. (This cannot be done during Reset() because the change will be undone
      // by the Joint's reset function afterwards.)
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

}
GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
// 添加在 gazebo 命名空间内，在最后的 } 闭合花括号之前

double bilerp(const std::vector<double>& xs,
                     const std::vector<double>& ys,
                     const std::vector<std::vector<double>>& f,
                     double x, double y) {
  // 查找边界
  auto it_x = std::upper_bound(xs.begin(), xs.end(), x);
  auto it_y = std::upper_bound(ys.begin(), ys.end(), y);
  
  // 边界处理
  if (it_x == xs.begin()) it_x++;
  if (it_y == ys.begin()) it_y++;
  if (it_x == xs.end()) it_x--;
  if (it_y == ys.end()) it_y--;
  
  // 计算索引
  size_t i1 = std::distance(xs.begin(), it_x) - 1;
  size_t i2 = std::distance(xs.begin(), it_x);
  size_t j1 = std::distance(ys.begin(), it_y) - 1;
  size_t j2 = std::distance(ys.begin(), it_y);
  
  // 权重计算
  double x1 = xs[i1], x2 = xs[i2];
  double y1 = ys[j1], y2 = ys[j2];
  double t = (x - x1) / (x2 - x1);
  double u = (y - y1) / (y2 - y1);
  
  // 双线性插值
  double f11 = f[i1][j1], f12 = f[i1][j2];
  double f21 = f[i2][j1], f22 = f[i2][j2];
  
  return (1-t)*(1-u)*f11 + t*(1-u)*f21 + (1-t)*u*f12 + t*u*f22;
}

inline double phys3(const double* th,
                    double h, double p, double r)
{
  // theta 布局：A1_0 A1_p A1_r  A2_0 A2_p A2_r  A3_0 A3_p A3_r
  //             B1_0 B1_p B1_r  B2_0 B2_p B2_r  n
  auto A1 = th[0] + th[1]*p + th[2]*r;
  auto A2 = th[3] + th[4]*p + th[5]*r;
  auto A3 = th[6] + th[7]*p + th[8]*r;
  auto B1 = th[9] + th[10]*p + th[11]*r;
  auto B2 = th[12]+ th[13]*p + th[14]*r;
  auto n  = th[15];
  return 1.0 + A1*std::exp(-B1*h) + A2*std::exp(-B2*h)
             + A3/std::pow(1.0+h, n);
}

double srGain(const GazeboMotorModel::SRModel& m,
              double h, double p, double r)
{
  const double* th = m.theta.data();           // θ 共 32
  const double* seg = (h <= m.boundary) ? th : th + 16;  // 选左/右 16
  double g = phys3(seg, h, p, r);
  return std::clamp(g, 0.9, 1.4);
}
void GazeboMotorModel::SpawnRippleOnce() {
  if (ripple_model_path_.empty()) {
    gzwarn << "[GazeboMotorModel] ripple_model_path is empty, skipping spawn.\n";
    return;
  }
  
  waiting_for_spawn_ = true;
  
  // 读取 SDF 文件
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  
  if (!sdf::readFile(ripple_model_path_, sdf)) {
    gzerr << "[GazeboMotorModel] Failed to read ripple model SDF: " 
          << ripple_model_path_ << "\n";
    waiting_for_spawn_ = false;
    return;
  }
  
  // 获取模型元素
  sdf::ElementPtr model_elem = sdf->Root()->GetElement("model");
  if (!model_elem) {
    gzerr << "[GazeboMotorModel] No model element in ripple SDF\n";
    waiting_for_spawn_ = false;
    return;
  }
  
  // 设置模型名称
  model_elem->GetAttribute("name")->SetFromString(ripple_model_name_);
  
  // 设置初始位置
  ignition::math::Pose3d uav_pose = link_->WorldPose();
  ignition::math::Pose3d ripple_pose(
    uav_pose.Pos().X(),
    uav_pose.Pos().Y(),
    0.0,  // 水面高度
    0, 0, 0
  );
  
  sdf::ElementPtr pose_elem = model_elem->GetElement("pose");
  if (pose_elem) {
    pose_elem->Set(ripple_pose);
  }
  
  // 插入模型到世界
  model_->GetWorld()->InsertModelSDF(*sdf);
  
  gzmsg << "[GazeboMotorModel] Spawned ripple model: " 
        << ripple_model_name_ << "\n";
  
  // 等待模型加载(在下一帧更新中获取指针)
}

void GazeboMotorModel::UpdateRipplePose() {
  // 如果还在等待生成,尝试获取模型指针
  if (waiting_for_spawn_ && !ripple_model_) {
    ripple_model_ = model_->GetWorld()->ModelByName(ripple_model_name_);
    if (ripple_model_) {
      waiting_for_spawn_ = false;
      gzmsg << "[GazeboMotorModel] Ripple model loaded successfully\n";
    }
    return;
  }
  
  // 如果模型存在,更新其位置
  if (ripple_model_) {
    ignition::math::Pose3d uav_pose = link_->WorldPose();
    ignition::math::Pose3d ripple_pose(
      uav_pose.Pos().X(),
      uav_pose.Pos().Y(),
      0.0,  // 保持在水面
      0, 0, 0
    );
    ripple_model_->SetWorldPose(ripple_pose);
  }
}

}




