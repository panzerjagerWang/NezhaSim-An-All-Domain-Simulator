

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#include "nezha_getWavefield.hh"
#include "nasv_waveField.hh"
// SYSTEM
#include <stdio.h>
#include <limits>
#include <gazebo/transport/transport.hh>
#include <deque>

// 3RD PARTY
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <std_msgs/String.h>
#include <gazebo/rendering/rendering.hh>
// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"
#include "CommandMotorSpeed.pb.h"
#include "WindSpeed.pb.h"
#include <ros/package.h>
#include <vector>
#include <array>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "Float32.pb.h"
#include <gazebo/transport/transport.hh>
#include <deque>

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}
struct SRModel {
  std::vector<double> theta;
  double boundary;
};
enum class MotorType {
  kVelocity,
  kPosition,
  kForce
};

namespace gz_std_msgs { class Float32; }

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> GzCommandMotorInputMsgPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeed> GzWindSpeedMsgPtr;

static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;

class GazeboMotorModel;

double bilerp(const std::vector<double>& xs,
              const std::vector<double>& ys,
              const std::vector<std::vector<double>>& f,
              double x, double y);

double phys3(const double* th, double h, double p, double r);

double srGain(const SRModel& m,  
              double h, double p, double r);

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:

    GazeboMotorModel(): MotorModel(),                 
        ModelPlugin(),
        pubs_and_subs_created_(false),              
        ripple_model_(nullptr),                      

        command_sub_topic_(mav_msgs::default_topics::COMMAND_ACTUATORS),
        wind_speed_sub_topic_(mav_msgs::default_topics::WIND_SPEED),
        motor_speed_pub_topic_(mav_msgs::default_topics::MOTOR_MEASUREMENT),
        motor_position_pub_topic_(mav_msgs::default_topics::MOTOR_POSITION_MEASUREMENT),
        motor_force_pub_topic_(mav_msgs::default_topics::MOTOR_FORCE_MEASUREMENT),
        namespace_(),                                   
        publish_speed_(true),
        publish_position_(false),
        publish_force_(false),
        motor_number_(0),
        turning_direction_(turning_direction::CW),
        motor_type_(MotorType::kVelocity),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        moment_constant_(kDefaultMomentConstant),
        motor_constant_(kDefaultMotorConstant),
        ref_motor_input_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp),
        node_handle_(nullptr),
        wind_speed_W_(0, 0, 0) {}                       

  virtual ~GazeboMotorModel();
  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  gazebo::physics::ModelPtr waveModel_;
  gazebo::physics::LinkPtr waveLink_;
  bool waveBound_ = false;
  gazebo::common::Time lastPrint_;
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();
  double NormalizeAngle(double input);
physics::ModelPtr ripple_model_;           
bool              ripple_shown_ = false;   

bool             ripple_spawned_   = false;   


  std::string command_sub_topic_;
  std::string wind_speed_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string motor_position_pub_topic_;
  std::string motor_force_pub_topic_;
  std::string namespace_;
  double QuerySurfaceZ(double x, double y) const;
  bool BindWavefieldOnce();
  bool publish_speed_;
  bool publish_position_;
  bool publish_force_;

  int motor_number_;
  int turning_direction_;
  MotorType motor_type_;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_input_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  common::PID pids_;

  gazebo::transport::NodePtr node_handle_;
  gazebo::transport::PublisherPtr motor_velocity_pub_;
  gazebo::transport::PublisherPtr motor_position_pub_;
  gazebo::transport::PublisherPtr motor_force_pub_;
  gazebo::transport::SubscriberPtr command_sub_;
  gazebo::transport::SubscriberPtr wind_speed_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;

  event::ConnectionPtr updateConnection_;
  boost::thread callback_queue_thread_;

  void QueueThread();
void ShowRipple();    
void HideRipple();    
  gz_std_msgs::Float32 turning_velocity_msg_;
  gz_std_msgs::Float32 position_msg_;
  gz_std_msgs::Float32 force_msg_;

  void ControlCommandCallback(GzCommandMotorInputMsgPtr& command_motor_input_msg);
  void WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg);

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  ignition::math::Vector3d wind_speed_W_;

  std::string blade_type_{"thin"};
  double blade_radius_{0.085};

  bool use_sr_model_{false};
  SRModel kt_sr_, kq_sr_;

  std::vector<double> d_ratio_grid_, rpm_ratio_grid_;
  std::vector<std::vector<double>> kt_gain_table_, kq_gain_table_;

  double kt_gain_{1.0}, kq_gain_{1.0};

  bool water_anim_enabled_{true};
  double dR_splash_threshold_{0.25};
  std::string ripple_model_path_;
  std::string ripple_model_name_{"ripple"};

  std::string terrain_status_{"unknown"};
  bool is_over_water_{false};
  bool is_underwater_{false};
  bool is_over_land_{false};

  bool use_ige_model_{false};
  SRModel kt_ige_, kq_ige_;
  double kt_ige_gain_{1.0}, kq_ige_gain_{1.0};
  bool ige_loaded_{false};

  std::string kt_ige_gain_pub_topic_{"kt_ige_gain"};
  std::string kq_ige_gain_pub_topic_{"kq_ige_gain"};
  ros::Publisher kt_ige_gain_pub_;
  ros::Publisher kq_ige_gain_pub_;

  ros::Subscriber terrain_status_sub_;
  void TerrainStatusCallback(const std_msgs::String::ConstPtr& msg);
  bool ripple_emitting_{false};
  void SpawnRippleOnce();
  void UpdateRipplePose();

  bool table_loaded_{false};
  static constexpr double kWaterRho = 997.0;
  static constexpr double kAirRho = 1.225;

  double sampling_time_ = 0.01;
  double prev_sim_time_ = 0.0;
  double motor_rot_vel_ = 0.0;

  bool publish_gain_ = true;
  std::string kt_gain_pub_topic_ = "kt_gain";
  std::string kq_gain_pub_topic_ = "kq_gain";
  ros::Publisher kt_gain_pub_;
  ros::Publisher kq_gain_pub_;
};

} // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

