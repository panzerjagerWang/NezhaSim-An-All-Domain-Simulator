// TransmediaDragPlugin.hh
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>

#include <array>
#include <memory>
#include <string>
#include <nezha_plugins/GetPhaseSample.h>

namespace gazebo {
namespace mini_expr { struct AST; }

class TransMediaDragPlugin : public ModelPlugin {
public:
  TransMediaDragPlugin();
  ~TransMediaDragPlugin() override;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  enum class Phase { None, Entry, Exit, Above };

  struct DragModel {
  enum class Kind { None, Poly3, Quadratic, PiecewiseCTR };

  Kind kind = Kind::None;

std::array<double,22> theta{};
double boundary_on_z_over_l{0.0};
bool   has_theta{false};
bool   use_residual{true};

std::shared_ptr<mini_expr::AST> residual_ast; 

double Eval(double v_abs, double z_over_l) const;


    std::array<double, 10> coeffs{};

    // quadratic: 0.5*rho*Cd*A*|v|^exponent
    double rho{1.225};
    double Cd{1.0};
    double A{0.01};
    double exponent{2.0};


  };


  static bool LoadModelFromJson(const std::string& path,
                                DragModel& out,
                                std::string* err = nullptr);

private:

std::string surfaceZService;
ros::ServiceClient surfaceClient;
bool wasSubmerged{false};
bool haveSurfaceZ{false};
double lastSurfaceZ{0.0};


  void OnUpdate();
bool QueryPhaseSample(double x, double y, double& outSurfaceZ, double& outZOverL, Phase& outPhase);

  // Gazebo
  physics::ModelPtr model;
  physics::LinkPtr  baseLink;
  event::ConnectionPtr updateConnection;
  gazebo::common::Time  lastUpdateTime;

  ignition::math::Vector3d lastForce{0,0,0};


double characteristicLen{1.0};  
double ctrScale{1.0};          
double zTopOffset{0.0};     

  double      waterSurfaceZ{0.0};
  double      updateRate{200.0};
  bool        alwaysPublish{true};
  std::string robotNamespace;

  // ROS
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Publisher  dragPub;
  ros::Subscriber phaseSub;
  std::string     phaseTopic{"transmedia_phase"};


  Phase currentPhase{Phase::None};
  DragModel entryModel, exitModel, aboveModel;


  std::string entryJson, exitJson, aboveJson;
};

} // namespace gazebo

