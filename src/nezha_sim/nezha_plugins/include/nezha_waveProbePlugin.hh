#pragma once

#include <memory>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS
#include <ros/ros.h>
#include <nezha_plugins/GetPhaseSample.h>

#include "nasv_waveField.hh"
#include "nezha_getWavefield.hh"

namespace gazebo { namespace event { class Connection; } }

namespace asv {

// Forward declare the wavefield entity from your wave sim package.
class WavefieldEntity;

class WaveProbePlugin : public gazebo::ModelPlugin
{
public:
  WaveProbePlugin();
  ~WaveProbePlugin() override;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:

  struct Data {
    // Gazebo
    gazebo::physics::ModelPtr  model;
    gazebo::physics::WorldPtr  world;

    boost::shared_ptr<asv::WavefieldEntity> wavefield;

    std::vector<std::string> wfCandidates;
    std::string boundName;

    gazebo::common::Time lastPrint{gazebo::common::Time::Zero};
    bool firstBindTried = false;
    std::string robotNamespace;
    gazebo::physics::ModelPtr waveModel;
    gazebo::physics::LinkPtr  waveLink;
    bool waveBound = false;

    // ROS
    std::unique_ptr<ros::NodeHandle> nh;
    ros::ServiceServer              srvPhase;

    // Params
    double L{1.0};
    double zTopOffset{NAN};

    // Trend cache
    double               rawZol_last{0.0};
    gazebo::common::Time t_last;
    std::string          lastPhase{"ABOVE"};
    gazebo::event::ConnectionPtr updateConn;
  };


  std::unique_ptr<Data> data;
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

    
    bool OnGetPhaseSample(
        nezha_plugins::GetPhaseSample::Request &req,
        nezha_plugins::GetPhaseSample::Response &res);
  bool BindWavefieldOnce();

  // Service handler
  bool SrvGetPhaseSample(
    nezha_plugins::GetPhaseSample::Request& req,
    nezha_plugins::GetPhaseSample::Response& res);

  // Helper queries
  double QuerySurfaceZ(double x, double y) const;
  std::string PhaseFromZOverL(double zOverL, double rawZol, double dzdt) const;

}; // end class WaveProbePlugin

} // namespace asv


