#ifndef ROBOT_STATE_PLUGIN_HH
#define ROBOT_STATE_PLUGIN_HH
#include "nezha_getWavefield.hh"
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/vector3d.pb.h>  

#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <uuv_gazebo_ros_plugins_msgs/SetFloat.h>
#include <uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiency.h>
#include <uuv_world_ros_plugins_msgs/SetCurrentVelocity.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <asv_wave_sim_gazebo_plugins/MeshTools.hh>
#include <asv_wave_sim_gazebo_plugins/Wavefield.hh>
#include <asv_wave_sim_gazebo_plugins/WavefieldEntity.hh>
#include <asv_wave_sim_gazebo_plugins/Grid.hh>
#include "asv_wave_sim_gazebo_plugins/Convert.hh"     

#include <memory>
#include <string>
#include <vector>

namespace gazebo
{


using MeshPtr = std::shared_ptr<::asv::Mesh>;
using MeshVector = std::vector<MeshPtr>;

class RobotStatePlugin : public ModelPlugin
{

  struct LinkHydroData
  {
    physics::LinkPtr link;                            
    std::vector<std::shared_ptr<::asv::Mesh>> initMeshes;   
    std::vector<std::shared_ptr<::asv::Mesh>> worldMeshes;  
    double immersion = 0.0;                          
  };

public:

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  void OnUpdate();

private:
  gazebo::physics::ModelPtr waveModel_;
  gazebo::physics::LinkPtr waveLink_;
  std::shared_ptr<asv::WavefieldModelPlugin> wavefield_;
  bool waveBound_ = false;
  gazebo::common::Time lastPrint_;
  std::vector<std::string> wfCandidates_;
  // 辅助方法
  bool BindWavefieldOnce();
  double QuerySurfaceZ(double x, double y) const;
  void InitSDFParameters(sdf::ElementPtr _sdf);
  void InitServiceClients();
void OnSurfaceZ(const boost::shared_ptr<const gazebo::msgs::Vector3d> &_msg);


bool hasPhase_ = false;
std::string lastPhase_;   


  bool CheckWaterSurfaceState();             
  void HandleStateTransition(bool isAboveWater); 

  template <typename ServiceType>
  ros::ServiceClient CreateClient(const std::string& serviceName);

  template <typename ServiceType>
  void AsyncServiceCall(ros::ServiceClient& client,
                        const typename ServiceType::Request& req);

  void SetUnderwaterParams(double scale);
  void SetThrusterEfficiency(double eff);
  void SetCurrentVelocity(double vel, double horz, double vert);

double ComputeImmersion(const std::vector<std::shared_ptr<::asv::Mesh>>& meshes, double waterZ);

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;
  gazebo::transport::NodePtr       gzNode_;      
  gazebo::transport::SubscriberPtr surfaceSub_;  

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::ServiceClient fluid_density_client_, volume_scaling_client_;
  ros::ServiceClient added_mass_scaling_client_, damping_scaling_client_;
  ros::ServiceClient thruster_clients_[3];
  ros::ServiceClient set_current_velocity_client_;

  std::vector<LinkHydroData> links_;

  double default_fluid_density_, default_volume_scaling_;
  double default_added_mass_scaling_, default_damping_scaling_;
  double water_surface_height_, default_current_velocity_;
  double updateInterval_;
  bool   lastState_ = false;            
  common::Time lastUpdateTime_;

  double surfaceZ_   {0.0};
  bool   hasSurface_ {false};

}; 


template <typename ServiceType>
ros::ServiceClient RobotStatePlugin::CreateClient(const std::string& serviceName)
{
  auto client = nh_->serviceClient<ServiceType>(serviceName);
  if (!client.waitForExistence(ros::Duration(2.0)))
    ROS_WARN_STREAM("Service " << serviceName << " not available");
  return client;
}

template <typename ServiceType>
void RobotStatePlugin::AsyncServiceCall(ros::ServiceClient& client,
                                        const typename ServiceType::Request& req)
{
  ServiceType srv;
  srv.request = req;
  if (client.call(srv))
    ROS_INFO_STREAM("Service call succeeded: " << client.getService());
  else
    ROS_ERROR_STREAM("Service call failed: " << client.getService());
}


void ApplyPoseToMesh(const ignition::math::Pose3d& pose,
                     const std::shared_ptr<::asv::Mesh>& src,
                     std::shared_ptr<::asv::Mesh>& dst);

} // namespace gazebo

#endif  // ROBOT_STATE_PLUGIN_HH


