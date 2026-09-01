//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
#ifndef __NEZHA_UNDERWATER_OBJECT_PLUGIN_HH__
#define __NEZHA_UNDERWATER_OBJECT_PLUGIN_HH__

#include <map>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>

// ✅ ROS Headers
#include <ros/ros.h>
#include <uuv_gazebo_ros_plugins_msgs/GetFloat.h>
#include <uuv_gazebo_ros_plugins_msgs/SetFloat.h>

#include "nezha_underwaterHydrodynamicsPlugin.hh"
#include <uuv_gazebo_plugins/Def.hh>

namespace gazebo
{

/// \brief Gazebo model plugin class for underwater objects
// !!! CHANGED CLASS NAME !!!
class NezhaUnderwaterObjectPlugin : public gazebo::ModelPlugin
{
  /// \brief Constructor
  // !!! CHANGED CONSTRUCTOR !!!
  public: NezhaUnderwaterObjectPlugin();

  /// \brief Destructor
  // !!! CHANGED DESTRUCTOR !!!
  public: virtual ~NezhaUnderwaterObjectPlugin();

  // Documentation inherited.
  public: virtual void Load(gazebo::physics::ModelPtr _model,
                          sdf::ElementPtr _sdf);

  // Documentation inherited.
  public: virtual void Init();

  /// \brief Update the simulation state.
  /// \param[in] _info Information used in the update event.
  public: virtual void Update(const gazebo::common::UpdateInfo &_info);

  /// \brief Connects the update event callback
  protected: virtual void Connect();

  /// \brief Reads flow velocity topic
  protected: void UpdateFlowVelocity(ConstVector3dPtr &_msg);

  /// \brief Publish current velocity marker
  protected: virtual void PublishCurrentVelocityMarker();

  /// \brief Publishes the state of the vehicle (is submerged)
  protected: virtual void PublishIsSubmerged();

  /// \brief Publish restoring force
  /// \param[in] _link Pointer to the link where the force information will
  /// be extracted from
  protected: virtual void PublishRestoringForce(
    gazebo::physics::LinkPtr _link);

  /// \brief Publish hydrodynamic wrenches
  /// \param[in] _link Pointer to the link where the force information will
  /// be extracted from
  protected: virtual void PublishHydrodynamicWrenches(
    gazebo::physics::LinkPtr _link);

  /// \brief Returns the wrench message for debugging topics
  /// \param[in] _force Force vector
  /// \param[in] _torque Torque vector
  /// \param[in] _output Stamped wrench message to be updated
  protected: virtual void GenWrenchMsg(
    ignition::math::Vector3d _force, ignition::math::Vector3d _torque,
    gazebo::msgs::WrenchStamped &_output);

  /// \brief Sets the topics used for publishing the intermediate data during
  /// the simulation
  /// \param[in] _link Pointer to the link
  /// \param[in] _hydro Pointer to the hydrodynamic model
  protected: virtual void InitDebug(gazebo::physics::LinkPtr _link, nezha::HydrodynamicModelPtr _hydro);
  // ✅ ROS Service Callbacks
  protected: bool GetFluidDensity(
      uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res);

  protected: bool SetFluidDensity(
      uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res);

  protected: bool GetVolumeScaling(
      uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res);

  protected: bool SetVolumeScaling(
      uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res);

  protected: bool GetAddedMassScaling(
      uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res);

  protected: bool SetAddedMassScaling(
      uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res);

  protected: bool GetDampingScaling(
      uuv_gazebo_ros_plugins_msgs::GetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::GetFloat::Response& res);

  protected: bool SetDampingScaling(
      uuv_gazebo_ros_plugins_msgs::SetFloat::Request& req,
      uuv_gazebo_ros_plugins_msgs::SetFloat::Response& res);

  /// \brief Pairs of links & corresponding hydrodynamic models
  protected: std::map<nezha::physics::LinkPtr,
                      nezha::HydrodynamicModelPtr> models;

  /// \brief Flow velocity vector read from topic
  protected: ignition::math::Vector3d flowVelocity;

  /// \brief Update event
  protected: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to the world plugin
  protected: gazebo::physics::WorldPtr world;

  /// \brief Pointer to the model structure
  protected: gazebo::physics::ModelPtr model;

  /// \brief Gazebo node
  protected: gazebo::transport::NodePtr node;

  /// \brief Name of vehicle's base_link
  protected: std::string baseLinkName;

  /// \brief Subcriber to flow message
  protected: gazebo::transport::SubscriberPtr flowSubscriber;

  /// \brief Flag to use the global current velocity or the individually
  /// assigned current velocity
  protected: bool useGlobalCurrent;

  /// \brief Publishers of hydrodynamic and hydrostatic forces and torques in
  /// the case the debug flag is on
  protected: std::map<std::string, gazebo::transport::PublisherPtr> hydroPub;

  // ✅ ROS related members
  protected: std::unique_ptr<ros::NodeHandle> rosNode;
  
  protected: std::map<std::string, ros::ServiceServer> services;
};
}
#endif  // __NEZHA_UNDERWATER_OBJECT_PLUGIN_HH__
