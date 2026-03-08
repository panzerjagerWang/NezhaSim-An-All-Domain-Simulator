

#ifndef TERRAIN_DETECTOR_PLUGIN_H
#define TERRAIN_DETECTOR_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <regex>
#include <vector>
namespace gazebo {

enum class TerrainType {
  UNKNOWN,
  LAND,
  SEA_SURFACE,
  UNDERWATER
};

class TerrainDetectorPlugin : public ModelPlugin {
public:
  TerrainDetectorPlugin();
  virtual ~TerrainDetectorPlugin();

  // Load function required by Gazebo
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Called by the world update start event
  virtual void OnUpdate(const common::UpdateInfo& _info);

  // Public methods for other plugins to query terrain status
  TerrainType GetTerrainType() const;
  std::string GetTerrainName() const;
  bool IsOverSea() const;
  bool IsUnderwater() const;
  bool IsOverLand() const;

private:
  // 波浪场相关
  gazebo::physics::ModelPtr wave_model_;
  gazebo::physics::LinkPtr wave_link_;
  bool wave_bound_;
  gazebo::common::Time last_print_time_;
  
  // 新增方法
  bool BindWavefieldOnce();
  double QuerySurfaceZ(double x, double y) const;
  double water_surface_level_;
  double surface_tolerance_;
  std::vector<std::string> water_keywords_;
  std::regex water_regex_;
  void InitializeWaterKeywords();
  void BuildWaterRegex(); 
  bool IsWaterEntity(const std::string& entity_name) const;
  // Detect current terrain type
  void DetectTerrain();
  
  // Publish terrain status to ROS topic
  void PublishTerrainStatus();

  // Model and world pointers
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  
  // Ray for ground detection
  physics::RayShapePtr ground_ray_;
  
  // Update connection
  event::ConnectionPtr update_connection_;
  
  // ROS related
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher terrain_pub_;
  
  // Parameters
  std::string namespace_;
  std::string link_name_;
  std::string terrain_pub_topic_;
  double update_rate_;
  double update_period_;
  
  // State
  TerrainType terrain_type_;
  std::string terrain_name_;
  common::Time last_update_time_;
};

} // namespace gazebo

#endif // TERRAIN_DETECTOR_PLUGIN_H

