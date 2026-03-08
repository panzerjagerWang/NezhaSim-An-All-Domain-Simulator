#include "nezha_terrainDetectorPlugin.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <sstream>

// 添加波浪场相关头文件
#include "nasv_waveField.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "nezha_getWavefield.hh"

namespace gazebo {

TerrainDetectorPlugin::TerrainDetectorPlugin()
  : ModelPlugin()
  , water_surface_level_(0.0)
  , surface_tolerance_(0.05)
  , water_keywords_()
  , water_regex_()
  , model_(nullptr)
  , world_(nullptr)
  , link_(nullptr)
  , ground_ray_(nullptr)
  , update_connection_(nullptr)
  , ros_node_(nullptr)
  , terrain_pub_()
  , namespace_("")
  , link_name_("base_link")
  , terrain_pub_topic_("terrain_status")
  , update_rate_(50.0)
  , update_period_(1.0/50.0)
  , terrain_type_(TerrainType::UNKNOWN)
  , terrain_name_("unknown")
  , last_update_time_(0)
  , wave_model_(nullptr)
  , wave_link_(nullptr)
  , wave_bound_(false)
  , last_print_time_(0)
{
  InitializeWaterKeywords();
  BuildWaterRegex();
}

void TerrainDetectorPlugin::InitializeWaterKeywords() {
  water_keywords_ = {
    "ocean", "sea", "marine", "maritime", "nautical", "pelagic",
    "water", "aqua", "hydro", "liquid", "fluid",
    "wave", "waves", "ocean_wave", "ocean_waves", "sea_wave", "sea_waves",
    "lake", "pond", "pool", "lagoon", "reservoir", "loch",
    "river", "stream", "creek", "brook", "canal", "channel",
    "swamp", "marsh", "wetland", "bog", "fen", "mire",
    "bay", "gulf", "strait", "harbor", "harbour", "port",
    "beach", "shore", "coast", "waterfront", "seaside",
    "spring", "fountain", "waterfall", "cascade",
    "underwater", "submerged", "submarine", "subsea", "subaquatic"
  };
  
  BuildWaterRegex();
}

void TerrainDetectorPlugin::BuildWaterRegex() {
  std::string pattern = "(";
  for (size_t i = 0; i < water_keywords_.size(); ++i) {
    pattern += water_keywords_[i];
    if (i < water_keywords_.size() - 1) {
      pattern += "|";
    }
  }
  pattern += ")";
  
  water_regex_ = std::regex(pattern, std::regex_constants::icase);
}

bool TerrainDetectorPlugin::IsWaterEntity(const std::string& entity_name) const {
  return std::regex_search(entity_name, water_regex_);
}

TerrainDetectorPlugin::~TerrainDetectorPlugin() {
  if (update_connection_) {
    update_connection_.reset();
  }
  if (ros_node_) {
    ros_node_->shutdown();
  }
}

void TerrainDetectorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  gzmsg << "TerrainDetectorPlugin::Load() called." << std::endl;

  model_ = _model;
  world_ = model_->GetWorld();

  // 保留作为默认/备用水面高度
  if (_sdf->HasElement("waterSurfaceLevel")) {
    water_surface_level_ = _sdf->GetElement("waterSurfaceLevel")->Get<double>();
  } else {
    water_surface_level_ = 0.0;
  }

  if (_sdf->HasElement("surfaceTolerance")) {
    surface_tolerance_ = _sdf->GetElement("surfaceTolerance")->Get<double>();
  } else {
    surface_tolerance_ = 0.5;  // 修改默认值为更合理的0.5米
  }

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[terrain_detector] Please specify a linkName." << std::endl;
    return;
  }

  if (_sdf->HasElement("updateRate")) {
    update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  }

  if (_sdf->HasElement("terrainPubTopic")) {
    terrain_pub_topic_ = _sdf->GetElement("terrainPubTopic")->Get<std::string>();
  }

  // 自定义水关键词
  if (_sdf->HasElement("waterKeywords")) {
    std::string keywords_str = _sdf->GetElement("waterKeywords")->Get<std::string>();
    std::stringstream ss(keywords_str);
    std::string keyword;
    while (std::getline(ss, keyword, ',')) {
      keyword.erase(0, keyword.find_first_not_of(" \t"));
      keyword.erase(keyword.find_last_not_of(" \t") + 1);
      if (!keyword.empty()) {
        water_keywords_.push_back(keyword);
        gzmsg << "[terrain_detector] Added custom water keyword: " << keyword << std::endl;
      }
    }
    BuildWaterRegex();
  }

  link_ = model_->GetLink(link_name_);
  if (!link_) {
    gzerr << "[terrain_detector] Couldn't find specified link \"" << link_name_ << "\"." << std::endl;
    return;
  }

  ground_ray_ = boost::dynamic_pointer_cast<physics::RayShape>(
      world_->Physics()->CreateShape("ray", physics::CollisionPtr()));

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "terrain_detector_plugin", ros::init_options::NoSigintHandler);
  }

  std::string node_namespace = namespace_;
  if (!node_namespace.empty() && node_namespace[0] != '/') {
    node_namespace = "/" + node_namespace;
  }
  ros_node_.reset(new ros::NodeHandle(node_namespace));

  terrain_pub_ = ros_node_->advertise<std_msgs::String>(terrain_pub_topic_, 10);

  update_period_ = 1.0 / update_rate_;
  last_update_time_ = world_->SimTime();

  // 尝试绑定波浪场
  BindWavefieldOnce();

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TerrainDetectorPlugin::OnUpdate, this, _1));

  gzmsg << "[terrain_detector] Plugin loaded successfully. Publishing to: " 
        << node_namespace << "/" << terrain_pub_topic_ << std::endl;
  gzmsg << "[terrain_detector] Monitoring link: " << link_name_ << std::endl;
  gzmsg << "[terrain_detector] Update rate: " << update_rate_ << " Hz" << std::endl;
  gzmsg << "[terrain_detector] Surface tolerance: " << surface_tolerance_ << " m" << std::endl;
}

bool TerrainDetectorPlugin::BindWavefieldOnce() {
  if (!world_) return false;
  if (wave_bound_) return true;

#if GAZEBO_MAJOR_VERSION >= 9
  physics::ModelPtr waveModel = world_->ModelByName("nezha_ocean_waves");
#else
  physics::ModelPtr waveModel;
  {
    auto models = world_->GetModels();
    for (auto const& m : models) {
      if (m->GetName() == "nezha_ocean_waves") { 
        waveModel = m; 
        break; 
      }
    }
  }
#endif

  // 如果找不到，尝试模糊匹配
  if (!waveModel) {
#if GAZEBO_MAJOR_VERSION >= 9
    for (auto const& m : world_->Models()) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#else
    auto models = world_->GetModels();
    for (auto const& m : models) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#endif
  }

  if (!waveModel) {
    const auto now = world_->SimTime();
    if ((now - last_print_time_).Double() > 2.0) {
      gzwarn << "[terrain_detector] Wave model 'nezha_ocean_waves' not found. Using static water level." << std::endl;
      last_print_time_ = now;
    }
    return false;
  }

  // 查找波浪链接
  physics::LinkPtr waveLink;
  auto links = waveModel->GetLinks();
  for (auto const& l : links) {
    if (l->GetName() == "ocean_waves_link") { 
      waveLink = l; 
      break; 
    }
  }

  if (!waveLink && !links.empty()) {
    waveLink = links.front();
  }

  wave_model_ = waveModel;
  wave_link_ = waveLink;
  wave_bound_ = (wave_model_ != nullptr);

  if (wave_bound_) {
    gzmsg << "[terrain_detector] ✓ Bound to wave model '" 
          << wave_model_->GetName() << "'" << std::endl;
  }

  return wave_bound_;
}

double TerrainDetectorPlugin::QuerySurfaceZ(double x, double y) const {
  if (!world_) {
    return water_surface_level_;
  }

  double simTime = world_->SimTime().Double();

  // 方法1: 使用全局波浪场查询
  if (auto wf = asv::GetWavefield(world_)) {
    double z = wf->SurfaceElevation(x, y, simTime);
    return z;
  }

  // 方法2: 使用绑定的波浪模型
  if (wave_model_) {
    // 这里可以添加更多波浪场查询逻辑
    // 暂时返回备用值
  }

  // 备用: 返回静态水面高度
  return water_surface_level_;
}

void TerrainDetectorPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time = _info.simTime;
  if ((current_time - last_update_time_).Double() < update_period_) {
    return;
  }
  last_update_time_ = current_time;

  // 如果还没绑定波浪场,尝试绑定
  if (!wave_bound_) {
    BindWavefieldOnce();
  }

  DetectTerrain();
  PublishTerrainStatus();
}

void TerrainDetectorPlugin::DetectTerrain() {
  ignition::math::Vector3d pos = link_->WorldPose().Pos();

  // **关键修改**: 获取当前位置的实时波浪高度
  double current_water_level = QuerySurfaceZ(pos.X(), pos.Y());

  ignition::math::Vector3d ray_start = pos + ignition::math::Vector3d(-0.5, 0, -0.1);
  ignition::math::Vector3d ray_end = pos - ignition::math::Vector3d(0, 0, 1000);
  
  ground_ray_->SetPoints(ray_start, ray_end);
  
  std::string entity_below;
  double dist;
  ground_ray_->GetIntersection(dist, entity_below);

  bool intersection_found = !entity_below.empty() && dist > 0 && dist < 1000;

  static int debug_counter = 0;
  if (debug_counter % 50 == 0) {
    gzdbg << "[terrain_detector] Pos: " << pos 
          << ", WaveZ: " << current_water_level
          << ", Entity: '" << entity_below 
          << "', Dist: " << dist 
          << ", RelZ: " << (pos.Z() - current_water_level) << std::endl;
  }
  debug_counter++;

  // **关键修改**: 使用实时波浪高度进行判断
  double z_relative = pos.Z() - current_water_level;

  if (intersection_found && IsWaterEntity(entity_below)) {
    // 检测到水体实体
    if (z_relative < -surface_tolerance_) {
      terrain_type_ = TerrainType::UNDERWATER;
      terrain_name_ = "underwater";
    } else if (std::abs(z_relative) <= surface_tolerance_) {
      terrain_type_ = TerrainType::SEA_SURFACE;
      terrain_name_ = "sea_surface";
    } else {
      terrain_type_ = TerrainType::SEA_SURFACE;
      terrain_name_ = "sea_above";
    }
  }
  else if (intersection_found) {
    // 检测到其他实体
    if (z_relative < -surface_tolerance_) {
      terrain_type_ = TerrainType::UNDERWATER;
      terrain_name_ = "underwater";
    } else {
      terrain_type_ = TerrainType::LAND;
      terrain_name_ = "land";
    }
  }
  else {
    // 没有检测到实体
    if (z_relative < -surface_tolerance_) {
      terrain_type_ = TerrainType::UNDERWATER;
      terrain_name_ = "underwater";
    } else if (std::abs(z_relative) <= surface_tolerance_) {
      terrain_type_ = TerrainType::SEA_SURFACE;
      terrain_name_ = "sea_surface";
    } else if (z_relative < 10.0) {
      terrain_type_ = TerrainType::SEA_SURFACE;
      terrain_name_ = "sea_above";
    } else {
      terrain_type_ = TerrainType::UNKNOWN;
      terrain_name_ = "unknown";
    }
  }
}

void TerrainDetectorPlugin::PublishTerrainStatus() {
  std_msgs::String msg;
  msg.data = terrain_name_;
  terrain_pub_.publish(msg);
  
  static int pub_counter = 0;
  if (pub_counter % 50 == 0) {
    gzdbg << "[terrain_detector] Published: " << terrain_name_ << std::endl;
  }
  pub_counter++;
}

TerrainType TerrainDetectorPlugin::GetTerrainType() const {
  return terrain_type_;
}

std::string TerrainDetectorPlugin::GetTerrainName() const {
  return terrain_name_;
}

bool TerrainDetectorPlugin::IsOverSea() const {
  return (terrain_type_ == TerrainType::SEA_SURFACE || 
          terrain_type_ == TerrainType::UNDERWATER);
}

bool TerrainDetectorPlugin::IsUnderwater() const {
  return terrain_type_ == TerrainType::UNDERWATER;
}

bool TerrainDetectorPlugin::IsOverLand() const {
  return terrain_type_ == TerrainType::LAND;
}

GZ_REGISTER_MODEL_PLUGIN(TerrainDetectorPlugin)

}

