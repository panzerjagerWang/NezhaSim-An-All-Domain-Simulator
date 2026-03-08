#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <Eigen/Dense> 

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <thread>

#include <nezha_plugins/HydrodynamicsForces.h>
// ============================================

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

namespace asv
{
  class HydrodynamicsPluginPrivate;

  class GAZEBO_VISIBLE HydrodynamicsPlugin : public gazebo::ModelPlugin
  {
  public:
    /// \brief Constructor
    HydrodynamicsPlugin();

    /// \brief Destructor
    virtual ~HydrodynamicsPlugin();

    /// \brief Inherited from ModelPlugin
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief Custom initialisation
    void Init();

    /// \brief Custom plugin reset
    void Reset();

    /// \brief Enable or disable the plugin
    /// \param[in] enable True to enable, false to disable
    void SetEnabled(bool enable);

    /// \brief Check if the plugin is enabled
    /// \return True if enabled, false otherwise
    bool IsEnabled() const;
    
    void ClearLastForces();
    
    /// \brief Get the name of this plugin instance
    std::string GetName() const;

    /// \brief Get the model this plugin is attached to
    gazebo::physics::ModelPtr GetModel() const;

    /// \brief Get the last computed total force
    ignition::math::Vector3d GetLastTotalForce() const;

    /// \brief Get the last computed total torque
    ignition::math::Vector3d GetLastTotalTorque() const;

  private:
    void InitPhysics();
    void InitMarkers();
    void InitWaterPatchMarkers();
    void InitWaterlineMarkers();
    void InitUnderwaterSurfaceMarkers();
    
    void ComputeAddedCoriolisMatrix(
      const Vector6d& vel,
      const Matrix6d& Ma,
      Matrix6d& Ca) const;
    
    void ComputeDampingMatrix(
      const Vector6d& vel,
      Matrix6d& D) const;
    double ComputeDynamicWaveClamp(
      double _submersionRatio,
      const ignition::math::Vector3d& _velocity) const;
    void ResetPhysics();
    void ResetMarkers();
    void ResetWaterPatchMarkers();
    void ResetWaterlineMarkers();
    void ResetUnderwaterSurfaceMarkers();

    void OnUpdate();
    void UpdatePhysics();
    void UpdateMarkers();
    void UpdateWaterPatchMarkers();
    void UpdateWaterlineMarkers();
    void UpdateUnderwaterSurfaceMarkers();

    void Fini();
    void FiniPhysics();
    void FiniMarkers();
    void FiniWaterPatchMarkers();
    void FiniWaterlineMarkers();
    void FiniUnderwaterSurfaceMarkers();

    bool IsLinkExempt(const std::string& linkName) const;

    static std::string BaseNameOfLink(const std::string& fullName);
    
    /// \brief Callback for gztopic "~/hydrodynamics"
    void OnHydrodynamicsMsg(ConstParam_VPtr &_msg);
    double ComputeWaveForceModulation(double _submersionRatio) const;
    // ========== ✅ 新增: ROS Service 回调函数 ==========
    /// \brief Service callback to get hydrodynamics forces
    /// \param[in] req Service request (empty)
    /// \param[out] res Service response with force components
    /// \return True if service call succeeded
    bool OnGetForcesService(
      nezha_plugins::HydrodynamicsForces::Request &req,
      nezha_plugins::HydrodynamicsForces::Response &res);
    
    /// \brief ROS callback queue thread function
    void QueueThread();
    // ==================================================

  private:
    /// \brief Flag to enable/disable the plugin
    bool enabled_ = true;
    
    /// \brief Frame counter for delayed activation
    int initFrameCount_ = 0;
    
    /// \brief Number of frames to wait before activating
    static const int REQUIRED_FRAMES = 1000; 
    
    /// \brief Pointer to the class private data
    std::shared_ptr<HydrodynamicsPluginPrivate> data;


  };

  class HydrodynamicsPluginRegistry
  {
  public:
    static HydrodynamicsPluginRegistry& GetInstance()
    {
      static HydrodynamicsPluginRegistry instance;
      return instance;
    }

    void Register(HydrodynamicsPlugin* plugin, gazebo::physics::ModelPtr model)  
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (plugin && model)
      {
        registry_[model->GetScopedName()] = plugin;
        gzmsg << "[HydrodynamicsPluginRegistry] Registered: " 
              << model->GetScopedName() << std::endl;
      }
    }

    void Unregister(gazebo::physics::ModelPtr model)  
    {
      if (!model) return;
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = registry_.find(model->GetScopedName());
      if (it != registry_.end())
      {
        registry_.erase(it);
        gzmsg << "[HydrodynamicsPluginRegistry] Unregistered: " 
              << model->GetScopedName() << std::endl;
      }
    }

    HydrodynamicsPlugin* FindByModel(gazebo::physics::ModelPtr model)  
    {
      if (!model) return nullptr;
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = registry_.find(model->GetScopedName());
      return (it != registry_.end()) ? it->second : nullptr;
    }

    std::vector<HydrodynamicsPlugin*> GetAllPlugins()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::vector<HydrodynamicsPlugin*> plugins;
      for (auto& pair : registry_)
      {
        if (pair.second) plugins.push_back(pair.second);
      }
      return plugins;
    }

    void DebugPrint()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gzmsg << "╔════════════════════════════════════════╗" << std::endl;
      gzmsg << "║  HydrodynamicsPluginRegistry Status   ║" << std::endl;
      gzmsg << "╠════════════════════════════════════════╣" << std::endl;
      gzmsg << "║  Total plugins: " << std::setw(20) << registry_.size() << " ║" << std::endl;
      for (const auto& pair : registry_)
      {
        gzmsg << "║  • " << std::setw(35) << std::left 
              << pair.first << " ║" << std::endl;
      }
      gzmsg << "╚════════════════════════════════════════╝" << std::endl;
    }

  private:
    HydrodynamicsPluginRegistry() = default;
    ~HydrodynamicsPluginRegistry() = default;
    HydrodynamicsPluginRegistry(const HydrodynamicsPluginRegistry&) = delete;
    HydrodynamicsPluginRegistry& operator=(const HydrodynamicsPluginRegistry&) = delete;
    
    std::map<std::string, HydrodynamicsPlugin*> registry_;
    std::mutex mutex_;
  };

} // namespace asv

#endif // _ASV_WAVE_SIM_GAZEBO_PLUGINS_HYDRODYNAMICS_PLUGIN_HH_
