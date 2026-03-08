#ifndef ASV_PHASE_SWITCH_PLUGIN_HH
#define ASV_PHASE_SWITCH_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <nezha_plugins/GetPhaseSample.h>
#include "nezha_underwaterHydrodynamicsPlugin.hh"  

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>


namespace asv
{
  class HydrodynamicsPlugin;
}

namespace gazebo
{

    class HydrodynamicModelRegistry;  
}

namespace asv
{
    extern HydrodynamicsPlugin* g_lastHydrodynamicsPlugin;
    extern std::vector<HydrodynamicsPlugin*> g_nasvPluginRegistry;
    extern std::mutex g_nasvRegistryMutex;
}


namespace asv
{


class PhaseSwitchPlugin : public gazebo::ModelPlugin
{
public:

  enum class PhaseState
  {
    UNKNOWN,      
    ABOVE,       
    WATER_ENTRY,  
    SURFACE,      
    WATER_EXIT,  
    BELOW         
  };

  enum class ForceMode
  {
    OFF,           
    SURFACE_ONLY,   
    DIRECT,        
    CALLBACK       
  };
 PhaseSwitchPlugin();
  virtual ~PhaseSwitchPlugin();
  

  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  PhaseState GetCurrentPhase() const;
  
  std::string StateToString(PhaseState state) const;
private:
  void FindAndCachePlugins();
 


struct Data
{

    gazebo::physics::ModelPtr model;
    gazebo::physics::LinkPtr targetLink;
    gazebo::physics::WorldPtr world;  
    
    std::string robotNamespace;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceClient phaseClient;
    std::thread rosSpinThread;
    std::atomic<bool> rosSpin{false};
    std::atomic<bool> rosOK{true}; 
    std::string surfacePluginName = "nezha_hydrodynamics";      
    std::string underwaterPluginName = "nezha_hydrodynamics";   
    
    std::string surfacePluginFile = "";   
    std::string underwaterPluginFile = ""; 
    

    std::string activeHydrodynamicPlugin = "None";  

    double lastKnownWaterZ = 0.0;
    int serviceFailCount = 0;
    int serviceFailWarnThreshold = 100; 
    
    PhaseState currentState = PhaseState::UNKNOWN;
    PhaseState lastDetected = PhaseState::UNKNOWN;
    PhaseState previousStableState = PhaseState::UNKNOWN;
    int transitionCounter = 0;
    int transitionFrames = 10;
    
    double aboveThreshold = 0.5;
    double belowThreshold = -0.5;
    double entryExitThreshold = 0.1;
    double entryExitVelocityThreshold = 0.2;  
    
    bool inTransition = false;
    double transitionStartTime = 0.0;
    double surfaceWeight = 0.0;
    double underwaterWeight = 0.0;
    double entryExitMaxDuration = 5.0;
    double entryStartTime = 0.0; 
    double exitStartTime = 0.0;   
    double forceScaleFactor = 1.0;
    bool forceScaleAppliedOnce = false; // 避免重复日志和无意义重复写
    ignition::math::Vector3d prevF = ignition::math::Vector3d::Zero;  
    ignition::math::Vector3d prevT = ignition::math::Vector3d::Zero;  
    

    ignition::math::Vector3d flowVelWorld = ignition::math::Vector3d::Zero;  
    
    HydrodynamicsPlugin* surfacePlugin = nullptr;
gazebo::HMFossen* fossenModel = nullptr;

    
    std::string targetModelName;  
    
    double updateRate = 50.0;
    gazebo::common::Time lastUpdateTime;
    gazebo::event::ConnectionPtr updateConn;
    

    ForceMode currentForceMode = ForceMode::OFF;
    

    Data()
        : rosSpin(false),
          rosOK(true),
          serviceFailCount(0),
          serviceFailWarnThreshold(100),
          currentState(PhaseState::UNKNOWN),
          lastDetected(PhaseState::UNKNOWN),
          previousStableState(PhaseState::UNKNOWN),
          transitionCounter(0),
          transitionFrames(10),
          aboveThreshold(0.5),
          belowThreshold(-0.5),
          entryExitThreshold(0.1),
          entryExitVelocityThreshold(0.2),
          inTransition(false),
          transitionStartTime(0.0),
          surfaceWeight(0.0),
          underwaterWeight(0.0),
          entryExitMaxDuration(5.0),
          entryStartTime(0.0),
          exitStartTime(0.0),
          prevF(ignition::math::Vector3d::Zero),
          prevT(ignition::math::Vector3d::Zero),
          flowVelWorld(ignition::math::Vector3d::Zero),
          surfacePlugin(nullptr),
          fossenModel(nullptr),
          updateRate(50.0),
          currentForceMode(ForceMode::OFF)
    {
    }
};


  std::unique_ptr<Data> data;  
    bool QueryWaveProbe(double& waterZ, 
                       std::string& phaseName, 
                       double& zOverL) const;
    

    PhaseState MapWaveProbePhase(const std::string& waveProbePhase) const;
    

    void OnUpdate(const gazebo::common::UpdateInfo& _info);


  void TransitionTo(PhaseState newState, double simTime);
  

  void OnStateEnter(PhaseState newState);


  void HandleAbove(const gazebo::common::Time& simTime);
    void HandleWaterExit(const gazebo::common::Time& simTime);
    void HandleWaterEntry(const gazebo::common::Time& simTime);

  void HandleSurface(const gazebo::common::Time& simTime);
  
  void HandleInterface(const gazebo::common::Time& simTime);  


  void HandleBelow(const gazebo::common::Time& simTime);


  PhaseState DetectPhase(double linkZ, double waterZ, double verticalVel);


  void SetNASVPluginsEnabled(bool enable);
  

  void SetNUUVModelsEnabled(bool enable);
  

void SetSurfacePluginMode(ForceMode mode);


  void GetNASVLastForce(
      ignition::math::Vector3d& force,
      ignition::math::Vector3d& torque);
  void EnableSurfaceForces();

  void SuppressSurfaceForces();


  bool QueryPhase(PhaseState& outState);

  void SpinThread();


  double QuerySurfaceZ(double x, double y) const;
  

  PhaseState ParsePhase(const std::string& name);
  

  std::string PhaseToString(PhaseState s);
  



};

} // namespace asv

#endif // ASV_PHASE_SWITCH_PLUGIN_HH


