#include "nezha_phaseSwitchPlugin.hh"
#include "nezha_surfacePlugin.hh"  
#include <nezha_plugins/GetPhaseSample.h>  
#include "nezha_underwaterHydrodynamicsPlugin.hh"  
#include <ros/service_client.h>            
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
    class HydrodynamicModel;
    class HMFossen; 
        class HydrodynamicModelRegistry;  

}


namespace asv
{

extern HydrodynamicsPlugin* g_lastHydrodynamicsPlugin;
extern std::vector<HydrodynamicsPlugin*> g_nasvPluginRegistry;
extern std::mutex g_nasvRegistryMutex;
PhaseSwitchPlugin::PhaseSwitchPlugin()
  : data(new Data)
{
}

PhaseSwitchPlugin::~PhaseSwitchPlugin()
{
  data->rosSpin = false;
  if (data->rosSpinThread.joinable())
    data->rosSpinThread.join();
  data->updateConn.reset();
}


void PhaseSwitchPlugin::Load(gazebo::physics::ModelPtr _model,
                             sdf::ElementPtr _sdf)
{
    gzmsg << "\n";
    gzmsg << "╔════════════════════════════════════════════════════════════╗\n";
    gzmsg << "║                                                            ║\n";
    gzmsg << "║              ███╗   ██╗███████╗███████╗██╗  ██╗ █████╗     ║\n";
    gzmsg << "║              ████╗  ██║██╔════╝╚══███╔╝██║  ██║██╔══██╗    ║\n";
    gzmsg << "║              ██╔██╗ ██║█████╗    ███╔╝ ███████║███████║    ║\n";
    gzmsg << "║              ██║╚██╗██║██╔══╝   ███╔╝  ██╔══██║██╔══██║    ║\n";
    gzmsg << "║              ██║ ╚████║███████╗███████╗██║  ██║██║  ██║    ║\n";
    gzmsg << "║              ╚═╝  ╚═══╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝    ║\n";
    gzmsg << "║                                                            ║\n";
    gzmsg << "║               Nezha Sim Plugin - Transmedia Control        ║\n";
    gzmsg << "║                   Version V1 ROS - 2025                    ║\n";
    gzmsg << "║                                                            ║\n";
    gzmsg << "╚════════════════════════════════════════════════════════════╝\n";
    gzmsg << "\n";
    data->model = _model;
    
    if (!_model) {
        gzerr << "[PhaseSwitchPlugin] Invalid model pointer!" << std::endl;
        return;
    }
    

    if (_sdf->HasElement("robot_namespace")) {
        data->robotNamespace = _sdf->Get<std::string>("robot_namespace");
    } 
    else {
        data->robotNamespace = _model->GetName();
    }

    if (_sdf->HasElement("surface_plugin_name")) {
        data->surfacePluginName = _sdf->Get<std::string>("surface_plugin_name");
        gzmsg << "[PhaseSwitchPlugin] Surface plugin name: " 
              << data->surfacePluginName << std::endl;
    }
    
    if (_sdf->HasElement("underwater_plugin_name")) {
        data->underwaterPluginName = _sdf->Get<std::string>("underwater_plugin_name");
        gzmsg << "[PhaseSwitchPlugin] Underwater plugin name: " 
              << data->underwaterPluginName << std::endl;
    }
    
    if (_sdf->HasElement("surface_plugin_file")) {
        data->surfacePluginFile = _sdf->Get<std::string>("surface_plugin_file");
        gzmsg << "[PhaseSwitchPlugin] Surface plugin file: " 
              << data->surfacePluginFile << std::endl;
    }
    
    if (_sdf->HasElement("underwater_plugin_file")) {
        data->underwaterPluginFile = _sdf->Get<std::string>("underwater_plugin_file");
        gzmsg << "[PhaseSwitchPlugin] Underwater plugin file: " 
              << data->underwaterPluginFile << std::endl;
    }

    if (!data->robotNamespace.empty() && data->robotNamespace[0] != '/') {
        data->robotNamespace = "/" + data->robotNamespace;
    }
    if (!data->robotNamespace.empty() && data->robotNamespace.back() == '/') {
        data->robotNamespace.pop_back();
    }
    
    gzmsg << "[PhaseSwitchPlugin] Robot namespace: " << data->robotNamespace << std::endl;
    

    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "phase_switch_plugin", 
                 ros::init_options::NoSigintHandler);
    }
    
data->rosNode = std::make_unique<ros::NodeHandle>(data->robotNamespace);

    
    std::string serviceName = "transmedia/get_phase_sample";  
    
data->phaseClient = data->rosNode->serviceClient<nezha_plugins::GetPhaseSample>(
    serviceName
    );
    
    gzmsg << "[PhaseSwitchPlugin] Waiting for WaveProbe service: " 
          << data->robotNamespace << "/" << serviceName << std::endl;

    if (data->phaseClient.waitForExistence(ros::Duration(5.0))) {
        gzmsg << "[PhaseSwitchPlugin] ✓ Connected to WaveProbe service!" << std::endl;
    } else {
        gzwarn << "[PhaseSwitchPlugin] ⚠ WaveProbe service not ready. "
               << "Will retry during runtime." << std::endl;
    }
    
if (_sdf->HasElement("target_link")) {
    std::string linkName = _sdf->Get<std::string>("target_link");
    data->targetLink = _model->GetLink(linkName);
    if (!data->targetLink) {
        gzwarn << "[PhaseSwitchPlugin] Link '" << linkName 
               << "' not found! Will try to use first available link." << std::endl;

        auto links = _model->GetLinks();
        if (!links.empty()) {
            data->targetLink = links[0];
            gzmsg << "[PhaseSwitchPlugin] Using first link: " 
                  << data->targetLink->GetName() << std::endl;
        }
    }
} else {

    auto links = _model->GetLinks();
    if (!links.empty()) {
        data->targetLink = links[0];
        gzmsg << "[PhaseSwitchPlugin] No target_link specified, using first link: " 
              << data->targetLink->GetName() << std::endl;
    } else {
        gzwarn << "[PhaseSwitchPlugin] Model has no links! "
               << "Plugin will operate in model-level mode." << std::endl;
    }
}


if (data->targetLink) {
    gzmsg << "[PhaseSwitchPlugin] Target link: " 
          << data->targetLink->GetName() << std::endl;
} else {
    gzwarn << "[PhaseSwitchPlugin] Running without target link. "
           << "Some features may be limited." << std::endl;
}

    

    if (_sdf->HasElement("above_threshold"))
        data->aboveThreshold = _sdf->Get<double>("above_threshold");
    
    if (_sdf->HasElement("below_threshold"))
        data->belowThreshold = _sdf->Get<double>("below_threshold");

    if (_sdf->HasElement("transition_frames"))
        data->transitionFrames = _sdf->Get<int>("transition_frames");
    data->transitionFrames = 0;  // 禁用防抖

    if (_sdf->HasElement("update_rate"))
        data->updateRate = _sdf->Get<double>("update_rate");
    

    data->updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&PhaseSwitchPlugin::OnUpdate, this, std::placeholders::_1)
    );
    

    data->rosSpin = true;
    data->rosSpinThread = std::thread([this]() {
        while (data->rosSpin && ros::ok()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    

    gzmsg << "[PhaseSwitchPlugin] ✓ Load complete\n"
          << "  Model         : " << _model->GetName() << "\n"
          << "  Target Link   : " << data->targetLink->GetName() << "\n"
          << "  Namespace     : " << data->robotNamespace << "\n"
          << "  Service       : " << data->robotNamespace << "/" << serviceName << "\n"
          << "  Update Rate   : " << data->updateRate << " Hz\n"
          << "  Debounce      : " << data->transitionFrames << " frames\n";

gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo& _info)
{
    static bool pluginsFound = false;
    if (!pluginsFound)
    {
        pluginsFound = true;
        
        gzmsg << "\n========================================" << std::endl;
        gzmsg << "[PhaseSwitchPlugin] First Update - Finding Plugins..." << std::endl;
        gzmsg << "  Simulation Time: " << _info.simTime.Double() << " s" << std::endl;
        


        {
            std::lock_guard<std::mutex> lock(g_nasvRegistryMutex);
            gzmsg << "  NASV Plugins: " << g_nasvPluginRegistry.size() << std::endl;
            if (data->surfacePlugin) {

                gzmsg << "    ✓ Surface Plugin Found for model: " << data->model->GetName() << std::endl;
            } else {
                gzwarn << "    ✗ Surface Plugin NOT Found" << std::endl;
            }
        }
        

{

    auto models = gazebo::HydrodynamicModelRegistry::GetInstance().GetModels();
    
    gzmsg << "  NUUV Models: " << models.size() << std::endl;
    
    if (data->fossenModel) {
        gzmsg << "    ✓ Fossen Model Found: " << data->fossenModel->GetName() << std::endl;
        
        auto modelLink = data->fossenModel->GetLink();
        
        if (modelLink) {
            gzmsg << "    ✓ Link: " << modelLink->GetName() << std::endl;
        } else {
            gzwarn << "    ✗ Link is null!" << std::endl;
        }
        
        gzmsg << "    ✓ Enabled: " << (data->fossenModel->IsEnabled() ? "YES" : "NO") << std::endl;
    } else {
        gzerr << "    ✗ Fossen Model NOT Found" << std::endl;
        
        if (models.empty()) {
            gzerr << "    ⚠ NUUV registry is EMPTY!" << std::endl;
            gzerr << "    ⚠ Check your model SDF for underwater plugin." << std::endl;
        } else {
            gzerr << "    ⚠ Registry has " << models.size() 
                  << " models, but none match link: " 
                  << (data->targetLink ? data->targetLink->GetName() : "null") << std::endl;
            

            gzerr << "    ⚠ Models in registry:" << std::endl;
            for (size_t i = 0; i < models.size(); ++i) {
                auto* model = models[i];
                if (model && model->GetLink()) {
                    gzerr << "      [" << i << "] " << model->GetLink()->GetScopedName() << std::endl;
                }
            }
        }
    }
}

        
        gzmsg << "========================================\n" << std::endl;
    }
});
    gzmsg << "╔════════════════════════════════════════╗" << std::endl;
    gzmsg << "║  NASV Plugin Registry Status          ║" << std::endl;
    gzmsg << "╠════════════════════════════════════════╣" << std::endl;
    
    std::lock_guard<std::mutex> lock(asv::g_nasvRegistryMutex);
    gzmsg << "║  Total plugins: " << std::setw(19) 
          << asv::g_nasvPluginRegistry.size() << " ║" << std::endl;
    
    for (size_t i = 0; i < asv::g_nasvPluginRegistry.size(); ++i)
    {
        auto* plugin = asv::g_nasvPluginRegistry[i];
        if (plugin)
        {
            auto model = plugin->GetModel();
            gzmsg << "║  [" << i << "] " << std::setw(33) << std::left 
                  << (model ? model->GetName() : "NULL") << " ║" << std::endl;
            gzmsg << "║      Address: " << std::setw(23) 
                  << plugin << " ║" << std::endl;
        }
    }
    gzmsg << "╚════════════════════════════════════════╝" << std::endl;
}

void PhaseSwitchPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{

    static bool pluginsFound = false;
    static gazebo::common::Time lastRetry;
    std::string activePlugin;

    if (!pluginsFound) {
        if ((_info.simTime - lastRetry).Double() > 1.0) {
            gzmsg << "[PhaseSwitchPlugin] Retrying plugin search at t=" 
                  << _info.simTime.Double() << "s..." << std::endl;
            
            FindAndCachePlugins();

            if (data->surfacePlugin) {
                pluginsFound = true;
                gzmsg << "[PhaseSwitchPlugin] ✓ Surface plugin found, proceeding..." << std::endl;
                
                if (!data->fossenModel) {
                    gzwarn << "[PhaseSwitchPlugin] ⚠ Underwater model not found, "
                           << "will operate in surface-only mode" << std::endl;
                }
            } else {
                gzwarn << "[PhaseSwitchPlugin] Still missing surface plugin" << std::endl;
            }
            
            lastRetry = _info.simTime;
        }
        
        if (!data->surfacePlugin) {
            return;
        }
    }

    if (!data->surfacePlugin && (_info.simTime - lastRetry).Double() > 1.0) {
        gzwarn << "[PhaseSwitchPlugin] Retrying plugin search..." << std::endl;
        FindAndCachePlugins();
        lastRetry = _info.simTime;
    }

    double dt = (_info.simTime - data->lastUpdateTime).Double();
    if (dt < 1.0 / data->updateRate) {
        return;
    }
    data->lastUpdateTime = _info.simTime;
    
    if (!data->targetLink) {
        static bool warnedOnce = false;
        if (!warnedOnce) {
            gzwarn << "[PhaseSwitchPlugin] No target link available, "
                   << "skipping update." << std::endl;
            warnedOnce = true;
        }
        return;
    }
    
    double waterZ = 0.0;
    std::string waveProbePhase;
    double zOverL = 0.0;
    
    bool serviceOK = QueryWaveProbe(waterZ, waveProbePhase, zOverL);
    
    if (!serviceOK) {
        if (data->serviceFailCount > 500) {
            gzwarn << "[PhaseSwitchPlugin] Service timeout. Forcing BELOW state." << std::endl;
            TransitionTo(PhaseState::BELOW, _info.simTime.Double());
        }
        return;
    }
    
    PhaseState detected = MapWaveProbePhase(waveProbePhase);
    
static gazebo::common::Time lastDebugPrint;
if ((_info.simTime - lastDebugPrint).Double() > 1.0)
{
    ignition::math::Pose3d pose = data->targetLink->WorldPose();
    ignition::math::Vector3d vel = data->targetLink->WorldLinearVel();
    
    gzdbg << "╔════════════════════════════════════════╗" << std::endl;
    gzdbg << "║  PhaseSwitchPlugin Status             ║" << std::endl;
    gzdbg << "╠════════════════════════════════════════╣" << std::endl;
    gzdbg << "║  Robot Z:    " << std::setw(24) << std::fixed << std::setprecision(5) 
          << pose.Pos().Z() << " m ║" << std::endl;
    gzdbg << "║  Water Z:    " << std::setw(24) << waterZ << " m ║" << std::endl;
    gzdbg << "║  Relative:   " << std::setw(24) << (pose.Pos().Z() - waterZ) << " m ║" << std::endl;
    gzdbg << "║  z/L:        " << std::setw(27) << zOverL << " ║" << std::endl;
    gzdbg << "║  Velocity Z: " << std::setw(24) << vel.Z() << " m/s ║" << std::endl;
    gzdbg << "║  WaveProbe:  " << std::setw(27) << waveProbePhase << " ║" << std::endl;
    gzdbg << "║  State:      " << std::setw(27) << StateToString(data->currentState) << " ║" << std::endl;
    gzdbg << "╠════════════════════════════════════════╣" << std::endl;
    
    if (data->currentState == PhaseState::BELOW)
    {
        activePlugin = data->underwaterPluginName + " (" + data->underwaterPluginFile + ")";
    }
    else if (data->currentState == PhaseState::SURFACE || 
             data->currentState == PhaseState::WATER_ENTRY ||
             data->currentState == PhaseState::WATER_EXIT)
    {
        activePlugin = data->surfacePluginName + " (" + data->surfacePluginFile + ")";
    }
    else
    {
        activePlugin = "None (Aerial)";
    }
    
    gzdbg << "║  Active:     " << std::setw(27) << activePlugin << " ║" << std::endl;
    gzdbg << "╚════════════════════════════════════════╝" << std::endl;
    
    lastDebugPrint = _info.simTime;
}
    

    if (detected != data->lastDetected) {
        data->transitionCounter = 0;
        data->lastDetected = detected;
        
        gzdbg << "[PhaseSwitchPlugin] Phase change detected: " 
              << StateToString(data->currentState) << " -> " 
              << StateToString(detected) << std::endl;
    } else {
        data->transitionCounter++;
    }
    

    if (data->transitionCounter >= data->transitionFrames) {
        if (detected != data->currentState) {
            gzmsg << "[PhaseSwitchPlugin] ✓ Phase transition: " 
                  << StateToString(data->currentState) << " -> " 
                  << StateToString(detected) << std::endl;
            
            TransitionTo(detected, _info.simTime.Double());
        }
    }
    
    switch (data->currentState)
    {
        case PhaseState::ABOVE:
            HandleAbove(_info.simTime);
            break;
        case PhaseState::SURFACE:
            HandleSurface(_info.simTime);
            break;
        case PhaseState::BELOW:
            HandleBelow(_info.simTime);
            break;
        case PhaseState::WATER_EXIT:
            HandleWaterExit(_info.simTime);  
            break;
       case PhaseState::WATER_ENTRY:
            HandleWaterEntry(_info.simTime);  
            break;
        default:
            break;
    }
    static gazebo::common::Time lastStatusPrint;
    if ((_info.simTime - lastStatusPrint).Double() > 10.0)
    {
        gzmsg << "\n╔════════════════════════════════════════╗" << std::endl;
        gzmsg << "║  Plugin Status Check                  ║" << std::endl;
        gzmsg << "╠════════════════════════════════════════╣" << std::endl;
        

        if (data->surfacePlugin)
        {
            gzmsg << "║  NASV Plugin: " << std::setw(23) << std::left 
                  << (data->surfacePlugin->IsEnabled() ? "ENABLED" : "DISABLED") 
                  << " ║" << std::endl;
            
            auto force = data->surfacePlugin->GetLastTotalForce();
            gzmsg << "║  Last Force:  (" 
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << force.X() << ", "
                  << std::setw(6) << force.Y() << ", "
                  << std::setw(6) << force.Z() << ") N  ║" << std::endl;
        }
        else
        {
            gzmsg << "║  NASV Plugin: NULL                    ║" << std::endl;
        }
        
        if (data->fossenModel)
        {
            gzmsg << "║  NUUV Model:  " << std::setw(23) << std::left 
                  << (data->fossenModel->IsEnabled() ? "ENABLED" : "DISABLED") 
                  << " ║" << std::endl;
        }
        else
        {
            gzmsg << "║  NUUV Model:  NULL                    ║" << std::endl;
        }
        
        gzmsg << "║  Current State: " << std::setw(21) << std::left 
              << StateToString(data->currentState) << " ║" << std::endl;
        gzmsg << "╚════════════════════════════════════════╝\n" << std::endl;
        
        lastStatusPrint = _info.simTime;
    }
}

void PhaseSwitchPlugin::HandleBelow(const gazebo::common::Time& simTime)
{

    SetNUUVModelsEnabled(true);
    

    static gazebo::common::Time lastDisableTime;
    if ((simTime - lastDisableTime).Double() > 0.01)  
    {
        SetNASVPluginsEnabled(false);
        lastDisableTime = simTime;
    }
    
    data->activeHydrodynamicPlugin = data->underwaterPluginName + 
                                    " (" + data->underwaterPluginFile + ")";
    
    static gazebo::common::Time lastDebugPrint;
    if ((simTime - lastDebugPrint).Double() > 1.0)
    {
        gzmsg << "╔════════════════════════════════════════╗" << std::endl;
        gzmsg << "║  BELOW State Status                   ║" << std::endl;
        gzmsg << "╠════════════════════════════════════════╣" << std::endl;
        

        if (data->surfacePlugin)
        {
            gzmsg << "║  NASV: " << std::setw(31) << std::left 
                  << (data->surfacePlugin->IsEnabled() ? "ENABLED ❌" : "DISABLED ✓") 
                  << " ║" << std::endl;
        }
        

        if (data->fossenModel)
        {
            gzmsg << "║  NUUV: " << std::setw(31) << std::left 
                  << (data->fossenModel->IsEnabled() ? "ENABLED ✓" : "DISABLED ❌") 
                  << " ║" << std::endl;
        }
        
        gzmsg << "║  Active: " << std::setw(29) << std::left 
              << data->activeHydrodynamicPlugin << " ║" << std::endl;
        gzmsg << "╚════════════════════════════════════════╝" << std::endl;
        
        lastDebugPrint = simTime;
    }
}


bool PhaseSwitchPlugin::QueryPhase(PhaseState& outState)
{
  if (!data->rosOK)
  {
    static bool warned = false;
    if (!warned)
    {
      gzwarn << "[PhaseSwitchPlugin] ROS not available for phase query" << std::endl;
      warned = true;
    }
    return false;
  }

  nezha_plugins::GetPhaseSample srv;
  if (!data->phaseClient.call(srv))
  {
    data->serviceFailCount++;
    if (data->serviceFailCount == data->serviceFailWarnThreshold)
    {
      gzwarn << "[PhaseSwitchPlugin] Phase service failed " 
             << data->serviceFailCount << " times" << std::endl;
    }
    return false;
  }


  if (data->serviceFailCount > 0)
  {
    gzmsg << "[PhaseSwitchPlugin] Phase service recovered" << std::endl;
  }
  data->serviceFailCount = 0;
  
  outState = ParsePhase(srv.response.phase_name);
  return true;
}
void PhaseSwitchPlugin::OnStateEnter(PhaseState newState)
{
    switch (newState)
    {
        case PhaseState::ABOVE:
            gzmsg << "[PhaseSwitchPlugin] 进入空中模式\n";
            SetNASVPluginsEnabled(false);  
            SetNUUVModelsEnabled(false);   
            
            data->prevF = ignition::math::Vector3d::Zero;
            data->prevT = ignition::math::Vector3d::Zero;
            break;

        case PhaseState::WATER_ENTRY:
            gzmsg << "[PhaseSwitchPlugin] 进入入水过程\n";
            SetNASVPluginsEnabled(true);  
            SetNUUVModelsEnabled(false);   
            break;

        case PhaseState::SURFACE:
            gzmsg << "[PhaseSwitchPlugin] 进入水面模式\n";
            SetNASVPluginsEnabled(true);  
            SetNUUVModelsEnabled(false);   
            break;

        case PhaseState::WATER_EXIT:
            gzmsg << "[PhaseSwitchPlugin] 进入出水过程\n";
            SetNASVPluginsEnabled(true);   
            SetNUUVModelsEnabled(false);   
            
            if (data->surfacePlugin && data->surfacePlugin->IsEnabled()) {
                data->prevF = data->surfacePlugin->GetLastTotalForce();
                data->prevT = data->surfacePlugin->GetLastTotalTorque();
            }
            break;

        case PhaseState::BELOW:
            gzmsg << "[PhaseSwitchPlugin] 🤿 进入水下模式" << std::endl;
            

            SetNUUVModelsEnabled(true);
            

            SetNASVPluginsEnabled(false);
            
            break;
        case PhaseState::UNKNOWN:
            gzwarn << "[PhaseSwitchPlugin] 进入未知状态,保持当前配置" << std::endl;
            break;
    }
}


void PhaseSwitchPlugin::SuppressSurfaceForces()
{

  SetSurfacePluginMode(ForceMode::OFF);
}


void PhaseSwitchPlugin::SetSurfacePluginMode(ForceMode mode)
{
  if (!data->surfacePlugin) {
    return;
  }

  switch (mode)
  {
    case ForceMode::OFF:

      data->surfacePlugin->SetEnabled(false);
      break;

    case ForceMode::SURFACE_ONLY:

      data->surfacePlugin->SetEnabled(true);
      break;

    case ForceMode::DIRECT:

      data->surfacePlugin->SetEnabled(true);
      break;

    default:
      gzwarn << "[PhaseSwitchPlugin] Unknown ForceMode: " 
             << static_cast<int>(mode) << std::endl;
      break;
  }
}


PhaseSwitchPlugin::PhaseState
PhaseSwitchPlugin::ParsePhase(const std::string& name)
{
  if (name == "ABOVE") return PhaseState::ABOVE;
  if (name == "WATER_ENTRY") return PhaseState::WATER_ENTRY;
  if (name == "WATER_EXIT") return PhaseState::WATER_EXIT;
  if (name == "BELOW") return PhaseState::BELOW;
  return PhaseState::UNKNOWN;
}

std::string PhaseSwitchPlugin::PhaseToString(PhaseState s)
{
  switch (s)
  {
    case PhaseState::ABOVE: return "ABOVE";
    case PhaseState::WATER_ENTRY: return "WATER_ENTRY";
    case PhaseState::WATER_EXIT: return "WATER_EXIT";
    case PhaseState::BELOW: return "BELOW";
    default: return "SURFACE";
  }
}

void PhaseSwitchPlugin::SpinThread()
{
  ros::Rate r(50);
  while (data->rosSpin && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}
void PhaseSwitchPlugin::HandleAbove(const gazebo::common::Time& /*simTime*/)
{

    if (data->surfacePlugin)
    {
        SetSurfacePluginMode(ForceMode::OFF);
    }
    

    data->activeHydrodynamicPlugin = "None (Aerial)";
}


bool PhaseSwitchPlugin::QueryWaveProbe(double& waterZ, 
                                       std::string& phaseName, 
                                       double& zOverL) const
{
    if (!data || !data->phaseClient) {
        static bool warnedOnce = false;
        if (!warnedOnce) {
            gzwarn << "[PhaseSwitchPlugin] Service client not initialized!" << std::endl;
            warnedOnce = true;
        }
        waterZ = data->lastKnownWaterZ;
        phaseName = "UNKNOWN";
        zOverL = 0.0;
        return false;
    }
    

    nezha_plugins::GetPhaseSample srv;
    
    if (data->phaseClient.call(srv)) {
        waterZ = srv.response.surface_z;
        phaseName = srv.response.phase_name;
        zOverL = srv.response.z_over_l;
        

        data->lastKnownWaterZ = waterZ;
        

        if (data->serviceFailCount > 0) {
            gzmsg << "[PhaseSwitchPlugin] WaveProbe service recovered after " 
                  << data->serviceFailCount << " failures" << std::endl;
            data->serviceFailCount = 0;
        }
        
        return true;
    } else {

        data->serviceFailCount++;
        

        if (data->serviceFailCount % 100 == 1) {
            gzwarn << "[PhaseSwitchPlugin] Failed to call WaveProbe service (count: " 
                   << data->serviceFailCount << "). Using cached values." << std::endl;
        }

        waterZ = data->lastKnownWaterZ;
        phaseName = "UNKNOWN";
        zOverL = 0.0;
        return false;
    }
}
PhaseSwitchPlugin::PhaseState PhaseSwitchPlugin::MapWaveProbePhase(

    const std::string& waveProbePhase) const
{

    if (waveProbePhase == "ABOVE") {
        return PhaseState::ABOVE;
    } 
    else if (waveProbePhase == "WATER_ENTRY") {
        return PhaseState::WATER_ENTRY;
    } 
    else if (waveProbePhase == "SURFACE") {
        return PhaseState::SURFACE;
    } 
    else if (waveProbePhase == "WATER_EXIT") {
        return PhaseState::WATER_EXIT;
    } 
    else if (waveProbePhase == "BELOW") {
        return PhaseState::BELOW;
    }
    else {

        static std::set<std::string> warnedPhases;
        if (warnedPhases.find(waveProbePhase) == warnedPhases.end()) {
            gzwarn << "[PhaseSwitchPlugin] Unknown WaveProbe phase: '" 
                   << waveProbePhase << "'. Keeping current state: "
                   << StateToString(data->currentState) << std::endl;
            warnedPhases.insert(waveProbePhase);
        }
        return data->currentState;
    }
}

void PhaseSwitchPlugin::HandleSurface(const gazebo::common::Time& simTime)
{

    SetNASVPluginsEnabled(true);
    

    SetNUUVModelsEnabled(false);
    

    data->activeHydrodynamicPlugin = data->surfacePluginName + 
                                    " (" + data->surfacePluginFile + ")";
    

    if (data->surfacePlugin)
    {

        if (!data->surfacePlugin->IsEnabled())
        {
            data->surfacePlugin->SetEnabled(true);
            gzmsg << "[HandleSurface] Re-enabled surface plugin" << std::endl;
        }
        
    }
    

    static gazebo::common::Time lastDebugPrint;
    if ((simTime - lastDebugPrint).Double() > 1.0)
    {
        gzmsg << "╔════════════════════════════════════════╗" << std::endl;
        gzmsg << "║  SURFACE State Status                 ║" << std::endl;
        gzmsg << "╠════════════════════════════════════════╣" << std::endl;
        

        if (data->surfacePlugin)
        {
            bool enabled = data->surfacePlugin->IsEnabled();
            gzmsg << "║  NASV: " << std::setw(31) << std::left 
                  << (enabled ? "ENABLED ✓" : "DISABLED ❌") 
                  << " ║" << std::endl;
            

            auto force = data->surfacePlugin->GetLastTotalForce();
            gzmsg << "║  Force: (" 
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << force.X() << ", "
                  << std::setw(6) << force.Y() << ", "
                  << std::setw(6) << force.Z() << ") N  ║" << std::endl;
        }
        else
        {
            gzmsg << "║  NASV: NULL ❌                        ║" << std::endl;
        }
        

        if (data->fossenModel)
        {
            gzmsg << "║  NUUV: " << std::setw(31) << std::left 
                  << (data->fossenModel->IsEnabled() ? "ENABLED ❌" : "DISABLED ✓") 
                  << " ║" << std::endl;
        }
        
        gzmsg << "║  Active: " << std::setw(29) << std::left 
              << data->activeHydrodynamicPlugin << " ║" << std::endl;
        gzmsg << "╚════════════════════════════════════════╝" << std::endl;
        
        lastDebugPrint = simTime;
    }
}
void PhaseSwitchPlugin::HandleWaterEntry(const gazebo::common::Time& simTime)
{

    
    SetNASVPluginsEnabled(true);
    SetNUUVModelsEnabled(false);
    
    data->activeHydrodynamicPlugin = data->surfacePluginName + 
                                    " (" + data->surfacePluginFile + ")";
    

    static gazebo::common::Time lastDebugPrint;
    if ((simTime - lastDebugPrint).Double() > 1.0)
    {
        gzmsg << "╔════════════════════════════════════════╗" << std::endl;
        gzmsg << "║  WATER_ENTRY State Status             ║" << std::endl;
        gzmsg << "╠════════════════════════════════════════╣" << std::endl;
        
        if (data->surfacePlugin)
        {
            bool enabled = data->surfacePlugin->IsEnabled();
            gzmsg << "║  NASV: " << std::setw(31) << std::left 
                  << (enabled ? "ENABLED ✓" : "DISABLED ❌") 
                  << " ║" << std::endl;
        }
        
        gzmsg << "║  Active: " << std::setw(29) << std::left 
              << data->activeHydrodynamicPlugin << " ║" << std::endl;
        gzmsg << "╚════════════════════════════════════════╝" << std::endl;
        
        lastDebugPrint = simTime;
    }
}
void PhaseSwitchPlugin::HandleWaterExit(const gazebo::common::Time& simTime)
{

    
    SetNASVPluginsEnabled(true);
    SetNUUVModelsEnabled(false);
    
    data->activeHydrodynamicPlugin = data->surfacePluginName + 
                                    " (" + data->surfacePluginFile + ")";
    

    static gazebo::common::Time lastDebugPrint;
    if ((simTime - lastDebugPrint).Double() > 1.0)
    {
        gzmsg << "╔════════════════════════════════════════╗" << std::endl;
        gzmsg << "║  WATER_EXIT State Status              ║" << std::endl;
        gzmsg << "╠════════════════════════════════════════╣" << std::endl;
        
        if (data->surfacePlugin)
        {
            bool enabled = data->surfacePlugin->IsEnabled();
            gzmsg << "║  NASV: " << std::setw(31) << std::left 
                  << (enabled ? "ENABLED ✓" : "DISABLED ❌") 
                  << " ║" << std::endl;
            
            auto force = data->surfacePlugin->GetLastTotalForce();
            gzmsg << "║  Force: (" 
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << force.X() << ", "
                  << std::setw(6) << force.Y() << ", "
                  << std::setw(6) << force.Z() << ") N  ║" << std::endl;
        }
        
        gzmsg << "║  Active: " << std::setw(29) << std::left 
              << data->activeHydrodynamicPlugin << " ║" << std::endl;
        gzmsg << "╚════════════════════════════════════════╝" << std::endl;
        
        lastDebugPrint = simTime;
    }
}


void PhaseSwitchPlugin::TransitionTo(PhaseState newState, double simTime)
{
    if (newState == data->currentState) {
        return;
    }
    
    PhaseState oldState = data->currentState;
    data->currentState = newState;
    
    gzmsg << "[PhaseSwitchPlugin] 🔄 State transition: " 
          << StateToString(oldState) << " -> " << StateToString(newState) << std::endl;
    
    OnStateEnter(newState);

    if (newState == PhaseState::ABOVE || newState == PhaseState::BELOW) {
        data->previousStableState = newState;
    }
}



void PhaseSwitchPlugin::SetNASVPluginsEnabled(bool enable)
{

    if (data->surfacePlugin)
    {
        data->surfacePlugin->SetEnabled(enable);
        
        bool actualState = data->surfacePlugin->IsEnabled();
        
        if (actualState != enable)
        {
            gzwarn << "[SetNASVPluginsEnabled] Failed to " 
                   << (enable ? "enable" : "disable") 
                   << " surface plugin!" << std::endl;
            
        data->surfacePlugin->SetEnabled(enable);
        if (enable)
        {

            data->surfacePlugin->Init();
            
            gzmsg << "[SetNASVPluginsEnabled] Plugin re-initialized" << std::endl;
        }
            actualState = data->surfacePlugin->IsEnabled();
            
            if (actualState == enable)
            {
                gzmsg << "[SetNASVPluginsEnabled] Retry successful!" << std::endl;
            }
        }
        else
        {
        }
        

        if (enable && actualState)
        {

            static gazebo::common::Time lastCheck;
            if ((gazebo::common::Time::GetWallTime() - lastCheck).Double() > 0.1)
            {
                auto force = data->surfacePlugin->GetLastTotalForce();
                if (force.Length() < 1e-6)
                {
                    gzwarn << "[SetNASVPluginsEnabled] Plugin enabled but force is zero!" 
                           << std::endl;
                }
                lastCheck = gazebo::common::Time::GetWallTime();
            }
        }
    }
    else
    {
        static bool warnedOnce = false;
        if (!warnedOnce)
        {
            gzwarn << "[SetNASVPluginsEnabled] Surface plugin is NULL!" << std::endl;
            warnedOnce = true;
        }
    }
    

    auto allPlugins = asv::HydrodynamicsPluginRegistry::GetInstance().GetAllPlugins();
    
    int successCount = 0;
    for (auto* plugin : allPlugins)
    {
        if (plugin)
        {
            plugin->SetEnabled(enable);
            if (plugin->IsEnabled() == enable)
            {
                successCount++;
            }
        }
    }
    
}



void PhaseSwitchPlugin::SetNUUVModelsEnabled(bool enable)
{

    auto models = gazebo::HydrodynamicModelRegistry::GetInstance().GetModels();
    
    if (models.empty())
    {
        static bool warnedOnce = false;
        if (!warnedOnce)
        {
            gzwarn << "[SetNUUVModelsEnabled] Registry is empty!" << std::endl;
            warnedOnce = true;
        }
        return;
    }
    
    int successCount = 0;
    int failCount = 0;
    
    for (auto* model : models)
    {
        if (!model)
        {
            failCount++;
            continue;
        }
        

        model->SetEnabled(enable);
        

        bool actualState = model->IsEnabled();
        
        if (actualState == enable)
        {
            successCount++;
        }
        else
        {
            failCount++;
            gzwarn << "[SetNUUVModelsEnabled] Failed to " 
                   << (enable ? "enable" : "disable") 
                   << " model at " << model << std::endl;
        }
    }
    
  
}




std::string PhaseSwitchPlugin::StateToString(PhaseState state) const
{
    switch (state)
    {
        case PhaseState::ABOVE:       return "ABOVE";
        case PhaseState::WATER_ENTRY: return "WATER_ENTRY";
        case PhaseState::SURFACE:     return "SURFACE";
        case PhaseState::WATER_EXIT:  return "WATER_EXIT";
        case PhaseState::BELOW:       return "BELOW";
        case PhaseState::UNKNOWN:     return "UNKNOWN";
        default:                      return "INVALID";
    }
}
void PhaseSwitchPlugin::FindAndCachePlugins()
{
    gzmsg << "[PhaseSwitchPlugin] 🔍 FindAndCachePlugins called" << std::endl;
    

    if (!data->surfacePlugin)
    {
        data->surfacePlugin = asv::HydrodynamicsPluginRegistry::GetInstance()
                                  .FindByModel(data->model);
        
        if (data->surfacePlugin)
        {
            gzmsg << "  ✓ Found NASV plugin via singleton for model: " 
                  << data->model->GetName() << std::endl;
        }
        else
        {
            gzwarn << "  ✗ NASV plugin not found in singleton registry" << std::endl;
            

            std::lock_guard<std::mutex> lock(asv::g_nasvRegistryMutex);
            gzmsg << "  Fallback: Searching global registry (" 
                  << asv::g_nasvPluginRegistry.size() << " plugins)..." << std::endl;
            
            for (auto* plugin : asv::g_nasvPluginRegistry)
            {
                if (!plugin) continue;
                
                auto pluginModel = plugin->GetModel();
                if (!pluginModel) continue;
                
                gzmsg << "    Checking: " << pluginModel->GetName() << std::endl;
                
                if (pluginModel == data->model)
                {
                    data->surfacePlugin = plugin;
                    gzmsg << "    ✓ Matched by model pointer" << std::endl;
                    break;
                }
            }
        }
    }
    

    if (!data->fossenModel && data->targetLink)
    {
        auto models = gazebo::HydrodynamicModelRegistry::GetInstance().GetModels();
        
        
        std::string targetScoped = data->targetLink->GetScopedName();
        
        for (auto* model : models)
        {
            if (!model) continue;
            
            auto link = model->GetLink();
            if (!link) continue;
            
            if (link->GetScopedName() == targetScoped)
            {
                data->fossenModel = dynamic_cast<gazebo::HMFossen*>(model);
                if (data->fossenModel)
                {
                    gzmsg << "  ✓ Found matching NUUV model: " 
                          << data->fossenModel->GetName() << std::endl;
                    break;
                }
            }
        }
    }
    

    gzmsg << "╔════════════════════════════════════════╗" << std::endl;
    gzmsg << "║  Plugin Search Results                ║" << std::endl;
    gzmsg << "╠════════════════════════════════════════╣" << std::endl;
    gzmsg << "║  NASV Plugin:  " << std::setw(22) << std::left 
          << (data->surfacePlugin ? "✓ FOUND" : "✗ NOT FOUND") << " ║" << std::endl;
    gzmsg << "║  NUUV Model:   " << std::setw(22) << std::left 
          << (data->fossenModel ? "✓ FOUND" : "✗ NOT FOUND") << " ║" << std::endl;
    gzmsg << "╚════════════════════════════════════════╝" << std::endl;
}



GZ_REGISTER_MODEL_PLUGIN(PhaseSwitchPlugin)
} 

