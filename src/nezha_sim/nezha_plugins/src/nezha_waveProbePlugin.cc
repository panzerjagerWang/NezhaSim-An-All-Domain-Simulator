#include "nezha_waveProbePlugin.hh"
#include "nasv_waveField.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include <gazebo/msgs/msgs.hh>
#include "nezha_getWavefield.hh"   
#include <nezha_plugins/GetPhaseSample.h>


namespace asv {
using gazebo::physics::ModelPtr;
using gazebo::physics::WorldPtr;
WaveProbePlugin::WaveProbePlugin() : data(new Data) {}
WaveProbePlugin::~WaveProbePlugin() {}
void WaveProbePlugin::Load(ModelPtr _model, sdf::ElementPtr _sdf)
{
  data->model = _model;
  data->world = _model->GetWorld();
  

  if (_sdf->HasElement("robot_namespace")) {
    data->robotNamespace = _sdf->Get<std::string>("robot_namespace");
  } 

  else if (_model) {
    data->robotNamespace = _model->GetName();
  }
  

  if (!data->robotNamespace.empty() && data->robotNamespace[0] != '/') {
    data->robotNamespace = "/" + data->robotNamespace;
  }
  if (data->robotNamespace.back() == '/') {
    data->robotNamespace.pop_back();
  }
  
  gzdbg << "[WaveProbePlugin] Robot namespace: " << data->robotNamespace << "\n";
  

  std::string wavefieldName = "wavefield";
  if (_sdf->HasElement("wavefield_name"))
    wavefieldName = _sdf->Get<std::string>("wavefield_name");

  std::vector<std::string> candidates = {
    wavefieldName,
    "nezha_ocean_waves::wavefield_entity",
    "wavefield_entity",
    "wavefield",
    data->world->Name() + std::string("::wavefield"),
    data->world->Name() + std::string("::wavefield_entity"),
    data->world->Name() + std::string("::nezha_ocean_waves::wavefield_entity")
  };

  const std::string modelName = data->model ? data->model->GetName() : "";
  if (!modelName.empty())
  {
    std::vector<std::string> modelScoped = {
      modelName + std::string("::") + wavefieldName,
      modelName + std::string("::wavefield_entity"),
      modelName + std::string("::wavefield"),
      modelName + std::string("::nezha_ocean_waves::wavefield_entity")
    };
    candidates.insert(candidates.end(), modelScoped.begin(), modelScoped.end());
  }

  data->wfCandidates = candidates;


  data->firstBindTried = true;
if (!BindWavefieldOnce()) {
  data->updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&WaveProbePlugin::OnUpdate, this, std::placeholders::_1));
  gzdbg << "[WaveProbePlugin] WavefieldEntity not yet available. Will retry in OnUpdate...\n";
} else {

  gzmsg << "[WaveProbePlugin] ✓ Wavefield bound immediately on Load.\n";
}


  if (!ros::isInitialized()) {
    int argc = 0; 
    char** argv = nullptr;
    ros::init(argc, argv, "wave_probe_plugin", ros::init_options::NoSigintHandler);
  }
  

  data->nh = std::make_unique<ros::NodeHandle>(data->robotNamespace);

  data->nh->param("characteristic_len", data->L, 1.0);
  if (_sdf->HasElement("characteristic_len"))
    data->L = _sdf->Get<double>("characteristic_len");
  
  double zTopParam;
  if (data->nh->getParam("z_top_offset", zTopParam))
    data->zTopOffset = zTopParam;
  if (_sdf->HasElement("z_top_offset"))
    data->zTopOffset = _sdf->Get<double>("z_top_offset");
  if (std::isnan(data->zTopOffset))
    data->zTopOffset = 0.5 * data->L;
  

  data->t_last = data->world->SimTime();
  
  std::string serviceName = "transmedia/get_phase_sample"; 
  
  data->srvPhase = data->nh->advertiseService(
      serviceName,
      &WaveProbePlugin::SrvGetPhaseSample, this);
  
  gzdbg << "[WaveProbePlugin] Service ready: " 
        << data->robotNamespace << "/" << serviceName << "\n"
        << "  characteristic_len: " << data->L << " m\n"
        << "  z_top_offset      : " << data->zTopOffset << " m\n";
}

bool WaveProbePlugin::SrvGetPhaseSample(
  nezha_plugins::GetPhaseSample::Request& req,
  nezha_plugins::GetPhaseSample::Response& res)
{

  if (!data || !data->model) {
    ROS_ERROR("[WaveProbe] Model not available!");
    return false;
  }

  auto pose = data->model->WorldPose();
  double x = pose.Pos().X();
  double y = pose.Pos().Y();
  double z = pose.Pos().Z();

  auto vel = data->model->WorldLinearVel();
  double vz = vel.Z();


  double zSurf = QuerySurfaceZ(x, y);
  res.surface_z = zSurf;

  double zRel = z - zSurf;  
  double L = data->L;


  double zOverL_signed = zRel / L;


  const double threshold_above = 0;   
  const double threshold_below = 1.0;   
  const double vel_threshold = 0.2;     


  bool in_surface_zone = false;
  if (zOverL_signed > 0) {

    in_surface_zone = (zOverL_signed < threshold_above);
  } else {
 
    in_surface_zone = (std::abs(zOverL_signed) < threshold_below);
  }


  if (in_surface_zone) {

    if (std::abs(vz) < vel_threshold) {
      res.phase_name = "SURFACE";        
    } 
    else if (vz < -vel_threshold) {
      res.phase_name = "WATER_ENTRY";    
    } 
    else {
      res.phase_name = "WATER_EXIT";     
    }
    
  } else {

    if (zOverL_signed > 0) {
      res.phase_name = "ABOVE";         
    } else {
      res.phase_name = "BELOW";          
    }
  }

  res.z_over_l = std::clamp(std::abs(zOverL_signed), 0.0, 4.0);


  ROS_DEBUG_THROTTLE(0.5, 
    "[WaveProbe] z=%.2f surf=%.2f zRel=%.3f vz=%.2f | z/L=%.3f(%.3f) Phase=%s",
    z, zSurf, zRel, vz, zOverL_signed, res.z_over_l, res.phase_name.c_str());

  return true;
}

double WaveProbePlugin::QuerySurfaceZ(double x, double y) const
{
  if (!data || !data->world) {
    gzwarn << "[WaveProbePlugin::QuerySurfaceZ] Invalid world pointer!\n";
    return 0.0;
  }


  double simTime = data->world->SimTime().Double();


  if (auto wf = asv::GetWavefield(data->world)) {

    double z = wf->SurfaceElevation(x, y, simTime);
    return z;
  }

  if (data->wavefield) {
    if (auto wf2 = data->wavefield->GetWavefield()) {
      return wf2->SurfaceElevation(x, y, simTime); 
    }
  }

  static bool warnedOnce = false;
  if (!warnedOnce) {
    gzwarn << "[WaveProbePlugin::QuerySurfaceZ] No wavefield available! "
           << "Returning z=0.0\n";
    warnedOnce = true;
  }
  return 0.0;
}





bool WaveProbePlugin::BindWavefieldOnce()
{
  if (!data || !data->world) return false;
  if (data->waveBound) return true; 

#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::physics::ModelPtr waveModel = data->world->ModelByName("nezha_ocean_waves");
#else
  gazebo::physics::ModelPtr waveModel;
  {

    auto models = data->world->GetModels();
    for (auto const& m : models) {
      if (m->GetName() == "nezha_ocean_waves") { waveModel = m; break; }
    }
  }
#endif


  if (!waveModel) {
#if GAZEBO_MAJOR_VERSION >= 9
    for (auto const& m : data->world->Models()) {
      const auto& n = m->GetName();
      if (n.find("ocean_waves") != std::string::npos || n.find("wave") != std::string::npos) {
        waveModel = m;
        break;
      }
    }
#else
    auto models = data->world->GetModels();
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

    const auto now = data->world->SimTime();
    if ((now - data->lastPrint).Double() > 1.0) {
      gzerr << "[WaveProbePlugin] Cannot bind wavefield: model 'nezha_ocean_waves' not found.\n";
#if GAZEBO_MAJOR_VERSION >= 9
      gzerr << "[WaveProbePlugin] Existing models:";
      for (auto const& m : data->world->Models()) gzerr << " '" << m->GetName() << "'";
      gzerr << "\n";
#else
      gzerr << "[WaveProbePlugin] Existing models:";
      auto models = data->world->GetModels();
      for (auto const& m : models) gzerr << " '" << m->GetName() << "'";
      gzerr << "\n";
#endif
      data->lastPrint = now;
    }
    return false;
  }


  gazebo::physics::LinkPtr waveLink;
  auto links = waveModel->GetLinks();
  for (auto const& l : links) {
    if (l->GetName() == "ocean_waves_link") { waveLink = l; break; }
  }

  if (!waveLink && !links.empty()) waveLink = links.front();

  data->waveModel = waveModel;
  data->waveLink  = waveLink;


  data->waveBound = (data->waveModel != nullptr); 
  if (data->waveBound) {
    gzmsg << "[WaveProbePlugin] Bound wavefield via model '"
          << data->waveModel->GetName() << "' link '"
          << (data->waveLink ? data->waveLink->GetName() : "(none)") << "'\n";
  }

  return data->waveBound;
}

void WaveProbePlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  if (!data || !data->world) return;

  if (!data->waveBound) {
    if (BindWavefieldOnce()) {
      if (data->updateConn) {
        data->updateConn.reset();
        gzmsg << "[WaveProbePlugin] ✓ Wavefield bound successfully. "
              << "OnUpdate callback disconnected.\\n";
      }
      return;
    }
    

    const auto now = data->world->SimTime();
    if ((now - data->lastPrint).Double() > 1.0) {
      gzerr << "[WaveProbePlugin] ⏳ Waiting for wave model 'nezha_ocean_waves'...\\n";
      data->lastPrint = now;
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(WaveProbePlugin)

} 
