#include "nezha_getWavefield.hh"
#include "nezha_surfacePlugin.hh"
#include "nezha_underwaterHydrodynamicsPlugin.hh"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/common/Console.hh>

#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "nasv_waveField.hh"

#include <map>
#include <mutex>

namespace asv {

static std::map<std::string, std::shared_ptr<const Wavefield>> g_wavefieldRegistry;
static std::mutex g_registryMutex;

void RegisterWavefield(const std::string& name, std::shared_ptr<const Wavefield> wavefield)
{
  std::lock_guard<std::mutex> lock(g_registryMutex);
  if (wavefield) {
    g_wavefieldRegistry[name] = wavefield;
    gzmsg << "[WavefieldRegistry] Registered wavefield: '" << name << "'\n";
  }
}

void UnregisterWavefield(const std::string& name)
{
  std::lock_guard<std::mutex> lock(g_registryMutex);
  auto it = g_wavefieldRegistry.find(name);
  if (it != g_wavefieldRegistry.end()) {
    g_wavefieldRegistry.erase(it);
    gzmsg << "[WavefieldRegistry] Unregistered wavefield: '" << name << "'\n";
  }
}

std::shared_ptr<const Wavefield> GetWavefield(gazebo::physics::WorldPtr world)
{
  if (!world) {
    gzwarn << "[GetWavefield] World pointer is null\n";
    return nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(g_registryMutex);
    if (!g_wavefieldRegistry.empty()) {
      return g_wavefieldRegistry.begin()->second;
    }
  }

  std::vector<std::string> candidates = {
    "nezha_ocean_waves::wavefield_entity",
    "ocean_waves::wavefield_entity",
    "wavefield_entity",
    "wavefield"
  };

  gzdbg << "[GetWavefield] Searching for WavefieldEntity in world '"
        << world->Name() << "'\n";

  for (const auto& name : candidates) {
    gzdbg << "[GetWavefield] Trying entity name: '" << name << "'\n";
    
#if GAZEBO_MAJOR_VERSION >= 9
    auto entity = world->EntityByName(name);
#else
    auto entity = world->GetEntity(name);
#endif
    
    if (entity) {
      WavefieldEntity* wfEntityPtr = dynamic_cast<WavefieldEntity*>(entity.get());
      if (wfEntityPtr) {
        gzmsg << "[GetWavefield] Found WavefieldEntity: '" << name << "'\n";
        return wfEntityPtr->GetWavefield();
      }
    }
  }

  gzwarn << "[GetWavefield] No wavefield found in world '" << world->Name() << "'\n";
  gzwarn << "[GetWavefield] Make sure WavefieldModelPlugin is loaded and initialized\n";
  return nullptr;
}

} 

