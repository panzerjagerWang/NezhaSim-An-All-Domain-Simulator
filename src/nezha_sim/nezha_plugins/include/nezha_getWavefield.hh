#ifndef ASV_GET_WAVEFIELD_HH_
#define ASV_GET_WAVEFIELD_HH_

#include <gazebo/physics/World.hh>
#include <memory>
#include <string>

namespace asv {

class Wavefield;


std::shared_ptr<const Wavefield> GetWavefield(gazebo::physics::WorldPtr world);


void RegisterWavefield(const std::string& name, std::shared_ptr<const Wavefield> wavefield);


void UnregisterWavefield(const std::string& name);

} 

#endif

