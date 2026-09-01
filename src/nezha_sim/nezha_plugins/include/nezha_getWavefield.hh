//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
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

