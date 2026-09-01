//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
#ifndef NEZHA_UNIFIED_HYDRODYNAMICS_PLUGIN_HH
#define NEZHA_UNIFIED_HYDRODYNAMICS_PLUGIN_HH

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>
#include <cmath> // For std::pow, std::max
#include <nezha_plugins/HydrodynamicsForces.h> 
// asv engine types used as members below
#include "nasv_physics.hh"                                  // asv::Hydrodynamics, asv::HydrodynamicsParameters
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"         // asv::Mesh, asv::Point3, asv::Vector3
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"         // asv::Wavefield, asv::WavefieldSampler
#include "asv_wave_sim_gazebo_plugins/Grid.hh"              // asv::Grid
// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/service_client.h>

// Ignition & Eigen
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ASV Wave Sim (Required for asv::Mesh)
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"

// Forward Declarations
namespace asv {
    class Wavefield;
}

namespace nezha {

    enum class PhaseState {
        ABOVE,          // Aerial
        WATER_ENTRY,    // Transitioning In
        SURFACE,        // Surface Operations
        WATER_EXIT,     // Transitioning Out
        BELOW           // Fully Submerged
    };

    // --- 1. PhaseManager Definition (Moved here from .cc) ---
    struct PhaseManager {
        PhaseState currentPhase = PhaseState::SURFACE;
        double zThresholdHigh = 0.5;
        double zThresholdLow = -0.5;
        double transitionAlpha = 0.0;

        // Logic to update phase based on water level
        void Update(double waterZ, double robotZ, double volume) {
            double L = std::pow(volume, 1.0/3.0); 
            if (L < 0.1) L = 1.0; 
            
            double ratio = (robotZ - waterZ) / L;
            PhaseState nextPhase = this->currentPhase;

            switch (this->currentPhase) {
                case PhaseState::ABOVE:
                    if (ratio < this->zThresholdHigh) nextPhase = PhaseState::WATER_ENTRY;
                    break;
                case PhaseState::WATER_ENTRY:
                    if (ratio < 0.0) nextPhase = PhaseState::SURFACE;
                    else if (ratio > this->zThresholdHigh) nextPhase = PhaseState::ABOVE;
                    break;
                case PhaseState::SURFACE:
                    if (ratio < this->zThresholdLow) nextPhase = PhaseState::WATER_EXIT; 
                    else if (ratio > this->zThresholdHigh) nextPhase = PhaseState::WATER_ENTRY; 
                    break;
                case PhaseState::WATER_EXIT: 
                    if (ratio < this->zThresholdLow - 0.2) nextPhase = PhaseState::BELOW;
                    else if (ratio > this->zThresholdLow) nextPhase = PhaseState::SURFACE;
                    break;
                case PhaseState::BELOW:
                    if (ratio > this->zThresholdLow) nextPhase = PhaseState::WATER_EXIT;
                    break;
            }

            if (this->currentPhase == PhaseState::WATER_ENTRY || this->currentPhase == PhaseState::WATER_EXIT) {
                double range = this->zThresholdHigh - this->zThresholdLow;
                this->transitionAlpha = 1.0 - std::max(0.0, std::min(1.0, (ratio - this->zThresholdLow) / range));
            } else if (this->currentPhase == PhaseState::BELOW) {
                this->transitionAlpha = 1.0;
            } else {
                this->transitionAlpha = 0.0;
            }
            this->currentPhase = nextPhase;
        }
    };

    struct FossenParams {
        Eigen::Matrix<double, 6, 6> Ma;
        Eigen::Matrix<double, 6, 6> DLin;
        Eigen::Matrix<double, 6, 6> DNonLin;
        double volume;
        double fluidDensity;

        FossenParams() {
            Ma.setZero();
            DLin.setZero();
            DNonLin.setZero();
            volume = 0.0;
            fluidDensity = 1028.0;
        }
    };

    class NezhaUnifiedHydrodynamicsPlugin : public gazebo::ModelPlugin {
    public:
        NezhaUnifiedHydrodynamicsPlugin();
        virtual ~NezhaUnifiedHydrodynamicsPlugin();
        
        // Member variables now have valid types
        PhaseManager phaseManager;
       std::string waveModelName;
double bodyHalfHeight = 0.1;
double heaveDamping = 8.0;    // linear vertical damping (under-critical, ~0.4*c_crit)
double heaveDragQuad = 8.0;   // quadratic vertical entry drag (0.5*rho*Cd*A lumped)

// Wave-kinematics coupling (orbital following + surface-slope alignment).
double waveDecayLength = 1.0; // depth [m] over which wave orbital motion decays
double waveAlignGain   = 2.0; // torque gain aligning body-up to the wave normal
// Previous-frame surface sample, for the wave vertical water velocity dEta/dt.
double prevSurfaceZ    = 0.0;
double prevSurfaceTime = 0.0;
bool   havePrevSurface = false;
std::shared_ptr<asv::HydrodynamicsParameters> hydroParams;
std::shared_ptr<const asv::Wavefield> wavefield;          // note: const
std::shared_ptr<asv::WavefieldSampler> wavefieldSampler;
std::vector<std::shared_ptr<asv::Mesh>> initLinkMeshes;
std::vector<std::shared_ptr<asv::Mesh>> linkMeshes;
std::vector<std::shared_ptr<asv::Hydrodynamics>> hydrodynamics;
    // ROS Service Variables
    ros::ServiceServer forcesService;
    ros::CallbackQueue rosQueue;

    // Force Tracking Variables
    ignition::math::Vector3d lastBuoyancyForce{0,0,0};
    ignition::math::Vector3d lastWaveForce{0,0,0};
    ignition::math::Vector3d lastDampingForce{0,0,0};
    ignition::math::Vector3d lastCoriolisForce{0,0,0};
    // Heave (vertical) damping + quadratic water-entry drag, WORLD frame. Tracked
    // separately so the reported resultant matches the actually-applied force.
    ignition::math::Vector3d lastHeaveForce{0,0,0};
    double lastSubmersionRatio{0.0};
    double lastWaveHeight{0.0};

    // Service Callbacks
    bool OnGetForcesService(
        nezha_plugins::HydrodynamicsForces::Request &req,
        nezha_plugins::HydrodynamicsForces::Response &res);
    void QueueThread();


protected:
    // --- Core Gazebo Callbacks ---
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void OnUpdate(const gazebo::common::UpdateInfo& _info);

    // --- Physics Engines ---
    void ComputeForces(const gazebo::common::UpdateInfo& _info);
    void ComputeHydrodynamicForces(const gazebo::common::UpdateInfo& _info);

    // --- Initialization ---
    void InitRos();
    void InitPhysics(sdf::ElementPtr _sdf);
    void InitWavefield(sdf::ElementPtr _sdf);
    void CreateCollisionMeshes();

    // --- Helpers ---
    std::vector<double> Str2Vector(const std::string& input);

private:
    // --- Member Variables ---
    // (NO FUNCTION DECLARATIONS HERE)

    gazebo::physics::ModelPtr model;
    gazebo::physics::LinkPtr targetLink;
    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr updateConnection;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceClient phaseClient;
    std::thread rosQueueThread;
    bool rosRunning;

    
    // Fossen parameters
    FossenParams fossen;
    
    // State variables
    Eigen::Matrix<double, 6, 1> lastVelRel;
    Eigen::Matrix<double, 6, 1> filteredAcc;
    double lastTime;
}; // End of Class




} // namespace nezha

#endif // NEZHA_UNIFIED_HYDRODYNAMICS_PLUGIN_HH

