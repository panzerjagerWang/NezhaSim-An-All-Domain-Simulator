//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
#include "nezha_unified_plugin.hh"

// Standard & Gazebo Includes
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>

// ASV / Wave Includes
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"

// ROS Includes
#include <nezha_plugins/GetPhaseSample.h>
#include <ros/ros.h>

#include <algorithm>

using namespace gazebo;

// ==========================================================================
// STATIC HELPER FUNCTIONS
// ==========================================================================
namespace {
    void CreateTessellatedBox(const std::string& name, const ignition::math::Vector3d& size, const ignition::math::Vector3i& segments) {
        gazebo::common::MeshManager *meshMgr = gazebo::common::MeshManager::Instance();
        if (meshMgr->HasMesh(name)) return;

        gazebo::common::Mesh *mesh = new gazebo::common::Mesh();
        mesh->SetName(name);
        gazebo::common::SubMesh *subMesh = new gazebo::common::SubMesh();
        mesh->AddSubMesh(subMesh);

        double dx = size.X() / 2.0;
        double dy = size.Y() / 2.0;
        double dz = size.Z() / 2.0;

        struct Face { int u_ax; int v_ax; int w_ax; double w_dir; int u_seg; int v_seg; };
        std::vector<Face> faces = {
            {1, 2, 0,  1.0, segments.Y(), segments.Z()},
            {1, 2, 0, -1.0, segments.Y(), segments.Z()},
            {0, 2, 1,  1.0, segments.X(), segments.Z()},
            {0, 2, 1, -1.0, segments.X(), segments.Z()},
            {0, 1, 2,  1.0, segments.X(), segments.Y()},
            {0, 1, 2, -1.0, segments.X(), segments.Y()}
        };

        int vertexOffset = 0;
        for (const auto& face : faces) {
            double w_val = face.w_dir * (face.w_ax == 0 ? dx : (face.w_ax == 1 ? dy : dz));
            double u_len = (face.u_ax == 0 ? size.X() : (face.u_ax == 1 ? size.Y() : size.Z()));
            double v_len = (face.v_ax == 0 ? size.X() : (face.v_ax == 1 ? size.Y() : size.Z()));
            double u_step = u_len / face.u_seg;
            double v_step = v_len / face.v_seg;

            for (int i = 0; i <= face.u_seg; ++i) {
                for (int j = 0; j <= face.v_seg; ++j) {
                    double u = -u_len / 2.0 + i * u_step;
                    double v = -v_len / 2.0 + j * v_step;
                    double coords[3];
                    coords[face.u_ax] = u; coords[face.v_ax] = v; coords[face.w_ax] = w_val;
                    subMesh->AddVertex(coords[0], coords[1], coords[2]);
                }
            }
            for (int i = 0; i < face.u_seg; ++i) {
                for (int j = 0; j < face.v_seg; ++j) {
                    int row = face.v_seg + 1;
                    int v0 = vertexOffset + i * row + j;
                    int v1 = vertexOffset + i * row + (j + 1);
                    int v2 = vertexOffset + (i + 1) * row + j;
                    int v3 = vertexOffset + (i + 1) * row + (j + 1);
                    if (face.w_dir > 0) {
                        subMesh->AddIndex(v0); subMesh->AddIndex(v2); subMesh->AddIndex(v1);
                        subMesh->AddIndex(v1); subMesh->AddIndex(v2); subMesh->AddIndex(v3);
                    } else {
                        subMesh->AddIndex(v0); subMesh->AddIndex(v1); subMesh->AddIndex(v2);
                        subMesh->AddIndex(v1); subMesh->AddIndex(v3); subMesh->AddIndex(v2);
                    }
                }
            }
            vertexOffset = subMesh->GetVertexCount();
        }
        meshMgr->AddMesh(mesh);
    }

    // Re-pose a body-frame surface mesh into the world frame.
    // Required every step, otherwise the hydrodynamics engine sees a static
    // mesh sitting at the origin and never detects the waterline.
    void ApplyPoseToMesh(const ignition::math::Pose3d& _pose,
                         const asv::Mesh& _source,
                         asv::Mesh& _target) {
        auto srcIt = std::begin(_source.vertices());
        auto tgtIt = std::begin(_target.vertices());
        for (; srcIt != std::end(_source.vertices()) &&
               tgtIt != std::end(_target.vertices());
             ++srcIt, ++tgtIt) {
            const asv::Point3& p0 = _source.point(*srcIt);
            ignition::math::Vector3d ignP0(
                CGAL::to_double(p0.x()),
                CGAL::to_double(p0.y()),
                CGAL::to_double(p0.z()));
            ignition::math::Vector3d ignP1 =
                _pose.Rot().RotateVector(ignP0) + _pose.Pos();
            _target.point(*tgtIt) = asv::Point3(ignP1.X(), ignP1.Y(), ignP1.Z());
        }
    }
}

namespace nezha {

// ==========================================================================
// CONSTRUCTOR / DESTRUCTOR
// ==========================================================================
NezhaUnifiedHydrodynamicsPlugin::NezhaUnifiedHydrodynamicsPlugin() : ModelPlugin() {
    this->fossen.DLin.setZero();
    this->fossen.DNonLin.setZero();
    this->fossen.volume = 0.001;
    this->lastTime = 0.0;
}

NezhaUnifiedHydrodynamicsPlugin::~NezhaUnifiedHydrodynamicsPlugin() {
    this->rosRunning = false;
    if (this->rosNode) {
        this->rosNode->shutdown();
    }
    if (this->rosQueueThread.joinable()) {
        this->rosQueueThread.join();
    }
}

// ==========================================================================
// LOAD
// ==========================================================================
void NezhaUnifiedHydrodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;
    this->world = _model->GetWorld();

    // 1. Link Resolution
    if (_sdf->HasElement("link_name")) {
        std::string linkName = _sdf->Get<std::string>("link_name");
        this->targetLink = _model->GetLink(linkName);
    }
    if (!this->targetLink) {
        auto links = _model->GetLinks();
        if (!links.empty()) this->targetLink = links[0];
    }
    if (!this->targetLink) {
        gzerr << "[NezhaUnified] No valid link found!" << std::endl;
        return;
    }

    // 2. Mesh-engine hydrodynamics parameters (drag / damping / buoyancy scale).
    //    Read straight from the <plugin> block. The drag/damping coeffs sit at
    //    the TOP LEVEL; robot_volume / buoyancy_scale / force_scale_factor are
    //    separate top-level elements applied via dedicated setters.
    this->hydroParams = std::make_shared<asv::HydrodynamicsParameters>();

    // 2a. Top-level drag/damping params (cDampL1 .. cPDrag2, *_on flags, vRDrag).
    this->hydroParams->SetFromSDF(*_sdf);

    // 2b. buoyancy_scale (default 1.0)
    double buoyancyScale = 1.0;
    if (_sdf->HasElement("buoyancy_scale")) {
        buoyancyScale = _sdf->Get<double>("buoyancy_scale");
    }
    this->hydroParams->SetBuoyancyScale(buoyancyScale);

    // 2c. force_scale_factor (default 1.0)
    double forceScaleFactor = 1.0;
    if (_sdf->HasElement("force_scale_factor")) {
        forceScaleFactor = _sdf->Get<double>("force_scale_factor");
    }
    this->hydroParams->SetForceScaleFactor(forceScaleFactor);

    // 2d. robot_volume -> max displaced volume cap (0 = disabled).
    //     THIS is what was missing -> log showed "Max Volume: 0.0000".
    //     Fallback: if <robot_volume> is absent, use the Fossen <volume> read in
    //     InitPhysics so the cap is never left at zero by accident.
    double robotVolume = 0.0;
    if (_sdf->HasElement("robot_volume")) {
        robotVolume = _sdf->Get<double>("robot_volume");
    }
    if (robotVolume <= 0.0) {
        // InitPhysics runs after this, so read <hydrodynamic_model><volume>
        // directly here as a fallback source for the cap.
        sdf::ElementPtr hm = _sdf;
        if (_sdf->HasElement("hydrodynamic_model"))
            hm = _sdf->GetElement("hydrodynamic_model");
        if (hm->HasElement("volume")) {
            robotVolume = hm->Get<double>("volume");
        }
    }
    if (robotVolume > 0.0) {
        this->hydroParams->SetMaxDisplacedVolume(robotVolume);
    } else {
        gzwarn << "[NezhaUnified] robot_volume not found anywhere; "
                  "mesh buoyancy will NOT be volume-capped (Max Volume = 0)."
               << std::endl;
    }

    // 2e. Heave (vertical) damping coefficient. This is the term that kills
    //     the bobbing/bouncing at the surface. It is applied in world Z,
    //     proportional to vertical velocity, and is NOT scaled down to zero at
    //     the surface (unlike the Fossen damping). Default tuned for a small
    //     ~1 kg robot; raise it if the robot still bobs, lower it if it feels
    //     sluggish / stuck to the surface.
    this->heaveDamping = 10.0;
    if (_sdf->HasElement("heave_damping")) {
        this->heaveDamping = _sdf->Get<double>("heave_damping");
    }

    // 2f. Quadratic vertical entry drag (0.5*rho*Cd*A lumped). This is the v^2
    //     term that dissipates water-entry kinetic energy so a dropped body
    //     decelerates and settles instead of bouncing back out. Linear
    //     heave_damping alone is too weak at impact speed.
    if (_sdf->HasElement("heave_drag_quad")) {
        this->heaveDragQuad = _sdf->Get<double>("heave_drag_quad");
    }

    // 2g. Wave-kinematics coupling.
    //   wave_decay_length: depth over which the wave's orbital water motion
    //     decays (e^(-depth/L)); makes the underwater following fade with depth.
    //   wave_align_gain:   strength of the torque that tilts the body's up-axis
    //     to the local wave-surface normal (so it lands on / floats along the
    //     wave face instead of flat). 0 disables it.
    if (_sdf->HasElement("wave_decay_length")) {
        this->waveDecayLength = _sdf->Get<double>("wave_decay_length");
    }
    if (_sdf->HasElement("wave_align_gain")) {
        this->waveAlignGain = _sdf->Get<double>("wave_align_gain");
    }

    // 3. Initialize Sub-Systems
    //    Order matters: wavefield before meshes (the sampler needs it).
    this->InitPhysics(_sdf);       // Load Fossen params
    this->InitWavefield(_sdf);     // Connect to Ocean
    this->CreateCollisionMeshes(); // Build init/work meshes + sampler + hydrodynamics
    this->InitRos();               // Connect to ROS

    // 4. Phase Manager Setup
    this->phaseManager.zThresholdHigh = _sdf->Get<double>("threshold_high", 0.05).first;
    this->phaseManager.zThresholdLow = _sdf->Get<double>("threshold_low", -0.05).first;
    this->phaseManager.currentPhase = PhaseState::SURFACE;

    // 5. Connect Update Loop
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&NezhaUnifiedHydrodynamicsPlugin::OnUpdate, this, std::placeholders::_1));
}

// ==========================================================================
// HELPER MEMBER FUNCTIONS
// ==========================================================================
std::vector<double> NezhaUnifiedHydrodynamicsPlugin::Str2Vector(const std::string& input) {
    std::vector<double> output;
    std::istringstream iss(input);
    double value;
    while (iss >> value) {
        output.push_back(value);
        if (iss.peek() == ',') iss.ignore();
    }
    return output;
}

// ==========================================================================
// INITIALIZATION ROUTINES
// ==========================================================================

void NezhaUnifiedHydrodynamicsPlugin::InitRos() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "nezha_unified_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    this->rosNode = std::make_unique<ros::NodeHandle>(this->model->GetName());

    std::string phaseServiceName = "/" + this->model->GetName() + "/transmedia/get_phase_sample";
    this->phaseClient = this->rosNode->serviceClient<nezha_plugins::GetPhaseSample>(phaseServiceName);

    // Register the Hydrodynamics Forces Service
    std::string serviceName = "/" + this->model->GetName() + "/get_hydrodynamics_forces";
    ros::AdvertiseServiceOptions aso =
        ros::AdvertiseServiceOptions::create<nezha_plugins::HydrodynamicsForces>(
            serviceName,
            boost::bind(&NezhaUnifiedHydrodynamicsPlugin::OnGetForcesService, this, _1, _2),
            ros::VoidPtr(),
            &this->rosQueue);

    this->forcesService = this->rosNode->advertiseService(aso);

    this->rosRunning = true;
    this->rosQueueThread = std::thread(std::bind(&NezhaUnifiedHydrodynamicsPlugin::QueueThread, this));
}

void NezhaUnifiedHydrodynamicsPlugin::InitPhysics(sdf::ElementPtr _sdf) {
    sdf::ElementPtr params = _sdf;
    if (_sdf->HasElement("hydrodynamic_model")) {
        params = _sdf->GetElement("hydrodynamic_model");
    }

    if (params->HasElement("volume")) {
        this->fossen.volume = params->Get<double>("volume");
    } else {
        gzwarn << "[NezhaUnified] No volume defined! Defaulting to 0.001 m^3." << std::endl;
        this->fossen.volume = 0.001;
    }

    if (params->HasElement("linear_damping")) {
        std::vector<double> vec = this->Str2Vector(params->Get<std::string>("linear_damping"));
        for (int i = 0; i < 6 && i < (int)vec.size(); ++i) this->fossen.DLin(i, i) = vec[i];
    }
    if (params->HasElement("quadratic_damping")) {
        std::vector<double> vec = this->Str2Vector(params->Get<std::string>("quadratic_damping"));
        for (int i = 0; i < 6 && i < (int)vec.size(); ++i) this->fossen.DNonLin(i, i) = vec[i];
    }
}

void NezhaUnifiedHydrodynamicsPlugin::InitWavefield(sdf::ElementPtr _sdf) {
    std::string waveModelName = "nezha_ocean_waves";
    if (_sdf->HasElement("wave_model")) waveModelName = _sdf->Get<std::string>("wave_model");
    this->waveModelName = waveModelName;

    physics::ModelPtr waveModel = this->world->ModelByName(waveModelName);
    if (!waveModel) {
        gzerr << "[NezhaUnified] Wave model '" << waveModelName << "' not found!" << std::endl;
        return;
    }

    std::string entityName = asv::WavefieldEntity::MakeName(waveModelName);
    physics::BasePtr base = waveModel->GetChild(entityName);
    auto wavefieldEntity = boost::dynamic_pointer_cast<asv::WavefieldEntity>(base);

    if (wavefieldEntity) {
        this->wavefield = wavefieldEntity->GetWavefield();  // const Wavefield
    } else {
        gzerr << "[NezhaUnified] Wavefield entity is null: " << entityName << std::endl;
    }
}

void NezhaUnifiedHydrodynamicsPlugin::CreateCollisionMeshes() {
    this->initLinkMeshes.clear();
    this->linkMeshes.clear();
    this->hydrodynamics.clear();
    this->bodyHalfHeight = 0.1;  // safe default; overwritten below

    if (!this->wavefield) {
        gzerr << "[NezhaUnified] Cannot init meshes: wavefield is null." << std::endl;
        return;
    }

    ignition::math::Pose3d linkPose    = this->targetLink->WorldPose();
    ignition::math::Pose3d linkCoMPose = this->targetLink->WorldCoGPose();

    // Water patch sized to the link bounding box (same heuristic as the
    // reference ASV HydrodynamicsPlugin). This is the moving local grid that
    // ComputeDepth samples against.
    auto boundingBox = this->targetLink->CollisionBoundingBox();
    double patchSize = 2.2 * boundingBox.Size().Length();
    if (patchSize < 1.0) patchSize = 1.0;

    std::shared_ptr<asv::Grid> initWaterPatch(
        new asv::Grid({patchSize, patchSize}, {4, 4}));

    this->wavefieldSampler = std::make_shared<asv::WavefieldSampler>(
        this->wavefield, initWaterPatch);
    this->wavefieldSampler->ApplyPose(linkPose);
    this->wavefieldSampler->UpdatePatch();

    asv::Vector3 linVel = asv::ToVector3(this->targetLink->WorldLinearVel());
    asv::Vector3 angVel = asv::ToVector3(this->targetLink->WorldAngularVel());

    double maxHalfZ = 0.0;

    for (auto&& collision : this->targetLink->GetCollisions()) {
        physics::ShapePtr shape = collision->GetShape();
        std::string meshName = collision->GetScopedName();

        if (shape->HasType(physics::Base::EntityType::BOX_SHAPE)) {
            physics::BoxShapePtr box = boost::dynamic_pointer_cast<physics::BoxShape>(shape);
            ignition::math::Vector3d size = box->Size();

            // Track the tallest collision box: its half-height sizes the blend band.
            maxHalfZ = std::max(maxHalfZ, size.Z() * 0.5);

            double res;
            double volume = size.X() * size.Y() * size.Z();
            if (volume < 0.5) res = 0.20;
            else if (volume < 2.0) res = 0.30;
            else res = 0.50;

            int seg_x = std::max(2, std::min(10, static_cast<int>(std::ceil(size.X() / res))));
            int seg_y = std::max(2, std::min(10, static_cast<int>(std::ceil(size.Y() / res))));
            int seg_z = std::max(2, std::min(10, static_cast<int>(std::ceil(size.Z() / res))));

            std::string uniqueName = meshName + "_tessellated";
            CreateTessellatedBox(uniqueName, size, ignition::math::Vector3i(seg_x, seg_y, seg_z));

            if (common::MeshManager::Instance()->HasMesh(uniqueName)) {
                auto gzMesh = common::MeshManager::Instance()->GetMesh(uniqueName);

                // Body-frame init mesh (immutable) + world-frame work mesh (re-posed each step).
                auto initMesh = std::make_shared<asv::Mesh>();
                asv::MeshTools::MakeSurfaceMesh(*gzMesh, *initMesh);
                auto workMesh = std::make_shared<asv::Mesh>(*initMesh);
                ApplyPoseToMesh(linkPose, *initMesh, *workMesh);

                auto hydro = std::make_shared<asv::Hydrodynamics>(
                    this->hydroParams, workMesh, this->wavefieldSampler);
                hydro->Update(this->wavefieldSampler, linkCoMPose, linVel, angVel);

                this->initLinkMeshes.push_back(initMesh);
                this->linkMeshes.push_back(workMesh);
                this->hydrodynamics.push_back(hydro);
            }
        }
    }

    if (maxHalfZ > 1e-4) this->bodyHalfHeight = maxHalfZ;
}

// ==========================================================================
// MAIN LOOP (OnUpdate)
// ==========================================================================
void NezhaUnifiedHydrodynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
    if (!this->targetLink) return;

    ignition::math::Pose3d pose = this->targetLink->WorldPose();

    // ----------------------------------------------------------------------
    // 0. SANITY GUARD. If the link pose or velocity is non-finite or absurd
    //    (uninitialized pose right after spawn, a TF glitch, or the start of a
    //    numerical runaway), do NOT compute or apply any force this step.
    //    Applying force on garbage state is what turns a small overshoot into
    //    an exponential blow-up (position -> 1e16). Bail out cleanly instead.
    // ----------------------------------------------------------------------
    {
        const ignition::math::Vector3d p = pose.Pos();
        const ignition::math::Vector3d v = this->targetLink->WorldLinearVel();
        const double POS_LIMIT = 1.0e4;   // 10 km: lake sim never legitimately exceeds this
        const double VEL_LIMIT = 1.0e3;   // 1000 m/s: physically impossible for this robot
        if (!p.IsFinite() || !v.IsFinite() ||
            std::abs(p.X()) > POS_LIMIT || std::abs(p.Y()) > POS_LIMIT ||
            std::abs(p.Z()) > POS_LIMIT ||
            std::abs(v.X()) > VEL_LIMIT || std::abs(v.Y()) > VEL_LIMIT ||
            std::abs(v.Z()) > VEL_LIMIT) {
            gzwarn << "[NezhaUnified] Skipping force: link state out of range "
                      "(pose/vel non-finite or too large). pos=" << p
                   << " vel=" << v << std::endl;
            return;
        }
    }

    // ----------------------------------------------------------------------
    // 1. Surface elevation under the robot.
    //    Sampled directly from the wavefield sampler (no ROS lag). The ROS
    //    phase service below is kept only for telemetry to the rest of the
    //    transmedia stack.
    // ----------------------------------------------------------------------
    // ----------------------------------------------------------------------
    // 1. Surface elevation under the robot, with explicit validity tracking.
    //    BUG FIX: ComputeDepth returns 0.0 on a patch miss. The old code then
    //    set waveHeight = posZ + 0 = posZ, giving d=0 -> w=0.5, so the robot
    //    read as half-submerged while still high in the AIR. We now require a
    //    finite, in-range sample AND that the body is actually near/under the
    //    surface; otherwise we treat it as airborne (waveHeight far below).
    // ----------------------------------------------------------------------
    double waveHeight;
    bool haveValidSurface = false;
    if (this->wavefield) {
        // Surface elevation under the body, solved ANALYTICALLY at (x, y, t).
        //
        // BUG FIX (damping persisted while OUT OF WATER): the surface used to be
        // read via wavefieldSampler->ComputeDepth(), which casts a vertical ray
        // in +Z and returns the 0.0 "miss" sentinel whenever that ray finds no
        // surface. Once the body is ABOVE the water that miss is GUARANTEED — a
        // ray going up never hits the water below it — so every airborne frame
        // fell through to the stale last-ROS surface height below. If that
        // height sat anywhere near the body, w stayed > 0 and the Fossen +
        // heave damping kept acting on a robot that had already left the water.
        //
        // ComputeDepthDirectly solves the Gerstner wave surface at (x, y, t)
        // with no ray, so it returns a correct signed depth whether the body is
        // above OR below the surface. Above water -> depth < -hBody -> w == 0,
        // and ALL hydrodynamic force terms switch off cleanly.
        auto params = this->wavefield->GetParameters();
        if (params) {
            asv::Point3 c(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
            double depth = asv::WavefieldSampler::ComputeDepthDirectly(
                *params, c, _info.simTime.Double());
            // depth = surfaceZ - pointZ (negative when the body is above water).
            if (std::isfinite(depth)) {
                waveHeight = pose.Pos().Z() + depth;
                haveValidSurface = true;
            }
        }
    }
    if (!haveValidSurface) {
        // Fall back to the last good ROS surface sample if we have one;
        // otherwise assume still water at z=0. Either way, if the body is well
        // above this, the blend below yields w=0 (airborne) as it should.
        waveHeight = this->lastWaveHeight;  // 0.0 until first ROS sample
    }

    // Telemetry-only ROS phase sample (throttled 20 Hz).
    if (_info.simTime.Double() - this->lastTime > 0.05) {
        nezha_plugins::GetPhaseSample srv;
        if (this->phaseClient.call(srv)) {
            this->lastWaveHeight = srv.response.surface_z;
        }
        this->lastTime = _info.simTime.Double();
    }

    // ----------------------------------------------------------------------
    // 2. Blend weight w in [0,1] sized by body height.
    //      d = (surface - centerZ)  ( >0 means the center is below the surface )
    //
    //    We want:
    //      w = 0  when the body TOP just touches the surface (d = -hBody)
    //             -> body still essentially in air        -> 100% mesh
    //      w = 1  when the body BOTTOM is just under       (d = +hBody)
    //             -> body fully submerged                  -> 100% Fossen
    //    so the transition band is one full body height and w hits 1.0 exactly
    //    when the whole body is underwater. This guarantees the (buggy when
    //    fully submerged) mesh engine is switched OFF underwater.
    // ----------------------------------------------------------------------
    double hBody = this->bodyHalfHeight;            // half body height
    double d = waveHeight - pose.Pos().Z();         // signed submergence of center
    double w = (d + hBody) / (2.0 * hBody);         // map [-h, +h] -> [0, 1]
    w = std::max(0.0, std::min(1.0, w));

    // Hard cutoff: once the body center is at/below the surface by a full
    // half-height (top fully under), force pure Fossen. Protects against the
    // mesh engine's normal-cancellation blow-up when fully submerged.
    if (d >= hBody) {
        w = 1.0;
    }

    double immersionRatio = w;  // reuse as the reported submersion ratio

    // Phase label (telemetry).
    if (w <= 0.0)      this->phaseManager.currentPhase = PhaseState::ABOVE;
    else if (w >= 1.0) this->phaseManager.currentPhase = PhaseState::BELOW;
    else               this->phaseManager.currentPhase = PhaseState::SURFACE;

    // ----------------------------------------------------------------------
    // 2b. WAVE KINEMATICS — surface slope (normal) and vertical water velocity.
    //   These drive the two missing wave-coupling effects:
    //     • waveNormal: local wave-surface normal from the spatial slope
    //       (dEta/dx, dEta/dy). Used to TILT the body to the wave face so it
    //       does not land/float flat on a sloped wave.
    //     • waterVelZ: the wave's vertical water velocity (dEta/dt), decayed
    //       with depth. Fed into the damping as a RELATIVE velocity so the
    //       water carries the body up/down with the wave — including while
    //       fully submerged, where there is otherwise no wave coupling at all.
    // ----------------------------------------------------------------------
    ignition::math::Vector3d waveNormal(0, 0, 1);  // default: flat water
    double waterVelZ = 0.0;
    auto slopeParams = this->wavefield ? this->wavefield->GetParameters() : nullptr;
    if (w > 0.0 && haveValidSurface && slopeParams) {
        const double ds = 0.05;  // finite-difference step [m]
        const double px = pose.Pos().X();
        const double py = pose.Pos().Y();
        const double pz = pose.Pos().Z();
        // Same analytic surface as above (ray-free), so the slope is valid even
        // when the body is above the water.
        const auto& params = slopeParams;
        const double nowT = _info.simTime.Double();
        auto surfAt = [&](double sx, double sy) {
            return pz + asv::WavefieldSampler::ComputeDepthDirectly(
                *params, asv::Point3(sx, sy, pz), nowT);
        };
        double dEtaDx = (surfAt(px + ds, py) - surfAt(px - ds, py)) / (2.0 * ds);
        double dEtaDy = (surfAt(px, py + ds) - surfAt(px, py - ds)) / (2.0 * ds);
        // Clamp slopes so a patch-edge glitch cannot produce a wild normal.
        dEtaDx = std::max(-2.0, std::min(2.0, dEtaDx));
        dEtaDy = std::max(-2.0, std::min(2.0, dEtaDy));
        waveNormal.Set(-dEtaDx, -dEtaDy, 1.0);
        waveNormal.Normalize();

        // Vertical surface velocity dEta/dt (finite difference between frames).
        double now = _info.simTime.Double();
        if (this->havePrevSurface) {
            double dtS = now - this->prevSurfaceTime;
            if (dtS > 1e-6) {
                waterVelZ = (waveHeight - this->prevSurfaceZ) / dtS;
                waterVelZ = std::max(-3.0, std::min(3.0, waterVelZ));  // clamp
            }
        }
        this->prevSurfaceZ    = waveHeight;
        this->prevSurfaceTime = now;
        this->havePrevSurface = true;

        // Orbital motion decays with depth below the surface.
        double depthBelow = std::max(0.0, waveHeight - pz);
        waterVelZ *= std::exp(-depthBelow / std::max(0.05, this->waveDecayLength));
    }
    const ignition::math::Vector3d vWaterWorld(0.0, 0.0, waterVelZ);

    // ----------------------------------------------------------------------
    // 3. SURFACE REGIME: full mesh wave hydrodynamics (buoyancy + viscous drag
    //    + pressure drag + damping). Weighted by (1 - w) so it fades out as the
    //    body submerges. WORLD frame.
    //
    //    PROBLEM A FIX (premature buoyancy): only run the mesh engine when the
    //    body is actually in contact with the water (w > 0, i.e. the lowest
    //    point is at/below the local surface). Previously it ran for any w < 1,
    //    including a fully-airborne body (w == 0) at full weight (1-w == 1), so
    //    any spurious buoyancy the mesh reported lifted the body while it was
    //    still above the surface. No water contact -> no hydrodynamic force.
    // ----------------------------------------------------------------------
    ignition::math::Vector3d meshForce(0, 0, 0);
    ignition::math::Vector3d meshTorque(0, 0, 0);
    ignition::math::Vector3d meshBuoyancy(0, 0, 0);
    ignition::math::Vector3d meshWaveDrag(0, 0, 0);

    if (this->wavefield && this->wavefieldSampler && !this->hydrodynamics.empty()
        && w > 0.0 && w < 1.0) {
        ignition::math::Pose3d linkCoMPose = this->targetLink->WorldCoGPose();

        this->wavefieldSampler->ApplyPose(pose);
        this->wavefieldSampler->UpdatePatch();

        // Clamp the velocity handed to the mesh engine. On water entry a fast-
        // falling body has a large vertical velocity; the engine's drag scales
        // with v (and v^2), so an unclamped impact velocity produces a single
        // enormous force that explicit integration turns into a launch. Bound
        // the speed used for force computation (not the actual body velocity).
        const double IMPACT_V_CLAMP = 5.0;  // m/s
        ignition::math::Vector3d vWorld = this->targetLink->WorldLinearVel();
        vWorld.X() = std::max(-IMPACT_V_CLAMP, std::min(IMPACT_V_CLAMP, vWorld.X()));
        vWorld.Y() = std::max(-IMPACT_V_CLAMP, std::min(IMPACT_V_CLAMP, vWorld.Y()));
        vWorld.Z() = std::max(-IMPACT_V_CLAMP, std::min(IMPACT_V_CLAMP, vWorld.Z()));

        asv::Vector3 mLinVel = asv::ToVector3(vWorld);

        // Clamp angular velocity too: unbounded spin -> large torque -> larger
        // spin next step. Bounding it breaks the rotational feedback loop that
        // turns a water-entry slam into a divergent tumble.
        const double IMPACT_W_CLAMP = 10.0;  // rad/s
        ignition::math::Vector3d wWorld = this->targetLink->WorldAngularVel();
        wWorld.X() = std::max(-IMPACT_W_CLAMP, std::min(IMPACT_W_CLAMP, wWorld.X()));
        wWorld.Y() = std::max(-IMPACT_W_CLAMP, std::min(IMPACT_W_CLAMP, wWorld.Y()));
        wWorld.Z() = std::max(-IMPACT_W_CLAMP, std::min(IMPACT_W_CLAMP, wWorld.Z()));
        asv::Vector3 mAngVel = asv::ToVector3(wWorld);

        // The wavefield patch is synced here only while the mesh engine is active.
        for (size_t j = 0; j < this->linkMeshes.size(); ++j) {
            ApplyPoseToMesh(pose, *this->initLinkMeshes[j], *this->linkMeshes[j]);
            this->hydrodynamics[j]->Update(
                this->wavefieldSampler, linkCoMPose, mLinVel, mAngVel);

            auto f = asv::ToIgn(this->hydrodynamics[j]->Force());
            auto t = asv::ToIgn(this->hydrodynamics[j]->Torque());
            if (f.IsFinite()) meshForce  += f;
            if (t.IsFinite()) meshTorque += t;

            auto b  = asv::ToIgn(this->hydrodynamics[j]->GetBuoyancyForce());
            auto wd = asv::ToIgn(this->hydrodynamics[j]->GetWaveDragForce());
            if (b.IsFinite())  meshBuoyancy += b;
            if (wd.IsFinite()) meshWaveDrag += wd;
        }

        // Guard ONLY the buoyancy component against a spurious downward sign
        // (inconsistent normals / fully-submerged cancellation). The total mesh
        // force also contains velocity-dependent DRAG, and that must be free to
        // act in BOTH directions.
        //
        // BUG FIX (robot flew into the sky under waves): the old code rectified
        // the TOTAL mesh Z force (zeroing any net-downward force). Drag opposes
        // motion, so this kept the up-push while the body rose and dropped the
        // down-push as it fell — a one-way energy pump that ratcheted the body
        // upward every wave cycle until it launched out of the world. It also
        // killed wave-following, which needs the force to swing both ways.
        ignition::math::Vector3d meshDrag = meshForce - meshBuoyancy;  // viscous+pressure+damping
        ignition::math::Vector3d buoy = meshBuoyancy;
        if (buoy.Z() < 0.0) buoy.Z() = 0.0;   // buoyancy alone never pulls down

        double mw = (1.0 - w);
        meshForce  = (buoy + meshDrag) * mw;
        meshTorque *= mw;
    }

    // ----------------------------------------------------------------------
    // 4. UNDERWATER REGIME: Fossen buoyancy (WORLD frame, up) + Fossen damping
    //    (BODY frame). Weighted by w so it fades IN exactly as the mesh fades
    //    OUT -> the sum is continuous across the surface, closing the gap.
    // ----------------------------------------------------------------------
    ignition::math::Vector3d fossenBuoyancy(0, 0, 0);   // WORLD frame
    ignition::math::Vector3d dampingForce(0, 0, 0);     // BODY frame
    ignition::math::Vector3d dampingTorque(0, 0, 0);    // BODY frame

    if (w > 0.0) {
        const double rho = 1028.0;
        const double g   = 9.81;

        // Fossen buoyancy: full displaced-volume buoyancy when fully submerged.
        fossenBuoyancy.Z() = this->fossen.volume * rho * g * w;

        // Fossen damping (linear + quadratic), body frame, scaled by w.
        //
        // UNDERWATER WAVE FOLLOWING: damp the body velocity RELATIVE TO THE
        // WATER, not relative to still water. Subtracting the wave orbital
        // velocity (vertical) before damping means the same term that resists
        // motion also drags the body along with the wave — so a fully submerged
        // body rises and falls with the wave above it instead of sitting still.
        ignition::math::Vector3d linVelWorldRel =
            this->targetLink->WorldLinearVel() - vWaterWorld;
        ignition::math::Vector3d linVelB = pose.Rot().RotateVectorReverse(linVelWorldRel);
        ignition::math::Vector3d angVelB = this->targetLink->RelativeAngularVel();

        Eigen::Matrix<double, 6, 1> vel;
        vel << linVelB.X(), linVelB.Y(), linVelB.Z(),
               angVelB.X(), angVelB.Y(), angVelB.Z();

        Eigen::Matrix<double, 6, 1> damping = -this->fossen.DLin * vel;
        for (int i = 0; i < 6; ++i) {
            damping(i) -= this->fossen.DNonLin(i, i) * std::abs(vel(i)) * vel(i);
        }
        damping *= w;

        dampingForce.Set(damping(0), damping(1), damping(2));
        dampingTorque.Set(damping(3), damping(4), damping(5));
    }

    // ----------------------------------------------------------------------
    // 4b. HEAVE DAMPING / ENTRY DRAG (world Z) — the anti-bounce term.
    //     A floating body is a mass on a spring (buoyancy). Without a velocity-
    //     dependent term it oscillates forever, and a body DROPPED into the
    //     water converts its impact kinetic energy into buoyancy spring energy
    //     and springs straight back out (Problem B). Two terms remove that
    //     energy:
    //       - LINEAR  (-c*vz):        sets the settling / bob decay at low speed.
    //       - QUADRATIC(-k*|vz|*vz):  the v^2 water-entry / slam drag that
    //                                 dominates at impact speed and smoothly
    //                                 arrests a fast entry without a rigid bounce.
    //     The Fossen damping above is scaled by w and vanishes at the surface
    //     (w~0) exactly where the bobbing/entry happens, so this term carries it.
    //
    //     Faded in with water contact = max(w, mesh-wet fraction): active
    //     whenever the mesh is wet (0<w<1 with buoyancy) or submerged (w>0).
    // ----------------------------------------------------------------------
    ignition::math::Vector3d heaveForce(0, 0, 0);  // WORLD frame
    {
        bool meshWet = (meshForce.Z() > 1e-6);
        bool inContact = (w > 0.0) || meshWet;
        if (inContact) {
            // Velocity RELATIVE TO THE WAVE'S vertical water motion, so this
            // term too carries the body with the wave (it damps the body toward
            // the water velocity, not toward zero) instead of fighting it.
            double vz = this->targetLink->WorldLinearVel().Z() - waterVelZ;

            // Clamp the velocity used for damping. A transient velocity spike
            // (from the stiff buoyancy spring overshooting on one timestep)
            // must NOT be turned into an enormous damping force, or explicit
            // integration diverges. Bounding vz here breaks that feedback loop.
            const double VZ_CLAMP = 5.0;  // m/s; well above any real heave speed
            vz = std::max(-VZ_CLAMP, std::min(VZ_CLAMP, vz));

            double contact = std::max(w, meshWet ? (1.0 - w) : 0.0);
            contact = std::max(0.0, std::min(1.0, contact));

            // Linear + quadratic vertical drag, both opposing vz and scaled by
            // the wetted fraction (drag grows with submerged area on entry).
            //
            // TUNING: keep this UNDER-damped (target damping ratio ~0.4 of
            // critical). Critical damping for this body is c_crit = 2*sqrt(k*m)
            // ~= 14 N*s/m (k = buoyancy spring stiffness, m = mass). Much above
            // that and the body stops dead on contact and oozes down instead of
            // plunging in and bobbing back like a real water entry. The big
            // rigid bounce this used to fight is now handled by the (gentle,
            // proportional) buoyancy itself, so this only needs to take the edge
            // off and decay the bob over a couple of cycles.
            double fz = -(this->heaveDamping * vz
                          + this->heaveDragQuad * std::abs(vz) * vz) * contact;

            // Hard cap on the heave force magnitude as a final stability
            // backstop. Kept to a few times the robot's weight so it cannot
            // arrest the plunge instantly (a high cap = "hits a wall" on entry);
            // still well under TOTAL_F_CLAMP below.
            const double F_CLAMP = 10.0;  // N
            fz = std::max(-F_CLAMP, std::min(F_CLAMP, fz));

            heaveForce.Z() = fz;
        }
    }

    // ----------------------------------------------------------------------
    // 4c. SURFACE-SLOPE ALIGNMENT TORQUE (world frame).
    //     A wave face is tilted, so a body sitting on / entering it should tilt
    //     to match instead of landing flat. Rotate the body's up-axis toward the
    //     local wave normal: T = gain * w * (bodyUp x waveNormal). The cross
    //     product is the shortest-rotation axis and its magnitude ~ sin(tilt),
    //     so the torque vanishes once aligned. Scaled by submersion w and
    //     damped by the Fossen angular damping above (no separate damping term).
    // ----------------------------------------------------------------------
    ignition::math::Vector3d alignTorque(0, 0, 0);
    if (w > 0.0 && this->waveAlignGain > 0.0) {
        ignition::math::Vector3d bodyUp =
            pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
        alignTorque = bodyUp.Cross(waveNormal) * (this->waveAlignGain * w);
    }

    // ----------------------------------------------------------------------
    // 5. Save for ROS telemetry.
    // ----------------------------------------------------------------------
    this->lastBuoyancyForce   = meshBuoyancy * (1.0 - w) + fossenBuoyancy;
    this->lastWaveForce       = meshWaveDrag * (1.0 - w);
    this->lastDampingForce    = dampingForce;
    this->lastCoriolisForce   = dampingTorque;
    // Heave damping + quadratic entry drag (WORLD frame). Reported separately so a
    // force sampler reconstructing the total vertical force does not under-count it.
    this->lastHeaveForce      = heaveForce;
    this->lastSubmersionRatio = immersionRatio;

    // ----------------------------------------------------------------------
    // 6. Apply forces.
    //    Mesh force/torque  -> WORLD frame
    //    Fossen buoyancy    -> WORLD frame
    //    Fossen damping     -> BODY frame
    // ----------------------------------------------------------------------
    ignition::math::Vector3d worldForce = meshForce + fossenBuoyancy + heaveForce;

    // Helper: clamp each component of a vector to +/- limit.
    auto clampVec = [](ignition::math::Vector3d v, double lim) {
        v.X() = std::max(-lim, std::min(lim, v.X()));
        v.Y() = std::max(-lim, std::min(lim, v.Y()));
        v.Z() = std::max(-lim, std::min(lim, v.Z()));
        return v;
    };

    // Final catch-all clamps. For a ~1 kg robot none of these should ever be
    // approached in normal operation; they exist purely to make a water-entry
    // slam or a bad explicit step IMPOSSIBLE to diverge.
    //
    // CRITICAL: torque was previously UNCLAMPED. On water entry the mesh engine
    // applies buoyancy at the center-of-pressure; the lever arm turns a big
    // entry force into a big torque, which spins the body, which feeds back as
    // an even bigger torque next step -> blow-up. Clamping torque closes that
    // loop (the force clamp alone never could).
    const double TOTAL_F_CLAMP = 200.0;  // N
    const double TOTAL_T_CLAMP = 50.0;   // N*m

    // Mesh slope torque + wave-normal alignment torque (both WORLD frame).
    ignition::math::Vector3d worldTorque = meshTorque + alignTorque;

    worldForce  = clampVec(worldForce,  TOTAL_F_CLAMP);
    worldTorque = clampVec(worldTorque, TOTAL_T_CLAMP);
    dampingForce  = clampVec(dampingForce,  TOTAL_F_CLAMP);
    dampingTorque = clampVec(dampingTorque, TOTAL_T_CLAMP);

    if (worldForce.IsFinite())  this->targetLink->AddForce(worldForce);
    if (worldTorque.IsFinite()) this->targetLink->AddTorque(worldTorque);

    if (dampingForce.IsFinite())  this->targetLink->AddRelativeForce(dampingForce);
    if (dampingTorque.IsFinite()) this->targetLink->AddRelativeTorque(dampingTorque);
}

bool NezhaUnifiedHydrodynamicsPlugin::OnGetForcesService(
    nezha_plugins::HydrodynamicsForces::Request &req,
    nezha_plugins::HydrodynamicsForces::Response &res) {

    res.buoyancy_x = this->lastBuoyancyForce.X();
    res.buoyancy_y = this->lastBuoyancyForce.Y();
    res.buoyancy_z = this->lastBuoyancyForce.Z();

    res.damping_x = this->lastDampingForce.X();
    res.damping_y = this->lastDampingForce.Y();
    res.damping_z = this->lastDampingForce.Z();

    res.added_mass_x = 0.0;
    res.added_mass_y = 0.0;
    res.added_mass_z = 0.0;

    res.wave_x = this->lastWaveForce.X();
    res.wave_y = this->lastWaveForce.Y();
    res.wave_z = this->lastWaveForce.Z();

    res.coriolis_x = this->lastCoriolisForce.X();
    res.coriolis_y = this->lastCoriolisForce.Y();
    res.coriolis_z = this->lastCoriolisForce.Z();

    // Heave damping + quadratic water-entry drag (WORLD frame). Including this in the
    // response makes the resultant buoyancy+damping+wave+heave equal the true applied
    // vertical force (previously omitted -> sampler under-counted the unified plugin).
    res.heave_x = this->lastHeaveForce.X();
    res.heave_y = this->lastHeaveForce.Y();
    res.heave_z = this->lastHeaveForce.Z();

    res.submersion_ratio = this->lastSubmersionRatio;

    if (this->world) {
        res.sim_time = this->world->SimTime().Double();
    } else {
        res.sim_time = 0.0;
    }

    return true;
}

void NezhaUnifiedHydrodynamicsPlugin::QueueThread() {
    static const double timeout = 0.01;
    while (this->rosRunning && this->rosNode) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(nezha::NezhaUnifiedHydrodynamicsPlugin)

} // namespace nezha
