//
// Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
// Shanghai Jiao Tong University, The Nezha Lab
// Key Laboratory of Polar Ecosystem and Climate Change
// State Key Laboratory of Submarine Geoscience
//
#include "nezha_surfacePlugin.hh"
#include "nasv_physics.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/MeshTools.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include <algorithm>
#include <cctype>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/Shape.hh>
#include <sstream>  // ✅ 

#include <ignition/math/Pose3.hh>
#include <ignition/math/Triangle3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport.hh>

#include <iostream>
#include <iomanip>
#include <string>
#include <exception>

using namespace gazebo;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace {
  std::vector<double> Str2Vector(const std::string& _input)
  {
    std::vector<double> output;
    std::istringstream iss(_input);
    double value;
    
    while (iss >> value) {
      output.push_back(value);
      // 
      if (iss.peek() == ',') {
        iss.ignore();
      }
    }
    
    return output;
  }
}

namespace asv
{
  std::vector<HydrodynamicsPlugin*> g_nasvPluginRegistry;
  std::mutex g_nasvRegistryMutex;
  HydrodynamicsPlugin* g_lastHydrodynamicsPlugin = nullptr;

  GZ_REGISTER_MODEL_PLUGIN(HydrodynamicsPlugin)
//  Box 
void CreateTessellatedBox(const std::string& name, 
                          const ignition::math::Vector3d& size, 
                          const ignition::math::Vector3i& segments)
{
    gazebo::common::MeshManager *meshMgr = gazebo::common::MeshManager::Instance();
    if (meshMgr->HasMesh(name)) return;

    gazebo::common::Mesh *mesh = new gazebo::common::Mesh();
    mesh->SetName(name);
    gazebo::common::SubMesh *subMesh = new gazebo::common::SubMesh();
    mesh->AddSubMesh(subMesh);

    // 
    double dx = size.X() / 2.0;
    double dy = size.Y() / 2.0;
    double dz = size.Z() / 2.0;

    //  6 
    // 0:X, 1:Y, 2:Z.  Direction: 1 or -1
    struct Face { int u_ax; int v_ax; int w_ax; double w_dir; int u_seg; int v_seg; };
    std::vector<Face> faces = {
        {1, 2, 0,  1.0, segments.Y(), segments.Z()}, // +X Face (Right)
        {1, 2, 0, -1.0, segments.Y(), segments.Z()}, // -X Face (Left)
        {0, 2, 1,  1.0, segments.X(), segments.Z()}, // +Y Face (Back)
        {0, 2, 1, -1.0, segments.X(), segments.Z()}, // -Y Face (Front)
        {0, 1, 2,  1.0, segments.X(), segments.Y()}, // +Z Face (Top)
        {0, 1, 2, -1.0, segments.X(), segments.Y()}  // -Z Face (Bottom)
    };

    int vertexOffset = 0;

    for (const auto& face : faces) {
        double w_val = face.w_dir * (face.w_ax == 0 ? dx : (face.w_ax == 1 ? dy : dz));
        double u_len = (face.u_ax == 0 ? size.X() : (face.u_ax == 1 ? size.Y() : size.Z()));
        double v_len = (face.v_ax == 0 ? size.X() : (face.v_ax == 1 ? size.Y() : size.Z()));

        double u_step = u_len / face.u_seg;
        double v_step = v_len / face.v_seg;

        // 
        for (int i = 0; i <= face.u_seg; ++i) {
            for (int j = 0; j <= face.v_seg; ++j) {
                ignition::math::Vector3d pt;
                //  UVW 
                double u = -u_len / 2.0 + i * u_step;
                double v = -v_len / 2.0 + j * v_step;
                
                //  XYZ
                double coords[3];
                coords[face.u_ax] = u;
                coords[face.v_ax] = v;
                coords[face.w_ax] = w_val;
                
                subMesh->AddVertex(coords[0], coords[1], coords[2]);
            }
        }

        //  ()
        for (int i = 0; i < face.u_seg; ++i) {
            for (int j = 0; j < face.v_seg; ++j) {
                int row_len = face.v_seg + 1;
                int v0 = vertexOffset + i * row_len + j;
                int v1 = vertexOffset + i * row_len + (j + 1);
                int v2 = vertexOffset + (i + 1) * row_len + j;
                int v3 = vertexOffset + (i + 1) * row_len + (j + 1);

                //  ()
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

void CreateCollisionMeshes(
  physics::ModelPtr _model,
  std::vector<physics::LinkPtr>& _links,
  std::vector<std::vector<std::shared_ptr<Mesh>>>& _meshes)
{
    _links.clear();
  _meshes.clear();
  GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
  

  int linkIndex = 0;
  for (auto&& link : _model->GetLinks())
  {
    GZ_ASSERT(link != nullptr, "Link must be valid");
    std::string linkName(link->GetName());
    
    
    size_t lastSlash = linkName.find_last_of("/:");
    std::string simpleName = (lastSlash != std::string::npos) 
                              ? linkName.substr(lastSlash + 1) 
                              : linkName;
    
    if (linkName.find("base_link") == std::string::npos)
    {
      linkIndex++;
      continue;
    }
    
    // ============  collision ============
    _links.push_back(link);
    std::vector<std::shared_ptr<Mesh>> linkMeshes;
    
    int collisionIndex = 0;
    for (auto&& collision : link->GetCollisions())
    {
      GZ_ASSERT(collision != nullptr, "Collision must be valid");
      std::string collisionName(collision->GetName());
      
      physics::ShapePtr shape = collision->GetShape();
      GZ_ASSERT(shape != nullptr, "Shape must be valid");
      // ========== CYLINDER_SHAPE ==========
      if (shape->HasType(physics::Base::EntityType::CYLINDER_SHAPE))
      {
        physics::CylinderShapePtr cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);
        GZ_ASSERT(cylinder != nullptr, "Failed to cast Shape to CylinderShape");
        
        double radius = cylinder->GetRadius();
        double length = cylinder->GetLength();
        gzmsg << "      Radius: " << radius << ", Length: " << length << std::endl;

        std::string meshName = std::string(_model->GetName())
          .append("::").append(linkName)
          .append("::").append(collisionName)
          .append("::cylinder");
        
        common::MeshManager::Instance()->CreateCylinder(
          meshName,
          radius,
          length,
          1,
          32);
        
        GZ_ASSERT(common::MeshManager::Instance()->HasMesh(meshName),
          "Failed to create Mesh for Cylinder");

        std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
        MeshTools::MakeSurfaceMesh(
          *common::MeshManager::Instance()->GetMesh(meshName), *mesh);
        GZ_ASSERT(mesh != nullptr, "Invalid Surface Mesh");
        linkMeshes.push_back(mesh);

        gzmsg << "      Vertices: " << mesh->number_of_vertices() << std::endl;
      }
      
// ========== BOX_SHAPE  ==========
if (shape->HasType(physics::Base::EntityType::BOX_SHAPE))
{
    physics::BoxShapePtr box = boost::dynamic_pointer_cast<physics::BoxShape>(shape);
    ignition::math::Vector3d size = box->Size();

    // ✅ 
    double res;
    double volume = size.X() * size.Y() * size.Z();
    
    if (volume < 0.5) {
        res = 0.20;  //  (< 0.5 m³)
    } else if (volume < 2.0) {
        res = 0.30;  //  (0.5-2 m³) ←  rexrov 
    } else {
        res = 0.50;  //  (> 2 m³)
    }
    
    // 
    int seg_x = std::max(2, std::min(40, static_cast<int>(std::ceil(size.X() / res))));
    int seg_y = std::max(2, std::min(40, static_cast<int>(std::ceil(size.Y() / res))));
    int seg_z = std::max(2, std::min(40, static_cast<int>(std::ceil(size.Z() / res))));

    ignition::math::Vector3i segments(seg_x, seg_y, seg_z);

    // 
    static int box_counter = 0;
    std::string unique_suffix = std::to_string(box_counter++);
    std::string meshName = std::string(_model->GetName()) + "::box_" + unique_suffix;

    // 
    CreateTessellatedBox(meshName, size, segments);

    // 
    if (common::MeshManager::Instance()->HasMesh(meshName))
    {
        const common::Mesh* gzMesh = common::MeshManager::Instance()->GetMesh(meshName);
        std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
        MeshTools::MakeSurfaceMesh(*gzMesh, *mesh);
        
        if (mesh && mesh->number_of_vertices() > 0) {
            linkMeshes.push_back(mesh);
            
            gzmsg << "   [Optimized Mesh] Box: " << collisionName 
                  << " | Res: " << res << "m"
                  << " | Grid: [" << seg_x << "×" << seg_y << "×" << seg_z << "]"
                  << " | Faces: " << mesh->number_of_faces() 
                  << " | Volume: " << std::fixed << std::setprecision(2) << volume << " m³"
                  << std::endl;
        }
    }
}



      // ========== MESH_SHAPE ==========
      if (shape->HasType(physics::Base::EntityType::MESH_SHAPE))
      {
        physics::MeshShapePtr meshShape = boost::dynamic_pointer_cast<physics::MeshShape>(shape);
        GZ_ASSERT(meshShape != nullptr, "Failed to cast Shape to MeshShape");

        std::string meshUri = meshShape->GetMeshURI();
        std::string meshStr = common::find_file(meshUri);
        gzmsg << "      MeshURI: " << meshUri << std::endl;
        gzmsg << "      MeshStr: " << meshStr << std::endl;

        if (!common::MeshManager::Instance()->HasMesh(meshStr))
        {
          gzerr << "Mesh: " << meshStr << " was not loaded" << std::endl;
          collisionIndex++;
          continue;
        }

        std::shared_ptr<Mesh> mesh = std::make_shared<Mesh>();
        MeshTools::MakeSurfaceMesh(
          *common::MeshManager::Instance()->GetMesh(meshStr), *mesh);
        GZ_ASSERT(mesh != nullptr, "Invalid Surface Mesh");
        linkMeshes.push_back(mesh);

      }

      collisionIndex++;
    }
    
    _meshes.push_back(linkMeshes);
    
    linkIndex++;
  }
}


  void ApplyPose(
    const ignition::math::Pose3d& _pose,
    const Mesh& _source,
    Mesh& _target)
  {
    for (
      auto&& it = std::make_pair(std::begin(_source.vertices()), std::begin(_target.vertices()));
      it.first != std::end(_source.vertices()) && it.second != std::end(_target.vertices());
      ++it.first, ++it.second)
    {
      auto& v0 = *it.first;
      auto& v1 = *it.second;
      const Point3& p0 = _source.point(v0);

      ignition::math::Vector3d ignP0(p0.x(), p0.y(), p0.z());
      ignition::math::Vector3d ignP1 = _pose.Rot().RotateVector(ignP0) + _pose.Pos();

      Point3& p1 = _target.point(v1);
      p1 = Point3(ignP1.X(), ignP1.Y(), ignP1.Z());
    }
  }

  std::shared_ptr<const Wavefield> GetWavefield(
    physics::WorldPtr _world,
    const std::string& _waveModelName)
  {
    GZ_ASSERT(_world != nullptr, "World is null");

    physics::ModelPtr wavefieldModel = _world->ModelByName(_waveModelName);    
    if(wavefieldModel == nullptr)
    {
      gzerr << "No Wavefield Model found with name '" << _waveModelName << "'." << std::endl;
      return nullptr;
    }

    std::string wavefieldEntityName(WavefieldEntity::MakeName(_waveModelName));

    physics::BasePtr base = wavefieldModel->GetChild(wavefieldEntityName);
    boost::shared_ptr<WavefieldEntity> wavefieldEntity 
      = boost::dynamic_pointer_cast<WavefieldEntity>(base);
    if (wavefieldEntity == nullptr)
    {
      gzerr << "Wavefield Entity is null: " << wavefieldEntityName << std::endl;
      return nullptr;
    }    
    GZ_ASSERT(wavefieldEntity->GetWavefield() != nullptr, "Wavefield is null.");

    return wavefieldEntity->GetWavefield();
  }


  class HydrodynamicsLinkData
  {
    public: physics::LinkPtr link;


    public: std::shared_ptr<WavefieldSampler> wavefieldSampler;
    

    public: std::vector<std::shared_ptr<Mesh>> initLinkMeshes;


    public: std::vector<std::shared_ptr<Mesh>> linkMeshes;


    public: std::vector<std::shared_ptr<Hydrodynamics>> hydrodynamics;


    public: ignition::msgs::Marker waterPatchMsg;


    public: std::vector<ignition::msgs::Marker> waterlineMsgs;


    public: std::vector<ignition::msgs::Marker> underwaterSurfaceMsgs;
    public: double lastBuoyancyCorrection{0.0}; 
    public: double lastSubmersionRatio{0.0};
    public: double lastGlobalWaterLevel{0.0};     
  };


  class HydrodynamicsPluginPrivate
  {
    public: physics::WorldPtr world;
    public: std::map<std::string, bool> forcesServiceRegistered;

    public: physics::ModelPtr model;
    public: physics::LinkPtr baseLink;

    public: std::shared_ptr<const Wavefield> wavefield;

    public: std::shared_ptr<HydrodynamicsParameters> hydroParams;

    public: std::vector<std::shared_ptr<HydrodynamicsLinkData>> hydroData;

    public: std::string waveModelName;
    
      public: double waterPatchLength{2.0};
  public: double waterPatchWidth{2.0};

    public: bool showWaterPatch;

    public: bool showWaterline;

    public: bool showUnderwaterSurface;

    public: double updateRate;

    public: common::Time prevTime;

    public: event::ConnectionPtr updateConnection;

    public: ignition::transport::Node ignNode;

    public: transport::NodePtr gzNode;

    public: transport::SubscriberPtr hydroSub;

    public: ignition::math::Vector3d lastTotalForce{0,0,0};
    public: ignition::math::Vector3d lastTotalTorque{0,0,0};
        // ==========  Fossen  ==========
    public: Matrix6d Ma;
    public: Matrix6d DLin;
    public: Matrix6d DNonLin;
    public: Matrix6d DLinForwardSpeed;

    
    double scalingAddedMass{1.0};
    double scalingDamping{1.0};
    double offsetLinearDamping{0.0};
    double offsetNonLinDamping{0.0};
    
    // 
    public: Vector6d lastVelRel;
    public: Vector6d filteredAcc;
    public: double lastTime{0.0};
    public: common::Time lastDebugTime;        // 
    public: double debugOutputInterval{0.5};    // 
      public: ros::NodeHandle* rosNode{nullptr};
  public: ros::ServiceServer forcesService;
  public: ros::CallbackQueue rosQueue;
  public: std::thread rosQueueThread;
  
  public: ignition::math::Vector3d lastBuoyancyForce{0,0,0};
  public: ignition::math::Vector3d lastDampingForce{0,0,0};
  public: ignition::math::Vector3d lastWaveForce{0,0,0};
  public: ignition::math::Vector3d lastAddedMassForce{0,0,0};
  public: ignition::math::Vector3d lastCoriolisForce{0,0,0};
  public: double lastSubmersionRatio{0.0};
  public: double lastGlobalWaterLevel{0.0}; 

  };


std::string HydrodynamicsPlugin::GetName() const
{
    if (this->data && this->data->model) {
        return this->data->model->GetName() + "::hydrodynamics";
    }
    return "hydrodynamics_unknown";
}
gazebo::physics::ModelPtr HydrodynamicsPlugin::GetModel() const
{
    if (this->data) {
        return this->data->model;
    }
    return nullptr;
}

HydrodynamicsPlugin::HydrodynamicsPlugin() : 
    ModelPlugin(),
    data(new HydrodynamicsPluginPrivate())
{
    std::lock_guard<std::mutex> lock(g_nasvRegistryMutex);
    g_nasvPluginRegistry.push_back(this);
}

HydrodynamicsPlugin::~HydrodynamicsPlugin()
{    if (this->data->rosNode)
    {
        this->data->forcesService.shutdown();
        this->data->rosNode->shutdown();
        delete this->data->rosNode;
        this->data->rosNode = nullptr;
    }
    
    // 
    if (this->data->rosQueueThread.joinable())
    {
        this->data->rosQueueThread.join();
    }

    if (data && data->model)
    {
        HydrodynamicsPluginRegistry::GetInstance().Unregister(data->model);
    }
    

    std::lock_guard<std::mutex> lock(g_nasvRegistryMutex);
    auto it = std::find(g_nasvPluginRegistry.begin(), 
                        g_nasvPluginRegistry.end(), this);
    if (it != g_nasvPluginRegistry.end())
    {
        g_nasvPluginRegistry.erase(it);
    }
}


void HydrodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
  GZ_ASSERT(_sdf   != nullptr, "Invalid parameter _sdf");

  this->data->model = _model;
  this->data->world = _model->GetWorld();
  GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

  this->data->gzNode = transport::NodePtr(new transport::Node());
  this->data->gzNode->Init(this->data->world->Name() + "/" + this->data->model->GetName());

  this->data->hydroSub = this->data->gzNode->Subscribe(
    "~/hydrodynamics", &HydrodynamicsPlugin::OnHydrodynamicsMsg, this);

  this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&HydrodynamicsPlugin::OnUpdate, this));
{
    static std::mutex rosInitMutex; // Static ensures it's shared across instances
    std::lock_guard<std::mutex> lock(rosInitMutex);
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        // Note: Use a generic name or AnonymousName to avoid conflicts
        ros::init(argc, argv, "nezha_sim_node", 
                 ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
}
    this->data->rosNode = new ros::NodeHandle("~");
    
    //  service
    std::string serviceName = "/" + this->data->model->GetName() + "/get_hydrodynamics_forces";
    
// ✅  ( nezha_plugins s)
ros::AdvertiseServiceOptions aso = 
    ros::AdvertiseServiceOptions::create<nezha_plugins::HydrodynamicsForces>(
        serviceName,
        boost::bind(&HydrodynamicsPlugin::OnGetForcesService, this, _1, _2),
        ros::VoidPtr(),
        &this->data->rosQueue);  // ←  this->data->
    
    this->data->forcesService = this->data->rosNode->advertiseService(aso);
      this->data->rosQueueThread = std::thread(
    std::bind(&HydrodynamicsPlugin::QueueThread, this)
  );
    gzmsg << "✓ Hydrodynamics Forces Service available at: " 
          << serviceName << std::endl;
  // ========== Wave Model  ==========
  std::string rawWaveModel = Utilities::SdfParamString(*_sdf, "wave_model", "");
  
  bool isValid = true;
  for (char c : rawWaveModel) {
    if (!std::isprint(static_cast<unsigned char>(c)) && !std::isspace(static_cast<unsigned char>(c))) {
      isValid = false;
      break;
    }
  }
  if (!isValid || rawWaveModel.empty()) {
    this->data->waveModelName = "nezha_ocean_waves";  
  } else {
    this->data->waveModelName = rawWaveModel;
  }

  this->data->hydroParams.reset(new HydrodynamicsParameters());
  
  
  // 1. robot_volume (max_displaced_volume)
  double robotVolume = 0.0;
  if (_sdf->HasElement("robot_volume")) {
    robotVolume = _sdf->Get<double>("robot_volume");
    gzmsg << " [Load] Read robot_volume from SDF: " << std::scientific 
          << std::setprecision(4) << robotVolume << " m" << std::endl;
  } 
  
  // 2. buoyancy_scale
  double buoyancyScale = 1.0; 
  if (_sdf->HasElement("buoyancy_scale")) {
    buoyancyScale = _sdf->Get<double>("buoyancy_scale");
    gzmsg << " [Load] Read buoyancy_scale from SDF: " << std::fixed 
          << std::setprecision(3) << buoyancyScale << std::endl;
  } else {
    gzmsg << "  [Load] buoyancy_scale not found, using default: " 
          << buoyancyScale << std::endl;
  }
  
  // 3. force_scale_factor
  double forceScaleFactor = 1.0;  
  if (_sdf->HasElement("force_scale_factor")) {
    forceScaleFactor = _sdf->Get<double>("force_scale_factor");
    gzmsg << " [Load] Read force_scale_factor from SDF: " << std::fixed 
          << std::setprecision(3) << forceScaleFactor << std::endl;
  } else {
    gzmsg << "  [Load] force_scale_factor not found, using default: " 
          << forceScaleFactor << std::endl;
  }
  // ==========  Fossen  ==========
  if (_sdf->HasElement("hydrodynamic_model"))
  {
    sdf::ElementPtr hydroModel = _sdf->GetElement("hydrodynamic_model");
    
    // 1.  (36 )
    if (hydroModel->HasElement("added_mass"))
    {
      std::vector<double> Ma_vec = Str2Vector(
        hydroModel->Get<std::string>("added_mass"));
      
      if (Ma_vec.size() == 36)
      {
        for (int i = 0; i < 6; i++)
          for (int j = 0; j < 6; j++)
            this->data->Ma(i, j) = Ma_vec[6*i + j];
        
        gzmsg << "✓ Loaded Added Mass Matrix (6x6)" << std::endl;
      }
    }
    
    // 2. 
    if (hydroModel->HasElement("linear_damping"))
    {
      std::vector<double> D_vec = Str2Vector(
        hydroModel->Get<std::string>("linear_damping"));
      
      if (D_vec.size() == 6) // 
      {
        this->data->DLin.setZero();
        for (int i = 0; i < 6; i++)
          this->data->DLin(i, i) = D_vec[i];
      }
      else if (D_vec.size() == 36) // 
      {
        for (int i = 0; i < 6; i++)
          for (int j = 0; j < 6; j++)
            this->data->DLin(i, j) = D_vec[6*i + j];
      }
      
      gzmsg << "✓ Loaded Linear Damping Matrix" << std::endl;
    }
    
    // 3. 
    if (hydroModel->HasElement("quadratic_damping"))
    {
      std::vector<double> Dq_vec = Str2Vector(
        hydroModel->Get<std::string>("quadratic_damping"));
      
      if (Dq_vec.size() == 6)
      {
        this->data->DNonLin.setZero();
        for (int i = 0; i < 6; i++)
          this->data->DNonLin(i, i) = Dq_vec[i];
      }
      else if (Dq_vec.size() == 36)
      {
        for (int i = 0; i < 6; i++)
          for (int j = 0; j < 6; j++)
            this->data->DNonLin(i, j) = Dq_vec[6*i + j];
      }
      
      gzmsg << "✓ Loaded Quadratic Damping Matrix" << std::endl;
    }
    
    // 4. 
    if (hydroModel->HasElement("scaling_added_mass"))
      this->data->scalingAddedMass = hydroModel->Get<double>("scaling_added_mass");
    
    if (hydroModel->HasElement("scaling_damping"))
      this->data->scalingDamping = hydroModel->Get<double>("scaling_damping");
  }
  else
  {
    gzwarn << "⚠ No <hydrodynamic_model> found, using default matrices" << std::endl;
    this->data->Ma.setZero();
    this->data->DLin.setZero();
    this->data->DNonLin.setZero();
  }
  
  // 
  this->data->lastVelRel.setZero();
  this->data->filteredAcc.setZero();
  this->data->lastTime = 0.0;
  
  double waterPatchLength = 1.0;
  double waterPatchWidth = 1.0;
  if (_sdf->HasElement("length")) {
    waterPatchLength = _sdf->GetElement("length")->Get<double>();
  }
  if (_sdf->HasElement("width")) {
    waterPatchWidth = _sdf->GetElement("width")->Get<double>();
  }
  
  this->data->waterPatchLength = waterPatchLength;
  this->data->waterPatchWidth = waterPatchWidth;
  this->data->hydroParams->SetFromSDF(*_sdf);
  
  this->data->hydroParams->SetBuoyancyScale(buoyancyScale);
  this->data->hydroParams->SetForceScaleFactor(forceScaleFactor);
  
  if (robotVolume > 0.0) {
    this->data->hydroParams->SetMaxDisplacedVolume(robotVolume);
  } else {
    gzerr << " [Load] robot_volume is ZERO or not set! Hydrodynamics will NOT work!" << std::endl;
    gzerr << "    Please add <robot_volume>0.0018</robot_volume> to your XACRO file." << std::endl;
  }

  if (_sdf->HasElement("markers")) {
    sdf::ElementPtr sdfMarkers = _sdf->GetElement("markers");
    this->data->updateRate            = Utilities::SdfParamDouble(*sdfMarkers, "update_rate", 30.0);
    this->data->showWaterPatch        = Utilities::SdfParamBool(*sdfMarkers, "water_patch", false);
    this->data->showWaterline         = Utilities::SdfParamBool(*sdfMarkers, "waterline", false);
    this->data->showUnderwaterSurface = Utilities::SdfParamBool(*sdfMarkers, "underwater_surface", false);
  }

  g_lastHydrodynamicsPlugin = this;
  HydrodynamicsPluginRegistry::GetInstance().Register(this, this->data->model);
  this->enabled_ = true;
  
// ========== ASCII Art Banner ==========
gzmsg << "\n";
gzmsg << "+============================================================+\n";
gzmsg << "|                                                            |\n";
gzmsg << "|    NN   NN  EEEEEEE  ZZZZZZZ  HH   HH    AAA              |\n";
gzmsg << "|    NNN  NN  EE          ZZ    HH   HH   AAAAA             |\n";
gzmsg << "|    NN N NN  EEEEEE     ZZ     HHHHHHH  AA   AA            |\n";
gzmsg << "|    NN  NNN  EE        ZZ      HH   HH  AAAAAAA            |\n";
gzmsg << "|    NN   NN  EEEEEEE  ZZZZZZZ  HH   HH  AA   AA            |\n";
gzmsg << "|                                                            |\n";
gzmsg << "|               Nezha Sim Plugin - Water Surface Plugin      |\n";
gzmsg << "|                   Version V1 ROS - 2025                    |\n";
gzmsg << "|                                                            |\n";
gzmsg << "+============================================================+\n";
gzmsg << "\n";
  this->data->lastDebugTime = this->data->world->SimTime();
  gzmsg << "\n";
  gzmsg << "========================================\n";
  gzmsg << "  Nezha Hydrodynamics Plugin Loaded\n";
  gzmsg << "========================================\n";
  gzmsg << "Model:              " << this->data->model->GetName() << "\n";
  gzmsg << "Wave Model:         " << this->data->waveModelName << "\n";
  gzmsg << "----------------------------------------\n";
  gzmsg << "Critical Parameters:\n";
  gzmsg << "  robot_volume:     " << this->data->hydroParams->MaxDisplacedVolume() << " m\n";
  gzmsg << "  buoyancy_scale:   " << this->data->hydroParams->BuoyancyScale() << "\n";
  gzmsg << "  force_scale:      " << this->data->hydroParams->ForceScaleFactor() << "\n";
  gzmsg << "----------------------------------------\n";
  gzmsg << "Water Patch:        " << waterPatchLength << "  " << waterPatchWidth << " m\n";
  gzmsg << "Status:             " << (robotVolume > 0.0 ? "ENABLED" : "DISABLED") << "\n";
  gzmsg << "========================================\n";
  gzmsg << std::endl;
  this->RegisterForcesService();
}

void HydrodynamicsPlugin::RegisterForcesService() {
    if (!this->data->rosNode) {
        this->data->rosNode = new ros::NodeHandle("~");
    }
    
    std::string modelName = this->data->model->GetName();
    std::string serviceName = "/" + modelName + "/get_hydrodynamics_forces";
    
    // Check if already registered
    if (this->data->forcesServiceRegistered[serviceName]) {
        gzmsg << "Service already registered: " << serviceName << std::endl;
        return;
    }
    
    ros::AdvertiseServiceOptions aso = 
        ros::AdvertiseServiceOptions::create<nezha_plugins::HydrodynamicsForces>(
            serviceName,
            boost::bind(&HydrodynamicsPlugin::OnGetForcesService, this, _1, _2),
            ros::VoidPtr(),
            &this->data->rosQueue);
    
    this->data->forcesService = this->data->rosNode->advertiseService(aso);
    this->data->forcesServiceRegistered[serviceName] = true;
    
    gzmsg << "✓ HydrodynamicsPlugin Service registered: " << serviceName << std::endl;
}
bool HydrodynamicsPlugin::OnGetForcesService(
    nezha_plugins::HydrodynamicsForces::Request &req,
    nezha_plugins::HydrodynamicsForces::Response &res) {
    
    // Buoyancy Force
    res.buoyancy_x = this->data->lastBuoyancyForce.X();
    res.buoyancy_y = this->data->lastBuoyancyForce.Y();
    res.buoyancy_z = this->data->lastBuoyancyForce.Z();
    
    // Damping Force
    res.damping_x = this->data->lastDampingForce.X();
    res.damping_y = this->data->lastDampingForce.Y();
    res.damping_z = this->data->lastDampingForce.Z();
    
    // Wave Force
    res.wave_x = this->data->lastWaveForce.X();
    res.wave_y = this->data->lastWaveForce.Y();
    res.wave_z = this->data->lastWaveForce.Z();
    
    // Added Mass Force
    res.added_mass_x = this->data->lastAddedMassForce.X();
    res.added_mass_y = this->data->lastAddedMassForce.Y();
    res.added_mass_z = this->data->lastAddedMassForce.Z();
    
    // Coriolis Force
    res.coriolis_x = this->data->lastCoriolisForce.X();
    res.coriolis_y = this->data->lastCoriolisForce.Y();
    res.coriolis_z = this->data->lastCoriolisForce.Z();
    
    // Submersion Ratio
    res.submersion_ratio = this->data->lastSubmersionRatio;
    
    // Sim Time
    if (this->data->world) {
        res.sim_time = this->data->world->SimTime().Double();
    } else {
        res.sim_time = 0.0;
    }
    
    return true;
}


// ROS queue thread:
void HydrodynamicsPlugin::QueueThread() {
    static const double timeout = 0.01;
    while (this->data->rosNode) {
        this->data->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
void HydrodynamicsPlugin::OnUpdate()
{
  if (!this->enabled_) {
    return;
  }
  
  this->UpdatePhysics();
  this->UpdateMarkers();
}
void HydrodynamicsPlugin::UpdatePhysics()
{
  if (!this->enabled_) return;
  
  physics::LinkPtr baseLink = this->data->baseLink;
  if (!baseLink)
  {
    for (const auto& link : this->data->model->GetLinks())
    {
      if (link->GetName().find("base_link") != std::string::npos)
      {
        baseLink = link;
        this->data->baseLink = link;
        break;
      }
    }
  }
  
  if (!baseLink)
  {
    gzerr << "base_link not found!" << std::endl;
    return;
  }
  
  // ========== 2.  ==========
  double submersionRatio = 0.0;
  int totalTriangles = 0;
  int submergedTriangles = 0;
  
  for (auto&& hd : this->data->hydroData)
  {
    if (!hd->link || !hd->wavefieldSampler) continue;
    
    const auto linkPose = hd->link->WorldPose();
    hd->wavefieldSampler->ApplyPose(linkPose);
    hd->wavefieldSampler->UpdatePatch();
    
    for (size_t j = 0; j < hd->linkMeshes.size(); ++j)
    {
      if (!hd->hydrodynamics[j]) continue;
      
      ApplyPose(linkPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);
      
      hd->hydrodynamics[j]->Update(
        hd->wavefieldSampler,
        hd->link->WorldCoGPose(),
        ToVector3(hd->link->WorldLinearVel()),
        ToVector3(hd->link->WorldAngularVel())
      );
      
      totalTriangles += hd->linkMeshes[j]->number_of_faces();
      submergedTriangles += hd->hydrodynamics[j]->GetSubmergedTriangles().size();
    }
  }
  
  if (totalTriangles > 0)
  {
    submersionRatio = static_cast<double>(submergedTriangles) / totalTriangles;
    submersionRatio = std::max(0.0, std::min(1.0, submersionRatio));
  }

  // 
  double hydroCorrection = submersionRatio;
  const double smoothFactor = 0.2;

  if (this->data->hydroData.size() > 0)
  {
    auto& firstLinkData = this->data->hydroData[0];
    hydroCorrection = (1.0 - smoothFactor) * firstLinkData->lastBuoyancyCorrection
                    + smoothFactor * hydroCorrection;
    firstLinkData->lastBuoyancyCorrection = hydroCorrection;
  }
  
  // ========== 3.  ==========
  ignition::math::Pose3d pose = baseLink->WorldPose();
  ignition::math::Vector3d linVel = baseLink->RelativeLinearVel();
  ignition::math::Vector3d angVel = baseLink->RelativeAngularVel();
  
  ignition::math::Vector3d flowVelWorld(0, 0, 0);
  ignition::math::Vector3d flowVelBody = pose.Rot().RotateVectorReverse(flowVelWorld);
  ignition::math::Vector3d velRel = linVel - flowVelBody;
  
  Vector6d velRelBody;
  velRelBody << velRel.X(), velRel.Y(), velRel.Z(),
                angVel.X(), angVel.Y(), angVel.Z();
  
  // ========== 4.  ==========
  double currentTime = this->data->world->SimTime().Double();
  double dt = currentTime - this->data->lastTime;
  
  if (dt > 1e-6 && this->data->lastTime > 0.0)
  {
    Vector6d acc = (velRelBody - this->data->lastVelRel) / dt;
    const double alpha = 0.3;
    this->data->filteredAcc = (1.0 - alpha) * this->data->filteredAcc + alpha * acc;
  }
  
  this->data->lastVelRel = velRelBody;
  this->data->lastTime = currentTime;
  
  // ========== 5.  Fossen  ==========
  
  // 5.1 
  Matrix6d Ma_scaled = this->data->scalingAddedMass * this->data->Ma * hydroCorrection;
  Vector6d tauAddedMass = -Ma_scaled * this->data->filteredAcc;
  
  // 5.2 
  Matrix6d Ca;
  ComputeAddedCoriolisMatrix(velRelBody, Ma_scaled, Ca);
  Vector6d tauCoriolis = -Ca * velRelBody;
  
  // 5.3 
  Matrix6d D;
  ComputeDampingMatrix(velRelBody, D);
  Vector6d tauDamping = -D * velRelBody * hydroCorrection;
  
// 5.4  (✅ )
double volume = this->data->hydroParams->MaxDisplacedVolume();
double density = 1000.0;
ignition::math::Vector3d gravity = this->data->world->Gravity();

// ✅ :  ( +Z)
double buoyancyMagnitude = density * volume * std::abs(gravity.Z()) * hydroCorrection;
ignition::math::Vector3d buoyancyWorld(0, 0, buoyancyMagnitude);

// ✅  Body Frame,
// ignition::math::Vector3d buoyancyBody = pose.Rot().RotateVectorReverse(buoyancyWorld);

// ========== 6.  (,) ==========
Vector6d tauTotal = tauAddedMass + tauCoriolis + tauDamping;

  
  // ========== 6.  () ==========
// ✅ 
ignition::math::Vector3d hydroForce(
  tauTotal(0), tauTotal(1), tauTotal(2)
);

ignition::math::Vector3d totalTorque(
  tauTotal(3), tauTotal(4), tauTotal(5)
);

// ========== 7.  base_link ==========
// ✅  ()
baseLink->AddForce(buoyancyWorld);

// ✅  Body Frame 
baseLink->AddRelativeForce(hydroForce);
baseLink->AddRelativeTorque(totalTorque);
  
  // ========== 8.  ( ROS Service) ==========
this->data->lastBuoyancyForce = buoyancyWorld;

  this->data->lastAddedMassForce.Set(tauAddedMass(0), tauAddedMass(1), tauAddedMass(2));
  this->data->lastCoriolisForce.Set(tauCoriolis(0), tauCoriolis(1), tauCoriolis(2));
  this->data->lastDampingForce.Set(tauDamping(0), tauDamping(1), tauDamping(2));
  this->data->lastSubmersionRatio = submersionRatio;
this->data->lastTotalForce = hydroForce + buoyancyWorld;
this->data->lastTotalTorque = totalTorque;
  
// ========== 9.  ==========
ignition::math::Vector3d totalWaveDrag(0, 0, 0);
ignition::math::Vector3d totalWaveTorque(0, 0, 0);

// ==========  ==========
double globalWaterLevel = 0.0;
double depth = 0.0;
double waveScale = 0.0;

//  Wavefield 
if (this->data->wavefield && this->data->hydroData.size() > 0)
{
    try {
        auto& hd = this->data->hydroData[0];
        if (hd->wavefieldSampler)
        {
            auto waterPatch = hd->wavefieldSampler->GetWaterPatch();
            if (waterPatch && waterPatch->GetCellCount()[0] > 0 && waterPatch->GetCellCount()[1] > 0)
            {
                size_t cx = waterPatch->GetCellCount()[0] / 2;
                size_t cy = waterPatch->GetCellCount()[1] / 2;
                
                Triangle tri = waterPatch->GetTriangle(cx, cy, 0);
                globalWaterLevel = CGAL::to_double(tri[0].z());
            }
        }
    }
    catch (const std::exception& e) {
        gzwarn << "Failed to get water level: " << e.what() << std::endl;
    }
}
bool isWaveActive = std::abs(globalWaterLevel - this->data->lastGlobalWaterLevel) > 1e-6;
this->data->lastGlobalWaterLevel = globalWaterLevel;

// ==========  ==========
double robotZ = pose.Pos().Z();
depth = globalWaterLevel - robotZ;

// ==========  ==========
// ==========  ==========
const double WAVE_CUTOFF_DEPTH = 3.0;
const double WAVE_INFLUENCE_HEIGHT = 0.5;  // ✅ 
const double waveLength = 2.0;
const double waveNumber = 2.0 * M_PI / waveLength;

// ✅ FIX: Actually calculate the waveScale based on depth!
if (depth > WAVE_CUTOFF_DEPTH) {
    waveScale = 0.0; // Too deep, no wave influence
} else if (depth < -WAVE_INFLUENCE_HEIGHT) {
    waveScale = 0.0; // Too high above water, no wave influence
} else {
    // Exponential decay of wave energy based on depth (Deep water wave approximation)
    waveScale = std::exp(-waveNumber * std::max(0.0, depth));
}

// FIXED: Apply wave forces directly to base_link
if (waveScale > 0.01 && isWaveActive && baseLink) {

    // Calculate total wave force from all submerged surfaces
    ignition::math::Vector3d totalWaveDrag(0, 0, 0);
    ignition::math::Vector3d totalWaveTorque(0, 0, 0);
    
    for (auto&& hd : this->data->hydroData) {
        if (!hd->link) continue;
        
        for (size_t j = 0; j < hd->linkMeshes.size(); ++j) {
            if (!hd->hydrodynamics[j]) continue;
            
            // Get raw wave forces (already in world frame from Hydrodynamics class)
            auto rawWaveForce  = ToIgn(hd->hydrodynamics[j]->Force());
            auto rawWaveTorque = ToIgn(hd->hydrodynamics[j]->Torque());
            
            // Normalize by submerged triangle count
            double numSubmerged = hd->hydrodynamics[j]->GetSubmergedTriangles().size();
            double totalFaces = hd->linkMeshes[j]->number_of_faces();
            double ratio = (totalFaces > 0) ? (numSubmerged / totalFaces) : 0.0;
            
            if (numSubmerged > 0) {
                // Apply slapping factor for shallow water entry
                double slappingFactor = 1.0;
                if (ratio < 0.2 && ratio > 0.0) {
                    slappingFactor = 2.0; // Amplify impact at water entry
                }
                rawWaveForce *= slappingFactor;
                rawWaveTorque *= slappingFactor;
            }
            
            // Apply depth attenuation
            auto waveForce  = rawWaveForce  * waveScale;
            auto waveTorque = rawWaveTorque * waveScale;
            
            // Force clipping
            const double MAX_WAVE_FORCE = 500.0;
            const double MAX_WAVE_TORQUE = 100.0;
            
            if (waveForce.Length() > MAX_WAVE_FORCE) {
                waveForce = waveForce.Normalize() * MAX_WAVE_FORCE;
            }
            if (waveTorque.Length() > MAX_WAVE_TORQUE) {
                waveTorque = waveTorque.Normalize() * MAX_WAVE_TORQUE;
            }
            
            totalWaveDrag += waveForce;
            totalWaveTorque += waveTorque;
        }
    }
    
    // ✅ KEY FIX: Apply to baseLink instead of hd->link
    if (totalWaveDrag.Length() > 1e-6) {
        baseLink->AddForce(totalWaveDrag);
    }
    if (totalWaveTorque.Length() > 1e-6) {
        baseLink->AddTorque(totalWaveTorque);
    }
    
    this->data->lastWaveForce = totalWaveDrag;
}



// 
this->data->lastWaveForce = totalWaveDrag;


  
  // ========== 10.  ==========
  common::Time currentSimTime = this->data->world->SimTime();
  double elapsedTime = (currentSimTime - this->data->lastDebugTime).Double();
  

}
double HydrodynamicsPlugin::ComputeDynamicWaveClamp(
    double _submersionRatio,
    const ignition::math::Vector3d& _velocity) const
{
    // ==========  ==========
    const double BASE_CLAMP = 500.0;  // 
    const double MIN_CLAMP = 50.0;    // ()
    const double MAX_CLAMP = 1000.0;  // ()
    
    // ========== 1.  ==========
    // ,()
    double submersionFactor;
    if (_submersionRatio < 0.1) {
        // : 
        submersionFactor = 1.0;
    } else if (_submersionRatio < 0.5) {
        // : 
        submersionFactor = 1.0 - 0.6 * (_submersionRatio - 0.1) / 0.4;
    } else if (_submersionRatio < 0.9) {
        // : 
        submersionFactor = 0.4 - 0.3 * (_submersionRatio - 0.5) / 0.4;
    } else {
        // : 
        submersionFactor = 0.1;
    }
    
    // ========== 2.  ==========
    double speed = _velocity.Length();
    double speedFactor;
    
    if (speed < 0.5) {
        // : 
        speedFactor = 1.0;
    } else if (speed < 2.0) {
        // : 
        speedFactor = 1.0 - 0.4 * (speed - 0.5) / 1.5;
    } else {
        // : ,
        speedFactor = 0.6 * std::exp(-0.3 * (speed - 2.0));
    }
    
    // ========== 3.  ==========
    double dynamicClamp = BASE_CLAMP * submersionFactor * speedFactor;
    
    // 
    dynamicClamp = std::clamp(dynamicClamp, MIN_CLAMP, MAX_CLAMP);
    
    return dynamicClamp;
}

// 
void HydrodynamicsPlugin::ComputeAddedCoriolisMatrix(
  const Vector6d& vel,
  const Matrix6d& Ma,
  Matrix6d& Ca) const
{
  Vector6d ab = Ma * vel;
  
  Matrix3d Sa;
  Sa <<        0, -ab(2),  ab(1),
         ab(2),       0, -ab(0),
        -ab(1),  ab(0),       0;
  
  Matrix3d Sb;
  Sb <<        0, -ab(5),  ab(4),
         ab(5),       0, -ab(3),
        -ab(4),  ab(3),       0;
  
  Ca << Matrix3d::Zero(), Sa,
        Sa, Sb;
}

// 
void HydrodynamicsPlugin::ComputeDampingMatrix(
  const Vector6d& vel,
  Matrix6d& D) const
{
  // 
  D = this->data->scalingDamping * 
      (this->data->DLin + this->data->offsetLinearDamping * Matrix6d::Identity());
  
  // 
  for (int i = 0; i < 6; i++)
  {
    D(i, i) += (this->data->DNonLin(i, i) + this->data->offsetNonLinDamping) * 
               std::abs(vel(i));
  }
}

  
  void HydrodynamicsPlugin::UpdateMarkers()
  {
    std::string topicName("/marker");

    auto updatePeriod = 1.0/this->data->updateRate;
    auto currentTime = this->data->world->SimTime();
    if ((currentTime - this->data->prevTime).Double() < updatePeriod)
    {
      return;
    }
    this->data->prevTime = currentTime; 

    if (this->data->showWaterPatch)      
      this->UpdateWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->UpdateWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->UpdateUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::UpdateWaterPatchMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      auto& grid = *hd->wavefieldSampler->GetWaterPatch();

      hd->waterPatchMsg.mutable_point()->Clear();
      for (size_t ix=0; ix<grid.GetCellCount()[0]; ++ix)
      {
        for (size_t iy=0; iy<grid.GetCellCount()[1]; ++iy)
        {
          for (size_t k=0; k<2; ++k)
          {
            Triangle tri = grid.GetTriangle(ix, iy, k);
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[0]));
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[1]));
            ignition::msgs::Set(hd->waterPatchMsg.add_point(), ToIgn(tri[2]));
          }
        }
      }
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    }
  }

  void HydrodynamicsPlugin::UpdateWaterlineMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].mutable_point()->Clear();
        if (hd->hydrodynamics[j]->GetWaterline().empty())
        {
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        }
        for (auto&& line : hd->hydrodynamics[j]->GetWaterline())
        {
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ToIgn(line.point(0)));
          ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ToIgn(line.point(1)));
        }
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    }
  }

  void HydrodynamicsPlugin::UpdateUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
        if (hd->hydrodynamics[j]->GetSubmergedTriangles().empty())
        {
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        }
        for (auto&& tri : hd->hydrodynamics[j]->GetSubmergedTriangles())
        {
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[0]));
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[1]));
          ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ToIgn(tri[2]));
        }
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    }
  }

  void HydrodynamicsPlugin::Init()
  {

    this->HydrodynamicsPlugin::InitPhysics();
    this->HydrodynamicsPlugin::InitMarkers();
  }

void HydrodynamicsPlugin::InitPhysics()
{
  this->data->wavefield = GetWavefield(
    this->data->world, this->data->waveModelName);
  if (this->data->wavefield == nullptr) 
  {
    gzerr << "+------------------------------------------------------------+" << std::endl;
    gzerr << "|  ERROR: Wavefield is NULL                                  |" << std::endl;
    gzerr << "|  Wave model: " << std::setw(44) << std::left 
          << this->data->waveModelName << " |" << std::endl;
    gzerr << "+------------------------------------------------------------+" << std::endl;
    return;
  }


  std::string modelName(this->data->model->GetName());
  gzmsg << "+------------------------------------------------------------+" << std::endl;
  gzmsg << "|  Initializing HydrodynamicsPlugin                          |" << std::endl;
  gzmsg << "+------------------------------------------------------------+" << std::endl;
  gzmsg << "|  Model: " << std::setw(49) << std::left 
        << modelName << " |" << std::endl;
  gzmsg << "+------------------------------------------------------------+" << std::endl;

  std::vector<physics::LinkPtr> links;
  std::vector<std::vector<std::shared_ptr<Mesh>>> meshes;
  
  gzmsg << "[DEBUG] Before CreateCollisionMeshes:" << std::endl;
  gzmsg << "   links.size() = " << links.size() << std::endl;
  gzmsg << "   meshes.size() = " << meshes.size() << std::endl;
  
  CreateCollisionMeshes(this->data->model, links, meshes);
  
  gzmsg << "[DEBUG] After CreateCollisionMeshes:" << std::endl;
  gzmsg << "   links.size() = " << links.size() << std::endl;
  gzmsg << "   meshes.size() = " << meshes.size() << std::endl;
  
  
  for (size_t i=0; i<links.size(); ++i)
  {
    std::string linkName = links[i]->GetName();
    size_t meshCount = meshes[i].size();
    
    std::shared_ptr<HydrodynamicsLinkData> hd(new HydrodynamicsLinkData);
    this->data->hydroData.push_back(hd);
    hd->initLinkMeshes.resize(meshCount);
    hd->linkMeshes.resize(meshCount);
    hd->hydrodynamics.resize(meshCount);
    hd->waterlineMsgs.resize(meshCount);
    hd->underwaterSurfaceMsgs.resize(meshCount);
    hd->link = links[i];

    ignition::math::Pose3d linkPose;
    ignition::math::Pose3d linkCoMPose;
    
    try
    {
      linkPose = hd->link->WorldPose();
      linkCoMPose = hd->link->WorldCoGPose();
    }
    catch (const std::exception& e)
    {
      gzerr << "Failed to get pose for link " << linkName 
            << ": " << e.what() << std::endl;
      continue;
    }

    const double MAX_VALID_COORD = 100000.0;
    const ignition::math::Vector3d pos = linkPose.Pos();
    
    bool poseValid = std::isfinite(pos.X()) && 
                     std::isfinite(pos.Y()) && 
                     std::isfinite(pos.Z()) &&
                     std::abs(pos.X()) < MAX_VALID_COORD &&
                     std::abs(pos.Y()) < MAX_VALID_COORD &&
                     std::abs(pos.Z()) < MAX_VALID_COORD;

    if (!poseValid)
    {
      continue;
    }



double patchLength = this->data->waterPatchLength;
double patchWidth = this->data->waterPatchWidth;

size_t gridResX, gridResY;

if (patchLength < 0.5) {
  gridResX = std::max(size_t(6), size_t(std::ceil(patchLength / 0.15)));  // 0.05→0.15
} else if (patchLength < 2.0) {
  gridResX = std::max(size_t(6), size_t(std::ceil(patchLength / 0.30)));  // 0.10→0.30
} else {
  gridResX = std::max(size_t(4), size_t(std::ceil(patchLength / 0.50)));  // 0.25→0.50
}

// Y 
if (patchWidth < 0.5) {
  gridResY = std::max(size_t(6), size_t(std::ceil(patchWidth / 0.15)));
} else if (patchWidth < 2.0) {
  gridResY = std::max(size_t(6), size_t(std::ceil(patchWidth / 0.30)));
} else {
  gridResY = std::max(size_t(4), size_t(std::ceil(patchWidth / 0.50)));
}

// ✅ :
gridResX = std::min(gridResX, size_t(16));  //  64 → 16
gridResY = std::min(gridResY, size_t(16));  //  64 → 16

gzmsg << "+------------------------------------------------------------+" << std::endl;
gzmsg << "|  Adaptive Grid Resolution                                  |" << std::endl;
gzmsg << "+------------------------------------------------------------+" << std::endl;
gzmsg << "|  Patch Size:   " << std::setw(5) << std::fixed << std::setprecision(2) 
      << patchLength << " x " << std::setw(5) << patchWidth << " m            |" << std::endl;
gzmsg << "|  Grid Cells:   " << std::setw(3) << gridResX << " x " 
      << std::setw(3) << gridResY << "                      |" << std::endl;
gzmsg << "|  Total Triangles: " << std::setw(5) << (gridResX * gridResY * 2) 
      << "                                  |" << std::endl;
gzmsg << "|  Cell Size:    " << std::setw(5) << std::fixed << std::setprecision(3)
      << (patchLength / gridResX) << " x " << std::setw(5) 
      << (patchWidth / gridResY) << " m            |" << std::endl;
gzmsg << "+------------------------------------------------------------+" << std::endl;

// ==========  Grid ==========
std::shared_ptr<Grid> initWaterPatch;
try
{
  initWaterPatch = std::make_shared<Grid>(
    std::array<double, 2>{patchLength, patchWidth},
    std::array<size_t, 2>{gridResX, gridResY}  
  );
  
  gzmsg << " Grid created successfully with " 
        << (gridResX * gridResY * 2) << " triangles" << std::endl;
}
catch (const std::exception& e)
{
  gzerr << "Failed to create Grid: " << e.what() << std::endl;
  gzerr << "   Falling back to 8x8 grid" << std::endl;
  
  initWaterPatch = std::make_shared<Grid>(
    std::array<double, 2>{patchLength, patchWidth},
    std::array<size_t, 2>{8, 8}
  );
}

    try
    {
      hd->wavefieldSampler.reset(new WavefieldSampler(
        this->data->wavefield, initWaterPatch));
    }
    catch (const std::exception& e)
    {
      gzerr << "Failed to create WavefieldSampler for link " << linkName 
            << ": " << e.what() << std::endl;
      continue;
    }

    try
    {
      hd->wavefieldSampler->ApplyPose(linkPose);
      hd->wavefieldSampler->UpdatePatch();
      gzmsg << "Successfully applied pose and updated patch for " << linkName << std::endl;
    }
catch (const std::exception& e)
{
  gzerr << "+------------------------------------------------------------+" << std::endl;
  gzerr << "|  Exception in ApplyPose/UpdatePatch                        |" << std::endl;
  gzerr << "+------------------------------------------------------------+" << std::endl;
  gzerr << "|  Link: " << std::setw(49) << std::left << linkName << " |" << std::endl;
  gzerr << "|  Error: " << std::setw(48) << std::left << e.what() << " |" << std::endl;
  gzerr << "+------------------------------------------------------------+" << std::endl;
  continue;
}


    Vector3 linVelocity = ToVector3(hd->link->WorldLinearVel());
    Vector3 angVelocity = ToVector3(hd->link->WorldAngularVel());


    for (size_t j=0; j<meshCount; ++j)
    {
      try
      {
        std::shared_ptr<Mesh> initLinkMesh = meshes[i][j];
        std::shared_ptr<Mesh> linkMesh = std::make_shared<Mesh>(*initLinkMesh);
        GZ_ASSERT(linkMesh != nullptr, "Invalid Mesh returned from CopyMesh");

        hd->initLinkMeshes[j] = initLinkMesh;
        hd->linkMeshes[j] = linkMesh;

        ApplyPose(linkPose, *hd->initLinkMeshes[j], *hd->linkMeshes[j]);

        hd->hydrodynamics[j].reset(
          new Hydrodynamics(
            this->data->hydroParams,
            hd->linkMeshes[j],
            hd->wavefieldSampler));
        
        hd->hydrodynamics[j]->Update(
          hd->wavefieldSampler, linkCoMPose, linVelocity, angVelocity);

        gzmsg << "  Mesh " << j << ": " 
              << linkMesh->number_of_vertices() << " vertices, "
              << linkMesh->number_of_faces() << " faces" << std::endl;
      }
      catch (const std::exception& e)
      {
        gzerr << "Failed to process mesh " << j << " for link " << linkName 
              << ": " << e.what() << std::endl;
        continue;
      }
    }

    gzmsg << "Link " << linkName << " initialized successfully." << std::endl;
    gzmsg << "----------------------------------------" << std::endl;
  }
  
  // ========== � ㄥ ==========

  // ========== � ==========
  size_t validLinks = 0;
for (auto&& hd : this->data->hydroData)
{
  if (hd->link && hd->wavefieldSampler)
    validLinks++;
}

gzmsg << "+------------------------------------------------------------+" << std::endl;
gzmsg << "|  Initialization Complete                                   |" << std::endl;
gzmsg << "+------------------------------------------------------------+" << std::endl;
gzmsg << "|  Total links:  " << std::setw(41) << std::right << links.size() << " |" << std::endl;
gzmsg << "|  Valid links:  " << std::setw(41) << std::right << validLinks << " |" << std::endl;
gzmsg << "|  Failed links: " << std::setw(41) << std::right << (links.size() - validLinks) << " |" << std::endl;
gzmsg << "+------------------------------------------------------------+" << std::endl;

if (validLinks == 0)
{
  gzerr << "+============================================================+" << std::endl;
  gzerr << "|  CRITICAL: No valid links initialized!                     |" << std::endl;
  gzerr << "|  Hydrodynamics will not function.                          |" << std::endl;
  gzerr << "+============================================================+" << std::endl;
}

}


  void HydrodynamicsPlugin::InitMarkers()
  {
    if (this->data->showWaterPatch)      
      this->InitWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->InitWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->InitUnderwaterSurfaceMarkers();    
  }

  void HydrodynamicsPlugin::InitWaterPatchMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.set_ns(modelName + "::water_patch");
      hd->waterPatchMsg.set_id(markerId++);
      hd->waterPatchMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      hd->waterPatchMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
      std::string waveMat("Gazebo/BlueTransparent");
      ignition::msgs::Material *waveMatMsg = hd->waterPatchMsg.mutable_material();
      GZ_ASSERT(waveMatMsg != nullptr, "Invalid Material pointer from waterPatchMsg");
      waveMatMsg->mutable_script()->set_name(waveMat);
    }
  }

  void HydrodynamicsPlugin::InitWaterlineMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].set_ns(modelName + "::waterline");
        hd->waterlineMsgs[j].set_id(markerId++);
        hd->waterlineMsgs[j].set_action(ignition::msgs::Marker::ADD_MODIFY);
        hd->waterlineMsgs[j].set_type(ignition::msgs::Marker::LINE_LIST);
        std::string lineMat("Gazebo/Black");
        ignition::msgs::Material *lineMatMsg = hd->waterlineMsgs[j].mutable_material();
        GZ_ASSERT(lineMatMsg != nullptr, "Invalid Material pointer from waterlineMsgs");
        lineMatMsg->mutable_script()->set_name(lineMat);
      }
    }
  }

  void HydrodynamicsPlugin::InitUnderwaterSurfaceMarkers()
  {
    std::string modelName(this->data->model->GetName());
    int markerId = 0;
    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].set_ns(modelName + "::submerged_triangles");
        hd->underwaterSurfaceMsgs[j].set_id(markerId++);
        hd->underwaterSurfaceMsgs[j].set_action(ignition::msgs::Marker::ADD_MODIFY);
        hd->underwaterSurfaceMsgs[j].set_type(ignition::msgs::Marker::TRIANGLE_LIST);
        std::string triMat("Gazebo/Blue");
        ignition::msgs::Material *triMatMsg = hd->underwaterSurfaceMsgs[j].mutable_material();
        GZ_ASSERT(triMatMsg != nullptr, "Invalid Material pointer from underwaterSurfaceMsgs");
        triMatMsg->mutable_script()->set_name(triMat);      
      }
    }
  }

  void HydrodynamicsPlugin::Reset()
  {

    this->data->prevTime = this->data->world->SimTime(); 

    this->ResetPhysics();
    this->ResetMarkers();
  }

  void HydrodynamicsPlugin::ResetPhysics()
  {

  }

  void HydrodynamicsPlugin::ResetMarkers()
  {

    if (this->data->showWaterPatch)
      this->ResetWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->ResetWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->ResetUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::ResetWaterPatchMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.mutable_point()->Clear();
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      ignition::msgs::Set(hd->waterPatchMsg.add_point(), ignition::math::Vector3d::Zero);
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    } 
  }

  void HydrodynamicsPlugin::ResetWaterlineMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].mutable_point()->Clear();
        ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->waterlineMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    } 
  }

  void HydrodynamicsPlugin::ResetUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].mutable_point()->Clear();
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        ignition::msgs::Set(hd->underwaterSurfaceMsgs[j].add_point(), ignition::math::Vector3d::Zero);
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    } 
  }

  void HydrodynamicsPlugin::Fini()
  {
    this->FiniPhysics();
    this->FiniMarkers();
  }

  void HydrodynamicsPlugin::FiniPhysics()
  {

  }

  void HydrodynamicsPlugin::FiniMarkers()
  {
    if (this->data->showWaterPatch)      
      this->FiniWaterPatchMarkers();

    if (this->data->showWaterline)  
      this->FiniWaterlineMarkers();

    if (this->data->showUnderwaterSurface)  
      this->FiniUnderwaterSurfaceMarkers();
  }

  void HydrodynamicsPlugin::FiniWaterPatchMarkers()
  {    
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      hd->waterPatchMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
      this->data->ignNode.Request(topicName, hd->waterPatchMsg);
    }    
  }

  void HydrodynamicsPlugin::FiniWaterlineMarkers()
  {    
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->waterlineMsgs[j].set_action(ignition::msgs::Marker::DELETE_MARKER);
        this->data->ignNode.Request(topicName, hd->waterlineMsgs[j]);
      }
    }    
  }

  void HydrodynamicsPlugin::FiniUnderwaterSurfaceMarkers()
  {
    std::string topicName("/marker");

    for (auto&& hd : this->data->hydroData)
    {
      for (size_t j=0; j<hd->linkMeshes.size(); ++j)
      {
        hd->underwaterSurfaceMsgs[j].set_action(ignition::msgs::Marker::DELETE_MARKER);
        this->data->ignNode.Request(topicName, hd->underwaterSurfaceMsgs[j]);
      }
    }    
  }

void HydrodynamicsPlugin::OnHydrodynamicsMsg(ConstParam_VPtr &_msg)
{
  GZ_ASSERT(_msg != nullptr, "Hydrodynamics message must not be null");

  auto& hydroParams = *this->data->hydroParams;
  
  hydroParams.SetFromMsg(*_msg);

  hydroParams.DebugPrint();
}



void HydrodynamicsPlugin::SetEnabled(bool enable)
{
    this->enabled_ = enable;
   
    if (!enable && this->data)
    {
        this->data->lastTotalForce.Set(0, 0, 0);
        this->data->lastTotalTorque.Set(0, 0, 0);
        
    }
}


bool HydrodynamicsPlugin::IsEnabled() const
{
    return this->enabled_;
}

void HydrodynamicsPlugin::ClearLastForces()
{
    if (this->data)
    {
        this->data->lastTotalForce.Set(0, 0, 0);
        this->data->lastTotalTorque.Set(0, 0, 0);
    }
}

ignition::math::Vector3d HydrodynamicsPlugin::GetLastTotalForce() const
{
    if (!this->enabled_)
    {
        return ignition::math::Vector3d::Zero;
    }
    
    return this->data ? this->data->lastTotalForce : ignition::math::Vector3d::Zero;
}

ignition::math::Vector3d HydrodynamicsPlugin::GetLastTotalTorque() const
{
    if (!this->enabled_)
    {
        return ignition::math::Vector3d::Zero;
    }
    
    return this->data ? this->data->lastTotalTorque : ignition::math::Vector3d::Zero;
}


} 

