
#include "nezha_underwaterHydrodynamicsPlugin.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <set> 
#include <mutex>
namespace gazebo
{
    static std::set<std::string> g_registered_services;
    static std::mutex g_service_mutex;
 void HydrodynamicModelRegistry::Register(HydrodynamicModel* model)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        models_.push_back(model);
        gzmsg << "[Registry] Registered model (total: " << models_.size() << ")" << std::endl;
    }

    void HydrodynamicModelRegistry::Unregister(HydrodynamicModel* model)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = std::find(models_.begin(), models_.end(), model);
        if (it != models_.end()) {
            models_.erase(it);
        }
    }

    std::vector<HydrodynamicModel*> HydrodynamicModelRegistry::GetModels()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return models_;  
    }

    size_t HydrodynamicModelRegistry::Size()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return models_.size();
    }

    HydrodynamicModel::HydrodynamicModel(sdf::ElementPtr _sdf,
                                         physics::LinkPtr _link) : BuoyantObject(_link)
    {
        GZ_ASSERT(_link != NULL, "Invalid link pointer");

        // ==========================================================================
        // [修复 1] 恢复丢失的初始化代码 (防止变量未初始化导致物理引擎崩溃)
        // ==========================================================================
        this->filteredAcc.setZero();
        this->lastVelRel.setZero();
        this->lastTime = 0.0;
        
        // 如果有 volume 参数，通常在这里读取，这里先设为默认防止未定义行为
        // 具体取决于您的头文件定义，但初始化为0是安全的起点
        
        // ==========================================================================
        // [修复 2] 确保 ROS 已初始化 (防止创建 NodeHandle 时崩溃)
        // ==========================================================================
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_hydrodynamics_model", 
                      ros::init_options::NoSigintHandler);
        }

        this->rosNode = new ros::NodeHandle("~");

        // ==========================================================================
        // [修复 3] 避免服务名称冲突 (保留您之前的修复)
        // ==========================================================================
        std::string linkName = this->link->GetName();
        std::string modelName = this->link->GetModel()->GetName();
        
        // 为每个 Link 使用唯一的服务名称
        std::string serviceName = "/" + modelName + "/" + linkName + "/get_hydrodynamics_forces";
        
        // [已注释] 防止与 nezha_surfacePlugin 冲突
        // if (linkName.find("base_link") != std::string::npos) {
        //      serviceName = "/" + modelName + "/get_hydrodynamics_forces";
        // }

        bool should_register = false;
        {
            std::lock_guard<std::mutex> lock(g_service_mutex);
            if (g_registered_services.find(serviceName) == g_registered_services.end())
            {
                g_registered_services.insert(serviceName);
                should_register = true;
            }
        }

        if (should_register)
        {
            ros::AdvertiseServiceOptions aso = 
                ros::AdvertiseServiceOptions::create<nezha_plugins::HydrodynamicsForces>(
                    serviceName,
                    boost::bind(&HydrodynamicModel::OnGetForcesService, this, _1, _2),
                    ros::VoidPtr(),
                    &this->rosQueue);
            
            this->forcesService = this->rosNode->advertiseService(aso);
            
            // 启动线程
            this->rosQueueThread = std::thread(
                std::bind(&HydrodynamicModel::QueueThread, this)
            );
            
            gzmsg << "✓ [Success] Service registered: " << serviceName << std::endl;
        }
        else
        {
             gzmsg << "- [Info] Service " << serviceName << " already handled." << std::endl;
        }
        
        // 注册到单例注册表
        HydrodynamicModelRegistry::GetInstance().Register(this);
    }


    // --- [NEW] Proper Destructor ---
    HydrodynamicModel::~HydrodynamicModel()
    {
        // 1. Unregister from Singleton first
        HydrodynamicModelRegistry::GetInstance().Unregister(this);

        // 2. Stop and Join the ROS Thread to prevent memory corruption
        if (this->rosNode) {
            this->rosNode->shutdown();
        }
        
        this->rosQueue.clear();
        this->rosQueue.disable();
        
        if (this->rosQueueThread.joinable()) {
            this->rosQueueThread.join();
        }
        
        // 3. Clean up pointers
        if (this->rosNode) {
            delete this->rosNode;
            this->rosNode = nullptr;
        }
    }

void HydrodynamicModel::ComputeAcc(Eigen::Vector6d _velRel, double _time,
                                  double _alpha)
{
  // Compute Fossen's nu-dot numerically. We have to do this for now since
  // Gazebo reports angular accelerations that are off by orders of magnitude.
  double dt = _time - lastTime;

  if (dt <= 0.0)  // Extra caution to prevent division by zero
    return;

  Eigen::Vector6d acc = (_velRel - this->lastVelRel) / dt;

  // TODO  We only have access to the acceleration of the previous simulation
  //       step. The added mass will induce a strong force/torque counteracting
  //       it in the current simulation step. This can lead to an oscillating
  //       system.
  //       The most accurate solution would probably be to first compute the
  //       latest acceleration without added mass and then use this to compute
  //       added mass effects. This is not how gazebo works, though.
  this->filteredAcc = (1.0 - _alpha) * this->filteredAcc + _alpha * acc;

  lastTime = _time;
  this->lastVelRel = _velRel;
}



ignition::math::Vector3d HydrodynamicModel::ToNED(ignition::math::Vector3d _vec)
{
  ignition::math::Vector3d output = _vec;
  output.Y() = -1 * output.Y();
  output.Z() = -1 * output.Z();
  return output;
}

ignition::math::Vector3d HydrodynamicModel::FromNED(ignition::math::Vector3d _vec)
{
  return this->ToNED(_vec);
}

bool HydrodynamicModel::CheckParams(sdf::ElementPtr _sdf)
{
  if (this->params.empty()) return true;

  for (auto tag : this->params)
  {
    if (!_sdf->HasElement(tag))
      {
        gzerr << "Hydrodynamic model: Expected element " <<
           tag << std::endl;
        return false;
      }
  }

  return true;
}

void HydrodynamicModel::ComputeFullForces(
    double time,
    const ignition::math::Vector3d &_flowVelWorld,
    ForceReport& outReport)
{

    outReport.totalForce = ignition::math::Vector3d::Zero;
    outReport.totalTorque = ignition::math::Vector3d::Zero;
    outReport.dampingForce = ignition::math::Vector3d::Zero;
    outReport.dampingTorque = ignition::math::Vector3d::Zero;
    outReport.addedMassForce = ignition::math::Vector3d::Zero;
    outReport.addedMassTorque = ignition::math::Vector3d::Zero;
    outReport.coriolisForce = ignition::math::Vector3d::Zero;
    outReport.coriolisTorque = ignition::math::Vector3d::Zero;

    // ================= 修正后的代码 =================
    // 1. 先获取当前姿态
    ignition::math::Pose3d pose;
#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->link->WorldPose();
#else
    pose = this->link->GetWorldPose();
#endif

    // 2. 定义变量来接收计算结果
    ignition::math::Vector3d buoyancyForce = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d buoyancyTorque = ignition::math::Vector3d::Zero;

    // 3. 调用父类函数 (传入 Pose, 输出 Force 和 Torque)
    this->GetBuoyancyForce(pose, buoyancyForce, buoyancyTorque);

    // 4. 将世界坐标系的浮力转换到机体坐标系 (Body Frame)
    outReport.buoyancyForce = pose.Rot().RotateVectorReverse(buoyancyForce);
    // ==============================================

    outReport.stamp = time;
    outReport.inBodyFrame = true;
}


HydrodynamicModel::ForceReport HydrodynamicModel::UpdateForces(
    double time, const ignition::math::Vector3d &_flowVelWorld)
{


    static int callCount = 0;
    if (++callCount % 100 == 0)  
    {
    }
    
    if (!this->enabled_)
    {

        return this->lastForceReport;
    }
  ForceReport rep;

  this->ComputeFullForces(time, _flowVelWorld, rep);


  this->lastForceReport = rep;
  return rep;
}


HydrodynamicModel * HydrodynamicModelFactory::CreateHydrodynamicModel(
    sdf::ElementPtr _sdf, physics::LinkPtr _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");
  sdf::ElementPtr sdfModel = _sdf->GetElement("hydrodynamic_model");
  if (!sdfModel->HasElement("type"))
  {
    std::cerr << "Model has no type" << std::endl;
    return NULL;
  }

  std::string identifier = sdfModel->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create HydrodynamicModel with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf, _link);
}

HydrodynamicModelFactory& HydrodynamicModelFactory::GetInstance()
{
  static HydrodynamicModelFactory instance;
  return instance;
}

bool HydrodynamicModelFactory::RegisterCreator(const std::string& _identifier,
                               HydrodynamicModelCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering HydrodynamicModel with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered HydrodynamicModel type " << _identifier << std::endl;
  return true;
}


const std::string HMFossen::IDENTIFIER = "fossen";

static bool HMFossen_registered = 
    HydrodynamicModelFactory::GetInstance().RegisterCreator(
        HMFossen::IDENTIFIER, 
        &HMFossen::create
    );

HydrodynamicModel* HMFossen::create(sdf::ElementPtr _sdf,
                                    physics::LinkPtr _link)
{
  return new HMFossen(_sdf, _link);
}

HMFossen::HMFossen(sdf::ElementPtr _sdf,
                   physics::LinkPtr _link)
                  : HydrodynamicModel(_sdf, _link)
{
  std::vector<double> addedMass(36, 0.0);
  std::vector<double> linDampCoef(6, 0.0);
  std::vector<double> linDampForward(6, 0.0);
  std::vector<double> quadDampCoef(6, 0.0);

  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");
  if (modelParams->HasElement("max_force"))
  this->maxForce_ = modelParams->Get<double>("max_force");
 else
   this->maxForce_ = 0.0; 
 if (modelParams->HasElement("max_torque"))
   this->maxTorque_ = modelParams->Get<double>("max_torque");
 else
   this->maxTorque_ = 0.0;


  if (modelParams->HasElement("added_mass"))
    addedMass = Str2Vector(modelParams->Get<std::string>("added_mass"));
  else
    gzmsg << "HMFossen: Using added mass NULL" << std::endl;

  this->params.push_back("added_mass");

  if (modelParams->HasElement("linear_damping"))
    linDampCoef = Str2Vector(modelParams->Get<std::string>("linear_damping"));
  else
    gzmsg << "HMFossen: Using linear damping NULL" << std::endl;

  this->params.push_back("scaling_added_mass");

  this->scalingAddedMass = 1.0;

  this->params.push_back("offset_added_mass");

  this->offsetAddedMass = 0.0;

  this->params.push_back("linear_damping");

  if (modelParams->HasElement("linear_damping_forward_speed"))
    linDampForward = Str2Vector(
      modelParams->Get<std::string>("linear_damping_forward_speed"));
  else
    gzmsg << "HMFossen: Using linear damping for forward speed NULL"
      << std::endl;

  this->params.push_back("linear_damping_forward_speed");

  if (modelParams->HasElement("quadratic_damping"))
    quadDampCoef = Str2Vector(
        modelParams->Get<std::string>("quadratic_damping"));
  else
    gzmsg << "HMFossen: Using quad damping NULL" << std::endl;

  this->params.push_back("quadratic_damping");

  this->params.push_back("scaling_damping");

  this->scalingDamping = 1.0;

  this->params.push_back("offset_linear_damping");

  this->offsetLinearDamping = 0.0;

  this->params.push_back("offset_lin_forward_speed_damping");

  this->offsetLinForwardSpeedDamping = 0.0;

  this->params.push_back("offset_nonlin_damping");

  this->offsetNonLinDamping = 0.0;

  this->params.push_back("volume");
if (modelParams->HasElement("volume")) {
    this->volume = modelParams->Get<double>("volume");
} else {
    // 如果没有设置，默认为 0，并打印警告
    this->volume = 0.0;
}
  this->params.push_back("scaling_volume");

  GZ_ASSERT(addedMass.size() == 36,
            "Added-mass coefficients vector must have 36 elements");
  GZ_ASSERT(linDampCoef.size() == 6 || linDampCoef.size() == 36,
            "Linear damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");
  GZ_ASSERT(linDampForward.size() == 6 || linDampForward.size() == 36,
            "Linear damping coefficients proportional to the forward speed "
            "vector must have 6 elements for a diagonal matrix or 36 elements"
            " for a full matrix");
  GZ_ASSERT(quadDampCoef.size() == 6 || quadDampCoef.size() == 36,
            "Quadratic damping coefficients vector must have 6 elements for a "
            "diagonal matrix or 36 elements for a full matrix");

  this->DLin.setZero();
  this->DNonLin.setZero();
  this->DLinForwardSpeed.setZero();

  for (int row = 0; row < 6; row++)
    for (int col = 0; col < 6; col++)
    {

      this->Ma(row, col) = addedMass[6*row+col];
      if (linDampCoef.size() == 36)
        this->DLin(row, col) = linDampCoef[6*row+col];
      if (quadDampCoef.size() == 36)
        this->DNonLin(row, col) = quadDampCoef[6*row+col];
      if (linDampForward.size() == 36)
        this->DLinForwardSpeed(row, col) = linDampForward[6*row+col];
    }

  for (int i = 0; i < 6; i++)
  {
    if (linDampCoef.size() == 6)
      this->DLin(i, i) = linDampCoef[i];
    if (quadDampCoef.size() == 6)
      this->DNonLin(i, i) = quadDampCoef[i];
    if (linDampForward.size() == 6)
      this->DLinForwardSpeed(i, i) = linDampForward[i];
  }

  this->linearDampCoef = linDampCoef;
  this->quadDampCoef = quadDampCoef;
double mass;
#if GAZEBO_MAJOR_VERSION >= 8
  mass = this->link->GetInertial()->Mass();
#else
  mass = this->link->GetInertial()->GetMass();
#endif

const double maxMaRatio = 10.0;
const double maxDampingCoef = 1000.0;  
const double minMass = 0.001;  


if (mass < minMass)
{
  gzwarn << "Link mass too small (" << mass << " kg), setting to minimum: " 
         << minMass << " kg" << std::endl;
  mass = minMass;
}


double maxMa = this->Ma.cwiseAbs().maxCoeff();
if (maxMa > maxMaRatio * mass)
{
  double scaleFactor = (maxMaRatio * mass) / maxMa;
  this->Ma *= scaleFactor;
  gzmsg << "Added mass matrix scaled by factor: " << scaleFactor << std::endl;
}


for (int i = 0; i < 6; i++)
{
  for (int j = 0; j < 6; j++)
  {
    if (std::abs(this->Ma(i, j)) > maxMaRatio * mass)
    {
      this->Ma(i, j) = std::copysign(maxMaRatio * mass, this->Ma(i, j));
    }
  }
}


double maxDLin = this->DLin.cwiseAbs().maxCoeff();
if (maxDLin > maxDampingCoef)
{
  double scaleFactor = maxDampingCoef / maxDLin;
  this->DLin *= scaleFactor;
  gzmsg << "Linear damping matrix scaled by factor: " << scaleFactor << std::endl;
}


for (int i = 0; i < 6; i++)
{
  for (int j = 0; j < 6; j++)
  {
    if (std::abs(this->DLin(i, j)) > maxDampingCoef)
    {
      this->DLin(i, j) = std::copysign(maxDampingCoef, this->DLin(i, j));
    }
  }
}


double maxDNonLin = this->DNonLin.cwiseAbs().maxCoeff();
if (maxDNonLin > maxDampingCoef)
{
  double scaleFactor = maxDampingCoef / maxDNonLin;
  this->DNonLin *= scaleFactor;
  gzmsg << "Nonlinear damping matrix scaled by factor: " << scaleFactor << std::endl;
}


for (int i = 0; i < 6; i++)
{
  for (int j = 0; j < 6; j++)
  {
    if (std::abs(this->DNonLin(i, j)) > maxDampingCoef)
    {
      this->DNonLin(i, j) = std::copysign(maxDampingCoef, this->DNonLin(i, j));
    }
  }
}

for (int i = 0; i < 6; i++)
{
  for (int j = 0; j < 6; j++)
  {
    if (std::abs(this->DLinForwardSpeed(i, j)) > maxDampingCoef)
    {
      this->DLinForwardSpeed(i, j) = std::copysign(maxDampingCoef, 
                                                    this->DLinForwardSpeed(i, j));
    }
  }
}

{
    auto models = HydrodynamicModelRegistry::GetInstance().GetModels();
    
    gzmsg << "✓ HMFossen 构造完成（基类已注册）" << std::endl;
    gzmsg << "  当前注册表大小: " << models.size() << std::endl;

    gzmsg << "  注册表内容:" << std::endl;
    for (size_t i = 0; i < models.size(); ++i) {
        auto* model = models[i];
        if (model && model->GetLink()) {
            gzmsg << "    [" << i << "] " 
                  << model->GetLink()->GetScopedName() 
                  << " @ " << model << std::endl;
        } else {
            gzmsg << "    [" << i << "] NULL 或无效指针" << std::endl;
        }
    }
}

    
    gzmsg << "════════════════════════════════════════" << std::endl;
    }

void HydrodynamicModel::ResetStateForBelow()
{
  this->filteredAcc.setZero();
  this->lastVelRel.setZero();
  this->lastTime = 0.0;
}


void HMFossen::ApplyHydrodynamicForces(
    double time, const ignition::math::Vector3d &_flowVelWorld)
{
    // 可选: 保留禁用检查
    if (!enabled_) {
        return;
    }

    HydrodynamicModel::ForceReport rep =
        this->UpdateForces(time, _flowVelWorld);

    // 可选: 保留力限幅 (原版没有,但可以保留作为安全措施)
    if (this->maxForce_ > 0.0)
    {
        double nF = rep.totalForce.Length();
        if (nF > this->maxForce_ && nF > 1e-9)
            rep.totalForce = rep.totalForce * (this->maxForce_ / nF);
    }
    if (this->maxTorque_ > 0.0)
    {
        double nT = rep.totalTorque.Length();
        if (nT > this->maxTorque_ && nT > 1e-9)
            rep.totalTorque = rep.totalTorque * (this->maxTorque_ / nT);
    }

    // Forces and torques are wrt link frame (已在 ComputeFullForces 中转换)
    this->link->AddRelativeForce(rep.totalForce);
    this->link->AddRelativeTorque(rep.totalTorque);

    this->ApplyBuoyancyForce();

    // 可选: 存储调试信息
    if (this->debugFlag)
    {
        this->StoreVector(UUV_DAMPING_FORCE, rep.dampingForce);
        this->StoreVector(UUV_DAMPING_TORQUE, rep.dampingTorque);

        this->StoreVector(UUV_ADDED_MASS_FORCE, rep.addedMassForce);
        this->StoreVector(UUV_ADDED_MASS_TORQUE, rep.addedMassTorque);

        this->StoreVector(UUV_ADDED_CORIOLIS_FORCE, rep.coriolisForce);
        this->StoreVector(UUV_ADDED_CORIOLIS_TORQUE, rep.coriolisTorque);
    }

    rep.stamp = time;
    this->lastForceReport = rep;
}



void HMFossen::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                          const Eigen::Matrix6d& _Ma,
                                          Eigen::Matrix6d &_Ca) const
{

  Eigen::Vector6d ab = this->GetAddedMass() * _vel;
  Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
  _Ca << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1 * CrossProductOperator(ab.tail<3>());
}


void HMFossen::ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                    Eigen::Matrix6d &_D) const
{
  // From Antonelli 2014: the viscosity of the fluid causes
  // the presence of dissipative drag and lift forces on the
  // body. A common simplification is to consider only linear
  // and quadratic damping terms and group these terms in a
  // matrix Drb

  _D.setZero();

  _D = (this->DLin + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) +
       std::abs(_vel[0]) * (this->DLinForwardSpeed +
       this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

  // Nonlinear damping matrix is considered as a diagonal matrix
  for (int i = 0; i < 6; i++)
  {
    _D(i, i) += (this->DNonLin(i, i) + this->offsetNonLinDamping) *
      std::fabs(_vel[i]);
  }
  _D *= this->scalingDamping;
}



Eigen::Matrix6d HMFossen::GetAddedMass() const
{
  return this->scalingAddedMass *
    (this->Ma + this->offsetAddedMass * Eigen::Matrix6d::Identity());
}

bool HMFossen::GetParam(std::string _tag, std::vector<double>& _output)
{
  _output = std::vector<double>();
  if (!_tag.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->Ma(i, j));
  }
  else if (!_tag.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLin(i, j));
  }
  else if (!_tag.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DLinForwardSpeed(i, j));
  }
  else if (!_tag.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(this->DNonLin(i, j));
  }
  else if (!_tag.compare("center_of_buoyancy"))
  {
    _output.push_back(this->centerOfBuoyancy.X());
    _output.push_back(this->centerOfBuoyancy.Y());
    _output.push_back(this->centerOfBuoyancy.Z());
  }
  else
    return false;
  gzmsg << "HydrodynamicModel::GetParam <" << _tag << ">=" << std::endl;
  for (auto elem : _output)
    std::cout << elem << " ";
  std::cout << std::endl;
  return true;
}

bool HMFossen::GetParam(std::string _tag, double& _output)
{
  _output = -1.0;
  if (!_tag.compare("volume"))
    _output = this->volume;
  else if (!_tag.compare("scaling_volume"))
    _output = this->scalingVolume;
  else if (!_tag.compare("scaling_added_mass"))
    _output = this->scalingAddedMass;
  else if (!_tag.compare("scaling_damping"))
    _output = this->scalingDamping;
  else if (!_tag.compare("fluid_density"))
    _output = this->fluidDensity;
  else if (!_tag.compare("bbox_height"))
    _output = this->boundingBox.ZLength();
  else if (!_tag.compare("bbox_width"))
    _output = this->boundingBox.YLength();
  else if (!_tag.compare("bbox_length"))
    _output = this->boundingBox.XLength();
  else if (!_tag.compare("offset_volume"))
    _output = this->offsetVolume;
  else if (!_tag.compare("offset_added_mass"))
    _output = this->offsetAddedMass;
  else if (!_tag.compare("offset_linear_damping"))
    _output = this->offsetLinearDamping;
  else if (!_tag.compare("offset_lin_forward_speed_damping"))
    _output = this->offsetLinForwardSpeedDamping;
  else if (!_tag.compare("offset_nonlin_damping"))
    _output = this->offsetNonLinDamping;
  else
  {
    _output = -1.0;
    return false;
  }

  gzmsg << "HydrodynamicModel::GetParam <" << _tag << ">=" << _output <<
    std::endl;
  return true;
}

bool HMFossen::SetParam(std::string _tag, double _input)
{
  if (!_tag.compare("scaling_volume"))
  {
    if (_input < 0)
      return false;
    this->scalingVolume = _input;
  }
  else if (!_tag.compare("scaling_added_mass"))
  {
    if (_input < 0)
      return false;
    this->scalingAddedMass = _input;
  }
  else if (!_tag.compare("scaling_damping"))
  {
    if (_input < 0)
      return false;
    this->scalingDamping = _input;
  }
  else if (!_tag.compare("fluid_density"))
  {
    if (_input < 0)
      return false;
    this->fluidDensity = _input;
  }
  else if (!_tag.compare("offset_volume"))
    this->offsetVolume = _input;
  else if (!_tag.compare("offset_added_mass"))
    this->offsetAddedMass = _input;
  else if (!_tag.compare("offset_linear_damping"))
    this->offsetLinearDamping = _input;
  else if (!_tag.compare("offset_lin_forward_speed_damping"))
    this->offsetLinForwardSpeedDamping = _input;
  else if (!_tag.compare("offset_nonlin_damping"))
    this->offsetNonLinDamping = _input;
  else
    return false;
  gzmsg << "HydrodynamicModel::SetParam <" << _tag << ">=" << _input <<
    std::endl;
  return true;
}

void HMFossen::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("all"))
  {
    for (auto tag : this->params)
      this->Print(tag);
    return;
  }
  if (!_message.empty())
    std::cout << _message << std::endl;
  else
    std::cout << this->link->GetModel()->GetName() << "::"
      << this->link->GetName() << "::" << _paramName
      << std::endl;
  if (!_paramName.compare("added_mass"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->Ma(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("linear_damping_forward_speed"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DLinForwardSpeed(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("quadratic_damping"))
  {
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
        std::cout << std::setw(12) << this->DNonLin(i, j);
      std::cout << std::endl;
    }
  }
  else if (!_paramName.compare("volume"))
  {
    std::cout << std::setw(12) << this->volume << " m^3" << std::endl;
  }
}



const std::string HMSphere::IDENTIFIER = "sphere";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSphere,
                                   &HMSphere::create)

HydrodynamicModel* HMSphere::create(sdf::ElementPtr _sdf,
                                    physics::LinkPtr _link)
{
  return new HMSphere(_sdf, _link);
}

HMSphere::HMSphere(sdf::ElementPtr _sdf,
                   physics::LinkPtr _link)
                   : HMFossen(_sdf, _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMSphere: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.XLength(),
                            std::min(this->boundingBox.YLength(),
                                     this->boundingBox.ZLength()));
  }
  gzmsg << "HMSphere::radius=" << this->radius << std::endl;
  gzmsg << "HMSphere: Computing added mass" << std::endl;

  this->params.push_back("radius");

  this->Re = 3e5;


  this->Cd = 0.5;

  this->areaSection = PI * std::pow(this->radius, 2.0);

  double sphereMa = -2.0 / 3.0 * this->fluidDensity * PI * \
                   std::pow(this->radius, 3.0);

  double Dq = -0.5 * this->fluidDensity * this->Cd * this->areaSection;

  for (int i = 0; i < 3; i++)
  {

    this->Ma(i, i) = -sphereMa; 
    this->DNonLin(i, i) = Dq;
  }
    this->volume = 4.0 / 3.0 * PI * std::pow(this->radius, 3.0);
}

void HMSphere::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("all"))
  {
    for (auto tag : this->params)
      this->Print(tag);
    return;
  }
  if (!_message.empty())
    std::cout << _message << std::endl;
  else
    std::cout << this->link->GetModel()->GetName() << "::"
      << this->link->GetName() << "::" << _paramName
      << std::endl;
  if (!_paramName.compare("radius"))
    std::cout << std::setw(12) << this->radius << std::endl;
  else
    HMFossen::Print(_paramName, _message);
}


const std::string HMCylinder::IDENTIFIER = "cylinder";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMCylinder,
                                   &HMCylinder::create)

HydrodynamicModel* HMCylinder::create(sdf::ElementPtr _sdf,
                                      physics::LinkPtr _link)
{
  return new HMCylinder(_sdf, _link);
}

HMCylinder::HMCylinder(sdf::ElementPtr _sdf,
                       physics::LinkPtr _link)
                       : HMFossen(_sdf, _link)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMCylinder: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.XLength(),
                            std::min(this->boundingBox.YLength(),
                                     this->boundingBox.ZLength()));
  }
  gzmsg << "HMCylinder::radius=" << this->radius << std::endl;

  if (modelParams->HasElement("length"))
    this->length = modelParams->Get<double>("length");
  else
  {
      gzmsg << "HMCylinder: Using the biggest length of bounding box as length"
            << std::endl;
      this->length = std::max(this->boundingBox.XLength(),
                              std::max(this->boundingBox.YLength(),
                                       this->boundingBox.ZLength()));
  }
  gzmsg << "HMCylinder::length=" << this->length << std::endl;

  this->dimRatio = this->length / (2* this->radius);

  gzmsg << "HMCylinder::dimension_ratio=" << this->dimRatio << std::endl;


  if (this->dimRatio <= 1) this->cdCirc = 0.91;
  else if (this->dimRatio > 1 && this->dimRatio <= 2) this->cdCirc = 0.85;
  else if (this->dimRatio > 2 && this->dimRatio <= 4) this->cdCirc = 0.87;
  else if (this->dimRatio > 4 && this->dimRatio <= 7) this->cdCirc = 0.99;

  if (this->dimRatio <= 1) this->cdLength = 0.63;
  else if (this->dimRatio > 1 && this->dimRatio <= 2) this->cdLength = 0.68;
  else if (this->dimRatio > 2 && this->dimRatio <= 5) this->cdLength = 0.74;
  else if (this->dimRatio > 5 && this->dimRatio <= 10) this->cdLength = 0.82;
  else if (this->dimRatio > 10 && this->dimRatio <= 40) this->cdLength = 0.98;
  else if (this->dimRatio > 40) this->cdLength = 0.98;

  if (modelParams->HasElement("axis"))
  {
    this->axis = modelParams->Get<std::string>("axis");
    GZ_ASSERT(this->axis.compare("i") == 0 ||
              this->axis.compare("j") == 0 ||
              this->axis.compare("k") == 0, "Invalid axis of rotation");
  }
  else
  {
    gzmsg << "HMCylinder: Using the direction of biggest length as axis"
          << std::endl;
    double maxLength = std::max(this->boundingBox.XLength(),
                                std::max(this->boundingBox.YLength(),
                                         this->boundingBox.ZLength()));
    if (maxLength == this->boundingBox.XLength())
      this->axis = "i";
    else if (maxLength == this->boundingBox.YLength())
      this->axis = "j";
    else
      this->axis = "k";
  }
  gzmsg << "HMCylinder::rotation_axis=" << this->axis << std::endl;

  double MaLength = -this->fluidDensity * PI *
                    std::pow(this->radius, 2.0) * this->length;

  double MaCirc = -this->fluidDensity * PI * std::pow(this->radius, 2.0);

  double MaLengthTorque = (-1.0/12.0) * this->fluidDensity * PI \
                * std::pow(this->radius, 2.0) * std::pow(this->length, 3.0);

  double DCirc = -0.5 * this->cdCirc * PI * std::pow(this->radius, 2.0) \
                    * this->fluidDensity;
  double DLength = -0.5 * this->cdLength * this->radius * this->length \
                    * this->fluidDensity;

  if (this->axis.compare("i") == 0)
  {
      this->Ma(0, 0) = -MaCirc;
      this->Ma(1, 1) = -MaLength;
      this->Ma(2, 2) = -MaLength;

      this->Ma(4, 4) = -MaLengthTorque;
      this->Ma(5, 5) = -MaLengthTorque;

      this->DNonLin(0, 0) = DCirc;
      this->DNonLin(1, 1) = DLength;
      this->DNonLin(2, 2) = DLength;
  }
  else if (this->axis.compare("j") == 0)
  {
      this->Ma(0, 0) = -MaLength;
      this->Ma(1, 1) = -MaCirc;
      this->Ma(2, 2) = -MaLength;

      this->Ma(3, 3) = -MaLengthTorque;
      this->Ma(5, 5) = -MaLengthTorque;

      this->DNonLin(0, 0) = DLength;
      this->DNonLin(1, 1) = DCirc;
      this->DNonLin(2, 2) = DLength;
  }
  else
  {
      this->Ma(0, 0) = -MaLength;
      this->Ma(1, 1) = -MaLength;
      this->Ma(2, 2) = -MaCirc;

      this->Ma(3, 3) = -MaLengthTorque;
      this->Ma(4, 4) = -MaLengthTorque;

      this->DNonLin(0, 0) = DLength;
      this->DNonLin(1, 1) = DLength;
      this->DNonLin(2, 2) = DCirc;
  }
}

void HMCylinder::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("radius"))
  {
    if (!_message.empty())
      gzmsg << this->link->GetName() << std::endl;
    std::cout << std::setw(12) << this->radius << std::endl;
  }
  else if (!_paramName.compare("length"))
  {
    if (!_message.empty())
      gzmsg << _message << std::endl;
    std::cout << std::setw(12) << this->length << std::endl;
  }
  else
    HMFossen::Print(_paramName, _message);
}

const std::string HMSpheroid::IDENTIFIER = "spheroid";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSpheroid,
                                   &HMSpheroid::create)

HydrodynamicModel* HMSpheroid::create(sdf::ElementPtr _sdf,
                                      physics::LinkPtr _link)
{
  return new HMSpheroid(_sdf, _link);
}

HMSpheroid::HMSpheroid(sdf::ElementPtr _sdf,
                       physics::LinkPtr _link)
                       : HMFossen(_sdf, _link)
{
  gzerr << "Hydrodynamic model for a spheroid is still in development!"
    << std::endl;
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("radius"))
    this->radius = modelParams->Get<double>("radius");
  else
  {
    gzmsg << "HMSpheroid: Using the smallest length of bounding box as radius"
          << std::endl;
    this->radius = std::min(this->boundingBox.XLength(),
                            std::min(this->boundingBox.YLength(),
                                     this->boundingBox.ZLength()));
  }
  GZ_ASSERT(this->radius > 0, "Radius cannot be negative");
  gzmsg << "HMSpheroid::radius=" << this->radius << std::endl;

  if (modelParams->HasElement("length"))
    this->length = modelParams->Get<double>("length");
  else
  {
      gzmsg << "HMSpheroid: Using the biggest length of bounding box as length"
            << std::endl;
      this->length = std::max(this->boundingBox.XLength(),
                              std::max(this->boundingBox.YLength(),
                                       this->boundingBox.ZLength()));
  }
  GZ_ASSERT(this->length > 0, "Length cannot be negative");
  gzmsg << "HMSpheroid::length=" << this->length << std::endl;

  double ecc = std::sqrt(1 -
               std::pow(this->radius / this->length, 2.0));

  gzmsg << "ecc=" << ecc << std::endl;

  double ln = std::log((1 + ecc) / (1 - ecc));
  double alpha = 2 * (1 - std::pow(ecc, 2.0)) / std::pow(ecc, 3.0);

  alpha *= (0.5 * ln - ecc);

  double beta = 1 / std::pow(ecc, 2.0) - \
                (1 - std::pow(ecc, 2.0) / (2 * std::pow(ecc, 3.0))) * ln;

  gzmsg << "alpha=" << alpha << std::endl;
  gzmsg << "beta=" << beta << std::endl;

  double mass;
#if GAZEBO_MAJOR_VERSION >= 8
  mass = this->link->GetInertial()->Mass();
#else
  mass = this->link->GetInertial()->GetMass();
#endif

  this->Ma(0, 0) = mass * alpha / (2 - alpha);
  this->Ma(1, 1) = mass * beta / (2 - beta);
  this->Ma(2, 2) = this->Ma(1, 1);
  this->Ma(3, 3) = 0;

  double ba_minus = std::pow(this->radius, 2.0) - std::pow(this->length, 2.0);
  double ba_plus = std::pow(this->radius, 2.0) + std::pow(this->length, 2.0);
  this->Ma(4, 4) = -0.2 * mass * std::pow(ba_minus, 2.0) * (alpha - beta);
  this->Ma(4, 4) /= (2 * ba_minus - ba_plus * (alpha - beta));

  this->Ma(5, 5) = this->Ma(4, 4);
}
void HMFossen::ComputeFullForces(
    double time,
    const ignition::math::Vector3d &_flowVelWorld,
    HydrodynamicModel::ForceReport& outReport)
{
    // 初始化报告
    outReport.totalForce = ignition::math::Vector3d::Zero;
    outReport.totalTorque = ignition::math::Vector3d::Zero;
    outReport.dampingForce = ignition::math::Vector3d::Zero;
    outReport.dampingTorque = ignition::math::Vector3d::Zero;
    outReport.addedMassForce = ignition::math::Vector3d::Zero;
    outReport.addedMassTorque = ignition::math::Vector3d::Zero;
    outReport.coriolisForce = ignition::math::Vector3d::Zero;
    outReport.coriolisTorque = ignition::math::Vector3d::Zero;
    outReport.buoyancyForce = ignition::math::Vector3d::Zero;
    outReport.stamp = time;
    outReport.inBodyFrame = true;

    if (!this->link)
    {
        gzerr << "[HMFossen] Link pointer is null" << std::endl;
        return;
    }

    // Link's pose
    ignition::math::Pose3d pose;
    ignition::math::Vector3d linVel, angVel;

#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->link->WorldPose();
    linVel = this->link->RelativeLinearVel();
    angVel = this->link->RelativeAngularVel();
#else
    pose = this->link->GetWorldPose().Ign();
    gazebo::math::Vector3 linVelG, angVelG;
    linVelG = this->link->GetRelativeLinearVel();
    angVelG = this->link->GetRelativeAngularVel();
    linVel = ignition::math::Vector3d(linVelG.x, linVelG.y, linVelG.z);
    angVel = ignition::math::Vector3d(angVelG.x, angVelG.y, angVelG.z);
#endif

    // Transform the flow velocity to the BODY frame
    ignition::math::Vector3d flowVel = pose.Rot().RotateVectorReverse(_flowVelWorld);

    Eigen::Vector6d velRel;
    // Compute the relative velocity
    velRel = EigenStack(
        this->ToNED(linVel - flowVel),
        this->ToNED(angVel));

    // ✅ 声明局部变量 Ca 和 D
    Eigen::Matrix6d Ca, D;

    // Update added Coriolis matrix
    this->ComputeAddedCoriolisMatrix(velRel, this->Ma, Ca);

    // Update damping matrix
    this->ComputeDampingMatrix(velRel, D);

    // Filter acceleration (see issue explanation above)
    this->ComputeAcc(velRel, time, 0.3);

    // We can now compute the additional forces/torques due to dynamic
    // effects based on Eq. 8.136 on p.222 of Fossen: Handbook of Marine Craft ...

    // Damping forces and torques
    Eigen::Vector6d damping = -D * velRel;

    // Added-mass forces and torques
    Eigen::Vector6d added = -this->GetAddedMass() * this->filteredAcc;

    // Added Coriolis term
    Eigen::Vector6d cor = -Ca * velRel;

    // All additional (compared to standard rigid body) Fossen terms combined.
    Eigen::Vector6d tau = damping + added + cor;

    GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is nan");

    if (!std::isnan(tau.norm()))
    {
        // Convert the forces and moments back to Gazebo's reference frame
        ignition::math::Vector3d hydForce =
            this->FromNED(Vec3dToGazebo(tau.head<3>()));
        ignition::math::Vector3d hydTorque =
            this->FromNED(Vec3dToGazebo(tau.tail<3>()));

        // 填充报告
        outReport.totalForce = hydForce;
        outReport.totalTorque = hydTorque;

        // 分项力 (转换回 Gazebo 坐标系)
        outReport.dampingForce = this->FromNED(Vec3dToGazebo(damping.head<3>()));
        outReport.dampingTorque = this->FromNED(Vec3dToGazebo(damping.tail<3>()));

        outReport.addedMassForce = this->FromNED(Vec3dToGazebo(added.head<3>()));
        outReport.addedMassTorque = this->FromNED(Vec3dToGazebo(added.tail<3>()));

        outReport.coriolisForce = this->FromNED(Vec3dToGazebo(cor.head<3>()));
        outReport.coriolisTorque = this->FromNED(Vec3dToGazebo(cor.tail<3>()));
    }
    double volume = this->GetVolume();
    double density = this->fluidDensity;
    
    // 获取重力(世界坐标系)
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d gravity = this->link->GetWorld()->Gravity();
#else
    ignition::math::Vector3d gravity = 
        this->link->GetWorld()->GetPhysicsEngine()->GetGravity().Ign();
#endif

    // 浮力 = -ρVg (世界坐标系,向上)
    ignition::math::Vector3d buoyancyWorld = -density * volume * gravity;
    
    // 转换到 body 坐标系
    ignition::math::Vector3d buoyancyBody = 
        pose.Rot().RotateVectorReverse(buoyancyWorld);
    
    outReport.buoyancyForce = buoyancyBody;

    outReport.stamp = time;
    outReport.inBodyFrame = false; // 已转换回 Gazebo 坐标系
        // ========== ✅ 保存力数据 (在函数末尾添加) ==========
    this->lastDampingForce = outReport.dampingForce;
    this->lastAddedMassForce = outReport.addedMassForce;
    this->lastCoriolisForce = outReport.coriolisForce;
    this->lastBuoyancyForce = outReport.buoyancyForce;
    this->lastSubmersionRatio = 1.0;  // 水下默认完全浸没
}




void HMSpheroid::Print(std::string _paramName, std::string _message)
{
  if (!_paramName.compare("radius"))
  {
    if (!_message.empty())
      gzmsg << this->link->GetName() << std::endl;
    std::cout << std::setw(12) << this->radius << std::endl;
  }
  else if (!_paramName.compare("length"))
  {
    if (!_message.empty())
      gzmsg << _message << std::endl;
    std::cout << std::setw(12) << this->length << std::endl;
  }
  else
    HMFossen::Print(_paramName, _message);
}


const std::string HMBox::IDENTIFIER = "box";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMBox,
                                   &HMBox::create)


HydrodynamicModel* HMBox::create(sdf::ElementPtr _sdf,
                                 physics::LinkPtr _link)
{
  return new HMBox(_sdf, _link);
}


HMBox::HMBox(sdf::ElementPtr _sdf,
             physics::LinkPtr _link)
             : HMFossen(_sdf, _link)
{
  gzerr << "Hydrodynamic model for box is still in development!" << std::endl;

  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model is missing");

  sdf::ElementPtr modelParams = _sdf->GetElement("hydrodynamic_model");

  if (modelParams->HasElement("cd"))
    this->Cd = modelParams->Get<double>("cd");
  else
  {
    gzmsg << "HMBox: Using 1 as drag coefficient"
          << std::endl;
    this->Cd = 1;
  }

  GZ_ASSERT(modelParams->HasElement("length"), "Length of the box is missing");
  GZ_ASSERT(modelParams->HasElement("width"), "Width of the box is missing");
  GZ_ASSERT(modelParams->HasElement("height"), "Height of the box is missing");

  this->length = modelParams->Get<double>("length");
  this->width = modelParams->Get<double>("width");
  this->height = modelParams->Get<double>("height");


  this->quadDampCoef[0] = -0.5 * this->Cd * this->width * this->height \
                    * this->fluidDensity;
  this->quadDampCoef[1] = -0.5 * this->Cd * this->length * this->height \
                    * this->fluidDensity;
  this->quadDampCoef[2] = -0.5 * this->Cd * this->width * this->length \
                    * this->fluidDensity;
}

void HMBox::Print(std::string _paramName, std::string _message)
{
    if (!_paramName.compare("length"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->length << std::endl;
    }
    else if (!_paramName.compare("width"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->width << std::endl;
    }
    else if (!_paramName.compare("height"))
    {
      if (!_message.empty())
        gzmsg << _message << std::endl;
      std::cout << std::setw(12) << this->height << std::endl;
    }
    else
      HMFossen::Print(_paramName, _message);
}
void HydrodynamicModel::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode && this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
bool HydrodynamicModel::OnGetForcesService(
    nezha_plugins::HydrodynamicsForces::Request &req,
    nezha_plugins::HydrodynamicsForces::Response &res)
{
    // 1. 获取最新的受力报告
    const ForceReport& report = this->lastForceReport;

    // 2. 填充浮力 (Buoyancy)
    res.buoyancy_x = report.buoyancyForce.X();
    res.buoyancy_y = report.buoyancyForce.Y();
    res.buoyancy_z = report.buoyancyForce.Z();

    // 3. 填充阻尼力 (Damping)
    res.damping_x = report.dampingForce.X();
    res.damping_y = report.dampingForce.Y();
    res.damping_z = report.dampingForce.Z();

    // 4. 填充附加质量力 (Added Mass)
    res.added_mass_x = report.addedMassForce.X();
    res.added_mass_y = report.addedMassForce.Y();
    res.added_mass_z = report.addedMassForce.Z();

    // 5. 填充科氏力 (Coriolis)
    res.coriolis_x = report.coriolisForce.X();
    res.coriolis_y = report.coriolisForce.Y();
    res.coriolis_z = report.coriolisForce.Z();

    // 6. 填充波浪力 (Wave)
    // 注意：这是水下动力学插件(Underwater Plugin)，通常只计算 Fossen 模型(阻尼/附加质量)。
    // 波浪力通常由 Surface Plugin 计算。在这里我们设为 0，或者你可以设为 totalForce。
    res.wave_x = 0.0; 
    res.wave_y = 0.0;
    res.wave_z = 0.0;

    // 7. 填充浸没比例 (直接访问类成员变量)
    res.submersion_ratio = this->lastSubmersionRatio;

    // 8. 填充时间戳
    res.sim_time = report.stamp;

    return true;
}
}
