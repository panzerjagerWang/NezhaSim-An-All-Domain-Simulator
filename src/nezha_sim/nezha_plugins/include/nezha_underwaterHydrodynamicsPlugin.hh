

#ifndef __UUV_GAZEBO_HYDRO_MODEL_HH__
#define __UUV_GAZEBO_HYDRO_MODEL_HH__
// 1. ROS 核心
#include <ros/ros.h>
#include <ros/callback_queue.h>

// 2. ROS 消息/服务
#include <nezha_plugins/HydrodynamicsForces.h>

// 3. Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>

// 4. 标准库
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

// 5. Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 6. 项目内部
#include <uuv_gazebo_plugins/Def.hh>
#include "nuuv_buoyantObject.hh"


// In ignition-math6 Box has been renamed to AxisAlignedBox and a 
// new Box class with different properties has been created. 
#if GAZEBO_MAJOR_VERSION >= 11
  using AxisAlignedBox = ignition::math::AxisAlignedBox;
#else
  using AxisAlignedBox = ignition::math::Box;
#endif


namespace gazebo
{
    class HydrodynamicModel;
 class HydrodynamicModelRegistry
    {
    public:
        static HydrodynamicModelRegistry& GetInstance()
        {
            static HydrodynamicModelRegistry instance;
            return instance;
        }

        void Register(HydrodynamicModel* model);
        void Unregister(HydrodynamicModel* model);
        std::vector<HydrodynamicModel*> GetModels();
        size_t Size();

    private:
        HydrodynamicModelRegistry() = default;
        ~HydrodynamicModelRegistry() = default;
        HydrodynamicModelRegistry(const HydrodynamicModelRegistry&) = delete;
        HydrodynamicModelRegistry& operator=(const HydrodynamicModelRegistry&) = delete;

        std::vector<HydrodynamicModel*> models_;
        std::mutex mutex_;
    };
class HydrodynamicModel : public BuoyantObject
{
protected:
    bool enabled_ = true;  
        // ========== ROS Service 相关 ==========
    ros::NodeHandle* rosNode{nullptr};
    ros::ServiceServer forcesService;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    
    // ========== 力数据存储 ==========
    ignition::math::Vector3d lastBuoyancyForce{0,0,0};
    ignition::math::Vector3d lastDampingForce{0,0,0};
    ignition::math::Vector3d lastAddedMassForce{0,0,0};
    ignition::math::Vector3d lastCoriolisForce{0,0,0};
    double lastSubmersionRatio{1.0};  // 水下默认完全浸没
public:
    physics::LinkPtr GetLink() const { return this->link; }
    

    void SetEnabled(bool enable) 
    { 
        enabled_ = enable; 
    }

    bool IsEnabled() const { return enabled_; }
        std::string GetName() const 
    { 
        if (this->link) {
            return this->link->GetName() + "::hydrodynamic_model";
        }
        return "hydrodynamic_model_unknown";
    }

public: struct ForceReport;

protected: HydrodynamicModel(sdf::ElementPtr _sdf, physics::LinkPtr _link);
public: virtual ~HydrodynamicModel();
public: virtual std::string GetType() = 0;
public: const ForceReport& GetLastForceReport() const { return this->lastForceReport; }
  /// \brief Computation of the hydrodynamic forces
  public: virtual void ApplyHydrodynamicForces(
    double time, const ignition::math::Vector3d &_flowVelWorld) = 0;

  void ResetStateForBelow();

  public: struct ForceReport
  {
    ignition::math::Vector3d totalForce      = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d totalTorque     = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d dampingForce    = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d dampingTorque   = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d addedMassForce  = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d addedMassTorque = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d coriolisForce   = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d coriolisTorque  = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d buoyancyForce   = ignition::math::Vector3d::Zero; 
    double stamp = 0.0;        
    bool   inBodyFrame = true; 
  };


  public: virtual ForceReport UpdateForces(
      double time, const ignition::math::Vector3d &_flowVelWorld);


  public: inline void SetAutoApply(bool _v) { this->autoApply_ = _v; }
  public: inline bool GetAutoApply() const { return this->autoApply_; }
  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
    std::string _message = std::string()) = 0;
protected:
    // ========== ✅ ROS Service 回调函数声明 ==========
    bool OnGetForcesService(
        nezha_plugins::HydrodynamicsForces::Request &req,
        nezha_plugins::HydrodynamicsForces::Response &res);
    
    void QueueThread();
    virtual void ComputeFullForces(
        double time,
        const ignition::math::Vector3d &_flowVelWorld,  
        ForceReport& outReport);
  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    std::vector<double>& _output) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    double& _output) = 0;

  /// \brief Set a scalar parameters
  public: virtual bool SetParam(std::string _tag, double _input) = 0;

  /// \brief Filter acceleration (fix due to the update structure of Gazebo)
  protected: void ComputeAcc(Eigen::Vector6d _velRel,
                            double _time,
                            double _alpha = 0.3);

  /// \brief Returns true if all parameters are available from the SDF element


  /// \brief Convert vector to comply with the NED reference frame
protected: ignition::math::Vector3d ToNED(ignition::math::Vector3d _vec);
protected: ignition::math::Vector3d FromNED(ignition::math::Vector3d _vec); 


  protected: bool CheckParams(sdf::ElementPtr _sdf);
  /// \brief Convert vector to comply with the NED reference frame


  protected: ForceReport lastForceReport;
  // mutable std::mutex reportMutex_;

  protected: bool autoApply_ = true;

  /// \brief Filtered linear & angular acceleration vector in link frame.
  /// This is used to prevent the model to become unstable given that Gazebo
  /// only calls the update function at the beginning or at the end of a
  /// iteration of the physics engine
  protected: Eigen::Vector6d filteredAcc;

  /// \brief Last timestamp (in seconds) at which ApplyHydrodynamicForces was
  /// called
  protected: double lastTime;

  /// \brief Last body-fixed relative velocity (nu_R in Fossen's equations)
  protected: Eigen::Vector6d lastVelRel;

  /// \brief List of parameters needed from the SDF element
  protected: std::vector<std::string> params;

  /// \brief Reynolds number (not used by all models)
  protected: double Re;

  /// \brief Temperature (not used by all models)
  protected: double temperature;

protected:

};
/////////////////////////////////////////////////
/// \brief Fossen's robot-like equations for underwater vehicles
/////////////////////////////////////////////////
/// \brief Fossen's robot-like equations for underwater vehicles
class HMFossen : public HydrodynamicModel
{
    /// \brief Constructor
    public: HMFossen(sdf::ElementPtr _sdf, physics::LinkPtr _link);
ignition::math::Vector3d ComputeBuoyancyForce();
    /// \brief Destructor
    public: virtual ~HMFossen() = default;

    /// \brief Return (derived) type of hydrodynamic model
    public: virtual std::string GetType() override { return IDENTIFIER; }

    /// \brief Computation of the hydrodynamic forces
    public: virtual void ApplyHydrodynamicForces(
        double time, 
        const ignition::math::Vector3d &_flowVelWorld) override;

    /// \brief Prints parameters
    public: virtual void Print(std::string _paramName,
        std::string _message = std::string()) override;

    /// \brief Return parameter in vector form
    public: virtual bool GetParam(std::string _tag,
        std::vector<double>& _output) override;

    /// \brief Return parameter in scalar form
    public: virtual bool GetParam(std::string _tag,
        double& _output) override;

    /// \brief Set a scalar parameter
    public: virtual bool SetParam(std::string _tag, double _input) override;

    /// \brief Factory method
    public: static HydrodynamicModel* create(
        sdf::ElementPtr _sdf, physics::LinkPtr _link);

protected:

    
    // ========== Service 回调函数 ==========
    bool OnGetForcesService(
        nezha_plugins::HydrodynamicsForces::Request &req,
        nezha_plugins::HydrodynamicsForces::Response &res);
    
    void QueueThread();
    virtual void ComputeFullForces(
        double time,
        const ignition::math::Vector3d &_flowVelWorld,
        ForceReport& outReport) override;

    /// \brief Compute added Coriolis matrix
    protected: void ComputeAddedCoriolisMatrix(
        const Eigen::Vector6d& _vel,
        const Eigen::Matrix6d& _Ma,
        Eigen::Matrix6d& _Ca) const;

    /// \brief Compute damping matrix
    protected: void ComputeDampingMatrix(
        const Eigen::Vector6d& _vel,
        Eigen::Matrix6d& _D) const;

    /// \brief Return added-mass matrix
    protected: Eigen::Matrix6d GetAddedMass() const;

    /// \brief Identifier for this model
    public: static const std::string IDENTIFIER;

    /// \brief Added-mass matrix
    protected: Eigen::Matrix6d Ma;

    /// \brief Linear damping matrix
    protected: Eigen::Matrix6d DLin;

    /// \brief Nonlinear damping matrix
    protected: Eigen::Matrix6d DNonLin;

    /// \brief Linear damping proportional to forward speed
    protected: Eigen::Matrix6d DLinForwardSpeed;

    /// \brief Scaling factors
    protected: double scalingAddedMass;
    protected: double offsetAddedMass;
    protected: double scalingDamping;
    protected: double offsetLinearDamping;
    protected: double offsetLinForwardSpeedDamping;
    protected: double offsetNonLinDamping;

    /// \brief Stored coefficients
    protected: std::vector<double> linearDampCoef;
    protected: std::vector<double> quadDampCoef;

    /// \brief Force/torque limits
    protected: double maxForce_;
    protected: double maxTorque_;
};


/// \brief Pointer to model
typedef boost::shared_ptr<HydrodynamicModel> HydrodynamicModelPtr;

/// \brief Function pointer to create a certain a model
typedef HydrodynamicModel* (*HydrodynamicModelCreator)(sdf::ElementPtr, \
                                                       physics::LinkPtr);

/// \brief Factory singleton class that creates a HydrodynamicModel from sdf.
class HydrodynamicModelFactory
{
  /// \brief Create HydrodynamicModel object according to its sdf Description.
  public: HydrodynamicModel* CreateHydrodynamicModel(sdf::ElementPtr _sdf,
                                                     physics::LinkPtr _link);

  /// \brief Returns the singleton instance of this factory.
  public: static HydrodynamicModelFactory& GetInstance();

  /// \brief Register a class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               HydrodynamicModelCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: HydrodynamicModelFactory() {}

  /// \brief Map of each registered identifier to its corresponding creator.
  private: std::map<std::string, HydrodynamicModelCreator> creators_;
};

/// Use the following macro within a HydrodynamicModel declaration:
#define REGISTER_HYDRODYNAMICMODEL(type) static const bool registeredWithFactory

/// Use the following macro before a HydrodynamicModel's definition:
#define REGISTER_HYDRODYNAMICMODEL_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  HydrodynamicModelFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);


class HMSphere : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  protected: REGISTER_HYDRODYNAMICMODEL(HMSphere);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMSphere(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Sphere radius
  protected: double radius;

  /// \brief Drag coefficient
  protected: double Cd;

  /// \brief Area of the cross section
  protected: double areaSection;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containing the methods and attributes for a hydrodynamic model
/// for a cylinder in the fluid
class HMCylinder : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMCylinder);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMCylinder(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Length of the cylinder
  protected: double length;

  /// \brief Sphere radius
  protected: double radius;

  /// \brief Name of the unit rotation axis (just a tag for x, y or z)
  protected: std::string axis;

  /// \brief Ratio between length and diameter
  protected: double dimRatio;

  /// \brief Approximated drag coefficient for the circular area
  protected: double cdCirc;

  /// \brief Approximated drag coefficient for the rectangular section
  protected: double cdLength;
};


class HMSpheroid : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMSpheroid);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMSpheroid(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Length of the sphroid
  protected: double length;

  /// \brief Prolate spheroid's smaller radius
  protected: double radius;
};


class HMBox : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMBox);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  /// \brief Constructor
  protected: HMBox(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Drag coefficient
  protected: double Cd;

  /// \brief Length of the box
  protected: double length;

  /// \brief Width of the box
  protected: double width;

  /// \brief Height of the box
  protected: double height;
};
}

#endif  // __UUV_GAZEBO_HYDRO_MODEL_HH__
