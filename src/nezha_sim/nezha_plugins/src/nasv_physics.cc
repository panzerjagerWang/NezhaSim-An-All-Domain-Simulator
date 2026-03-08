
#include "nasv_physics.hh"
#include "asv_wave_sim_gazebo_plugins/Algorithm.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/PhysicalConstants.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>

#include <gazebo/gazebo.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>

namespace asv 
{

  void DebugPrint(const Triangle& triangle)
  {
    gzmsg << "Vertex[0]:   " << triangle[0] << std::endl;
    gzmsg << "Vertex[1]:   " << triangle[1] << std::endl;
    gzmsg << "Vertex[2]:   " << triangle[2] << std::endl;
    gzmsg << "Normal:      " << Geometry::Normal(triangle) << std::endl;
  }


  Point3 Physics::CenterOfForce(
    double _fA, double _fB,
    const Point3& _A,
    const Point3& _B
  )
  { 
    double div = _fA + _fB;
    if (div != 0)
    {
      double t = _fA / div;
      return _B + (_A - _B) * t;  
    }
    else
    {
      return _B;
    }
  }

  double Physics::DeepWaterDispersionToOmega(double _wavenumber)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return std::sqrt(g * _wavenumber);
  }

  double Physics::DeepWaterDispersionToWavenumber(double _omega)
  {
    const double g = std::fabs(PhysicalConstants::Gravity());
    return _omega * _omega / g; 
  }

  double Physics::ViscousDragCoefficient(double Rn)
  {

    double r = std::max(1.0E3, Rn);
    double d = std::log10(r) - 2.0;
    double d2 = d*d;
    double CF = 0.075 / d2;
    return CF;
  }

Point3 Physics::CenterOfPressureApexUp(
  double _z0,
  const Point3& _H,
  const Point3& _M,
  const Point3& _B
)
{

  const double MIN_DIV = 1.0e-8;
  const double MIN_TC = -0.2;
  const double MAX_TC = 1.2;
  
  Vector3 alt = _B - _H;
  double h = _H.z() - _M.z();
  double tc = 2.0/3.0;
  
  double div = 6.0 * _z0 + 4.0 * h;
  if (std::abs(div) > MIN_DIV)
  {
    double tc_calc = (4.0 * _z0 + 3.0 * h) / div;
    
    if (tc_calc >= MIN_TC && tc_calc <= MAX_TC)
    {
      tc = tc_calc;
    }
  }
  
  return _H + alt * tc;
}

Point3 Physics::CenterOfPressureApexDn(
  double _z0,
  const Point3& _L,
  const Point3& _M,
  const Point3& _B
)
{

  const double MIN_DIV = 1.0e-8;
  const double MIN_TC = -0.2;
  const double MAX_TC = 1.2;
  
  Vector3 alt = _L - _B;
  double h = _M.z() - _L.z();
  double tc = 1.0/3.0;
  
  double div = 6.0 * _z0 + 2.0 * h;
  if (std::abs(div) > MIN_DIV)
  {
    double tc_calc = (2.0 * _z0 + h) / div;
    
    if (tc_calc >= MIN_TC && tc_calc <= MAX_TC)
    {
      tc = tc_calc;
    }
  }
  
  return _B + alt * tc;
}




void Physics::BuoyancyForceAtCenterOfPressure(
  double _depthC,
  const Point3& _C,
  const Point3& _H,
  const Point3& _M,
  const Point3& _L,
  const Vector3& _normal,
  Point3& _center, 
  Vector3& _force
)
{
  double fluidDensity = PhysicalConstants::WaterDensity(); 
  

  double gravity = std::fabs(PhysicalConstants::Gravity());
  
  Point3 D = Geometry::HorizontalIntercept(_H, _M, _L);
  Point3 B = Geometry::MidPoint(_M, D);
   
  double fU=0, fL=0;
  Point3 CpU = B;   
  Point3 CpL = B;

  if (_H.z() >= _M.z())
  {
    double z0 = _depthC - (_H.z() - _C.z());
    CpU = CenterOfPressureApexUp(z0, _H, _M, B);
    
    Point3 CU = Geometry::TriangleCentroid(_H, _M, D);
    double hCU = _depthC + (_C.z() - CU.z());
    double area = Geometry::TriangleArea(_H, _M, D);
    fU = fluidDensity * gravity * area * hCU;
  }

  if (_M.z() > _L.z())
  { 
    double z0 = _depthC + (_C.z() - _M.z());
    CpL = CenterOfPressureApexDn(z0, _L, _M, B);
    

    Point3 CL = Geometry::TriangleCentroid(_L, _M, D);
    double hCL = _depthC + (_C.z() - CL.z());
    double area = Geometry::TriangleArea(_L, _M, D);
    fL = fluidDensity * gravity * area * hCL;
  }
      

  _force = _normal * (fU + fL);
  _center = CenterOfForce(fU, fL, CpU, CpL);
}


  void Physics::BuoyancyForceAtCentroid(
    const WavefieldSampler& _wavefieldSampler,
    const Triangle& _triangle,
    Point3& _center,
    Vector3& _force
  )
  {

    double density = PhysicalConstants::WaterDensity();   
    double gravity = PhysicalConstants::Gravity();       
    

    _center = Geometry::TriangleCentroid(_triangle);


    double h = _wavefieldSampler.ComputeDepth(_center);


    Vector3 normal = Geometry::Normal(_triangle);
    double area = Geometry::TriangleArea(_triangle);
    _force = normal * (density * gravity * area * h);
  }

  void Physics::BuoyancyForceAtCenterOfPressure(
    const WavefieldSampler& _wavefieldSampler,
    const Triangle& _triangle,
    Point3& _center,
    Vector3& _force
  )
  {

    std::array<Point3, 3> v {
      _triangle[0],
      _triangle[1],
      _triangle[2]
    };
    std::array<double, 3> vz { v[0].z(), v[1].z(), v[2].z() };
    auto index = algorithm::sort_indexes(vz); 
    
    Point3 H = v[index[0]];
    Point3 M = v[index[1]];
    Point3 L = v[index[2]];


    Point3 C = Geometry::TriangleCentroid(_triangle);

  
    double depthC = _wavefieldSampler.ComputeDepth(C);

    Vector3 normal = Geometry::Normal(_triangle);


    BuoyancyForceAtCenterOfPressure(depthC, C, H, M, L, normal, _center, _force);
  }

  std::array<double, 3> Physics::ComputeHeightMap(
    const WavefieldSampler& _wavefieldSampler,
    const Triangle& _triangle
  )
  {

    Direction3 direction(0, 0, -1);
    std::array<double, 3> heightMap;

 
    for (int i=0; i<3; ++i)
    {
      Point3 vertex = _triangle[i];
      heightMap[i] = -_wavefieldSampler.ComputeDepth(vertex);
    }
    return heightMap;
  }    


  class HydrodynamicsParametersPrivate
  {
    public: HydrodynamicsParametersPrivate() :
      dampingOn(true),
      cDampL1(1.0E-6),
      cDampL2(1.0E-6),
      cDampR1(1.0E-6),
      cDampR2(1.0E-6),
      viscousDragOn(true),
      pressureDragOn(true),
      cPDrag1(1.0E+4),
      cPDrag2(1.0E+4),
      fPDrag(1),
      cSDrag1(1.0E+2),
      cSDrag2(1.0E+2),
      fSDrag(0.4),
      vRDrag(1.0),
      buoyancyScale(1.0),
      forceScaleFactor(1.0),
      maxDisplacedVolume(0.0)
    {}

  
    public: bool dampingOn;
    public: double cDampL1;
    public: double cDampL2;
    public: double cDampR1;
    public: double cDampR2;
    public: bool viscousDragOn;
    public: bool pressureDragOn;
    public: double cPDrag1;
    public: double cPDrag2;
    public: double fPDrag;
    public: double cSDrag1;
    public: double cSDrag2;
    public: double fSDrag;
    public: double vRDrag;
    public: double buoyancyScale;
    public: double forceScaleFactor;
    public: double maxDisplacedVolume;
  };

  HydrodynamicsParameters::~HydrodynamicsParameters()
  {
  }

  HydrodynamicsParameters::HydrodynamicsParameters() :
    data(new HydrodynamicsParametersPrivate())
  {
  }

  bool HydrodynamicsParameters::DampingOn() const
  {
    return this->data->dampingOn;
  }

  bool HydrodynamicsParameters::ViscousDragOn() const
  {
    return this->data->viscousDragOn;
  }

  bool HydrodynamicsParameters::PressureDragOn() const
  {
    return this->data->pressureDragOn;
  }

  void HydrodynamicsParameters::SetDampingOn(bool _on)
  {
    this->data->dampingOn = _on;
  }

  void HydrodynamicsParameters::SetViscousDragOn(bool _on)
  {
    this->data->viscousDragOn = _on;
  }

  void HydrodynamicsParameters::SetPressureDragOn(bool _on)
  {
    this->data->pressureDragOn = _on;
  }
  double HydrodynamicsParameters::CDampL1() const
  {
    return this->data->cDampL1;
  }

  double HydrodynamicsParameters::CDampL2() const
  {
    return this->data->cDampL2;
  }

  double HydrodynamicsParameters::CDampR1() const
  {
    return this->data->cDampR1;
  }

  double HydrodynamicsParameters::CDampR2() const
  {
    return this->data->cDampR2;
  }

  double HydrodynamicsParameters::CPDrag1() const
  {
    return this->data->cPDrag1;
  }

  double HydrodynamicsParameters::CPDrag2() const
  {
    return this->data->cPDrag2;
  }

  double HydrodynamicsParameters::FPDrag() const
  {
    return this->data->fPDrag;
  }

  double HydrodynamicsParameters::CSDrag1() const
  {
    return this->data->cSDrag1;
  }

  double HydrodynamicsParameters::CSDrag2() const
  {
    return this->data->cSDrag2;
  }

  double HydrodynamicsParameters::FSDrag() const
  {
    return this->data->fSDrag;
  }
  
  double HydrodynamicsParameters::VRDrag() const
  {
    return this->data->vRDrag;
  }

void HydrodynamicsParameters::SetFromMsg(const gazebo::msgs::Param_V& _msg)
{
  this->data->dampingOn      = Utilities::MsgParamBool(_msg,  "damping_on",       this->data->dampingOn);
  this->data->viscousDragOn  = Utilities::MsgParamBool(_msg,  "viscous_drag_on",  this->data->viscousDragOn);
  this->data->pressureDragOn = Utilities::MsgParamBool(_msg,  "pressure_drag_on", this->data->pressureDragOn);

  this->data->cDampL1 = Utilities::MsgParamDouble(_msg, "cDampL1",  this->data->cDampL1);
  this->data->cDampL2 = Utilities::MsgParamDouble(_msg, "cDampL2",  this->data->cDampL2);
  this->data->cDampR1 = Utilities::MsgParamDouble(_msg, "cDampR1",  this->data->cDampR1);
  this->data->cDampR2 = Utilities::MsgParamDouble(_msg, "cDampR2",  this->data->cDampR2);
  this->data->cPDrag1 = Utilities::MsgParamDouble(_msg, "cPDrag1",  this->data->cPDrag1);
  this->data->cPDrag2 = Utilities::MsgParamDouble(_msg, "cPDrag2",  this->data->cPDrag2);
  this->data->fPDrag  = Utilities::MsgParamDouble(_msg, "fPDrag",   this->data->fPDrag);
  this->data->cSDrag1 = Utilities::MsgParamDouble(_msg, "cSDrag1",  this->data->cSDrag1);
  this->data->cSDrag2 = Utilities::MsgParamDouble(_msg, "cSDrag2",  this->data->cSDrag2);
  this->data->fSDrag  = Utilities::MsgParamDouble(_msg, "fSDrag",   this->data->fSDrag);
  this->data->vRDrag  = Utilities::MsgParamDouble(_msg, "vRDrag",   this->data->vRDrag);
}
void HydrodynamicsParameters::SetFromSDF(sdf::Element& _sdf)
{
  this->data->dampingOn      = Utilities::SdfParamBool(_sdf,  "damping_on",       this->data->dampingOn);
  this->data->viscousDragOn  = Utilities::SdfParamBool(_sdf,  "viscous_drag_on",  this->data->viscousDragOn);
  this->data->pressureDragOn = Utilities::SdfParamBool(_sdf,  "pressure_drag_on", this->data->pressureDragOn);

  this->data->cDampL1 = Utilities::SdfParamDouble(_sdf, "cDampL1",  this->data->cDampL1);
  this->data->cDampL2 = Utilities::SdfParamDouble(_sdf, "cDampL2",  this->data->cDampL2);
  this->data->cDampR1 = Utilities::SdfParamDouble(_sdf, "cDampR1",  this->data->cDampR1);
  this->data->cDampR2 = Utilities::SdfParamDouble(_sdf, "cDampR2",  this->data->cDampR2);
  this->data->cPDrag1 = Utilities::SdfParamDouble(_sdf, "cPDrag1",  this->data->cPDrag1);
  this->data->cPDrag2 = Utilities::SdfParamDouble(_sdf, "cPDrag2",  this->data->cPDrag2);
  this->data->fPDrag  = Utilities::SdfParamDouble(_sdf, "fPDrag",   this->data->fPDrag);
  this->data->cSDrag1 = Utilities::SdfParamDouble(_sdf, "cSDrag1",  this->data->cSDrag1);
  this->data->cSDrag2 = Utilities::SdfParamDouble(_sdf, "cSDrag2",  this->data->cSDrag2);
  this->data->fSDrag  = Utilities::SdfParamDouble(_sdf, "fSDrag",   this->data->fSDrag);
  this->data->vRDrag  = Utilities::SdfParamDouble(_sdf, "vRDrag",   this->data->vRDrag);
}


  void HydrodynamicsParameters::DebugPrint() const
  {
    gzmsg << "damping_on:       " << this->data->dampingOn << std::endl;
    gzmsg << "viscous_drag_on:  " << this->data->viscousDragOn << std::endl;
    gzmsg << "pressure_drag_on: " << this->data->pressureDragOn << std::endl;
    gzmsg << "cDampL1:          " << this->data->cDampL1 << std::endl;
    gzmsg << "cDampL2:          " << this->data->cDampL2 << std::endl;
    gzmsg << "cDampR1:          " << this->data->cDampR1 << std::endl;
    gzmsg << "cDampR2:          " << this->data->cDampR2 << std::endl;
    gzmsg << "cPDrag1:          " << this->data->cPDrag1 << std::endl;
    gzmsg << "cPDrag2:          " << this->data->cPDrag2 << std::endl;
    gzmsg << "fPDrag:           " << this->data->fPDrag << std::endl;
    gzmsg << "cSDrag1:          " << this->data->cSDrag1 << std::endl;
    gzmsg << "cSDrag2:          " << this->data->cSDrag2 << std::endl;
    gzmsg << "fSDrag:           " << this->data->fSDrag << std::endl;
    gzmsg << "vRDrag:           " << this->data->vRDrag << std::endl;
    gzmsg << "buoyancy_scale:   " << this->data->buoyancyScale << std::endl;
    gzmsg << "force_scale_factor:   " << this->data->forceScaleFactor << std::endl;
    gzmsg << "max_displaced_volume: " << this->data->maxDisplacedVolume << " m³" << std::endl;

  }

// ✅ 在这里添加新函数
double HydrodynamicsParameters::BuoyancyScale() const
{
  return this->data->buoyancyScale;
}

void HydrodynamicsParameters::SetBuoyancyScale(double _scale)
{
  this->data->buoyancyScale = _scale;
}

double HydrodynamicsParameters::ForceScaleFactor() const
{
  return this->data->forceScaleFactor;
}

void HydrodynamicsParameters::SetForceScaleFactor(double _factor)
{
  this->data->forceScaleFactor = _factor;
  
  gzmsg << "╔════════════════════════════════════════╗" << std::endl;
  gzmsg << "║  Force Scale Factor Updated           ║" << std::endl;
  gzmsg << "╠════════════════════════════════════════╣" << std::endl;
  gzmsg << "║  New Value: " << std::setw(26) << std::fixed 
        << std::setprecision(3) << _factor << " ║" << std::endl;
  gzmsg << "╚════════════════════════════════════════╝" << std::endl;
}
double HydrodynamicsParameters::MaxDisplacedVolume() const
{
  return this->data->maxDisplacedVolume;
}

void HydrodynamicsParameters::SetMaxDisplacedVolume(double _volume)
{
  this->data->maxDisplacedVolume = _volume;
  
  gzmsg << "╔════════════════════════════════════════╗" << std::endl;
  gzmsg << "║  Max Displaced Volume Updated         ║" << std::endl;
  gzmsg << "╠════════════════════════════════════════╣" << std::endl;
  gzmsg << "║  New Value: " << std::setw(24) << std::fixed 
        << std::setprecision(4) << _volume << " m³ ║" << std::endl;
  gzmsg << "║  Status: " << (_volume > 0 ? "ENABLED " : "DISABLED") 
        << "                        ║" << std::endl;
  gzmsg << "╚════════════════════════════════════════╝" << std::endl;
}
  class TriangleProperties
  {
    public: TriangleProperties() :
      index(0),
      normal(CGAL::NULL_VECTOR),
      area(std::numeric_limits<double>::signaling_NaN()),
      subArea(std::numeric_limits<double>::signaling_NaN()),
      vh(CGAL::ORIGIN),
      vm(CGAL::ORIGIN),
      vl(CGAL::ORIGIN),
      hh(std::numeric_limits<double>::signaling_NaN()),
      hm(std::numeric_limits<double>::signaling_NaN()),
      hl(std::numeric_limits<double>::signaling_NaN())
    {
    }

    public: size_t index;                        
    public: Vector3 normal;                     
    public: double area;                        
    public: double subArea;                     
    public: std::array<double, 3> heightMap;        
    public: Point3 vh;                          
    public: Point3 vm;                          
    public: Point3 vl;                         
    public: double hh;                         
    public: double hm;                       
    public: double hl;                         
  };

  void DebugPrint(const TriangleProperties& props)
  {
    gzmsg << "index:        " << props.index << std::endl;
    gzmsg << "normal:       " << props.normal << std::endl;
    gzmsg << "area:         " << props.area << std::endl;
    gzmsg << "subArea:      " << props.subArea << std::endl;
    gzmsg << "vh:           " << props.vh << std::endl;
    gzmsg << "vm:           " << props.vm << std::endl;
    gzmsg << "vl:           " << props.vl << std::endl;
    gzmsg << "hh:           " << props.hh << std::endl;
    gzmsg << "hm:           " << props.hm << std::endl;
    gzmsg << "hl:           " << props.hl << std::endl;
  }

  class SubmergedTriangleProperties
  {
    public: SubmergedTriangleProperties() :
      index(0),
      normal(CGAL::NULL_VECTOR),
      centroid(CGAL::ORIGIN),
      xr(CGAL::NULL_VECTOR),
      area(std::numeric_limits<double>::signaling_NaN()),
      vp(CGAL::NULL_VECTOR),
      up(CGAL::NULL_VECTOR),
      cosTheta(std::numeric_limits<double>::signaling_NaN()),
      vn(CGAL::NULL_VECTOR),
      vt(CGAL::NULL_VECTOR),
      ut(CGAL::NULL_VECTOR),
      uf(CGAL::NULL_VECTOR),
      vf(CGAL::NULL_VECTOR)
    {          
    }

    public: int index;       
    public: Vector3 normal;   
    public: Point3 centroid;  
    public: Vector3 xr;      
    public: double area;     
    public: Vector3 vp;       
    public: Vector3 up;       
    public: double cosTheta; 
    public: Vector3 vn;       
    public: Vector3 vt;      
    public: Vector3 ut;      
    public: Vector3 uf;      
    public: Vector3 vf;       
  };

  void DebugPrint(const SubmergedTriangleProperties& props)
  {
    gzmsg << "index:        " << props.index << std::endl;
    gzmsg << "normal:       " << props.normal << std::endl;
    gzmsg << "centroid:     " << props.centroid << std::endl;
    gzmsg << "xr:           " << props.xr << std::endl;
    gzmsg << "area:         " << props.area << std::endl;
    gzmsg << "vp:           " << props.vp << std::endl;
    gzmsg << "up:           " << props.up << std::endl;
    gzmsg << "cosTheta:     " << props.cosTheta << std::endl;
    gzmsg << "vn:           " << props.vn << std::endl;
    gzmsg << "vt:           " << props.vt << std::endl;
    gzmsg << "ut:           " << props.ut << std::endl;
    gzmsg << "uf:           " << props.uf << std::endl;
    gzmsg << "vf:           " << props.vf << std::endl;
  }


  class HydrodynamicsPrivate
  {

    public: std::shared_ptr<const HydrodynamicsParameters> params;


    public: std::shared_ptr<const Mesh> linkMesh;


    public: std::shared_ptr<const WavefieldSampler>  wavefieldSampler;


    public: ignition::math::Pose3d pose;


    public: Point3 position;


    public: Vector3 linVelocity;

    public: Vector3 angVelocity;

    public: double waterlineLength;

    public: Mesh::Property_map<Mesh::Vertex_index, double> depths;
    public: std::vector<Triangle> submergedTriangles;
    public: std::vector<TriangleProperties> triangleProperties;
    public: std::vector<SubmergedTriangleProperties> submergedTriangleProperties;
    public: std::vector<Line> waterline;

    public: double area;

    public: double submergedArea;

    public: std::vector<Vector3> fBuoyancy;
    public: std::vector<Point3>  cBuoyancy;
  
    public: Vector3 force;

    public: Vector3 torque;
      public: Vector3 waveDragForce_;
  public: Vector3 meshBuoyancyForce_;
  public: Vector3 waveDragTorque_;
  };

  Hydrodynamics::Hydrodynamics(
    std::shared_ptr<const HydrodynamicsParameters> _params,
    std::shared_ptr<const Mesh> _linkMesh,
    std::shared_ptr<const WavefieldSampler> _wavefieldSampler
  ) : data(new HydrodynamicsPrivate())
  {
    this->data->params = _params;
    this->data->linkMesh = _linkMesh;
    this->data->wavefieldSampler = _wavefieldSampler;
    this->data->position = CGAL::ORIGIN;
    this->data->linVelocity = CGAL::NULL_VECTOR;
    this->data->angVelocity = CGAL::NULL_VECTOR;
    this->data->waterlineLength = 0.0;
  }

  void Hydrodynamics::Update(
    std::shared_ptr<const WavefieldSampler> _wavefieldSampler,
    const ignition::math::Pose3d& _pose,
    const Vector3& _linVelocity,
    const Vector3& _angVelocity
  )
  {
    this->data->wavefieldSampler = _wavefieldSampler;
    this->data->pose = _pose;
    this->data->position = ToPoint3(_pose.Pos());
    this->data->linVelocity = _linVelocity;
    this->data->angVelocity = _angVelocity;

    this->data->force = CGAL::NULL_VECTOR;
    this->data->torque = CGAL::NULL_VECTOR;
  this->data->waveDragForce_ = CGAL::NULL_VECTOR;
  this->data->meshBuoyancyForce_ = CGAL::NULL_VECTOR;
  this->data->waveDragTorque_ = CGAL::NULL_VECTOR;
    this->UpdateSubmergedTriangles();
    this->ComputeAreas();
    this->ComputeWaterlineLength();
    this->ComputePointVelocities();
    this->ComputeBuoyancyForce();

    if (this->data->params->ViscousDragOn())
      this->ComputeViscousDragForce();
    
    if (this->data->params->PressureDragOn())
      this->ComputePressureDragForce();
    
    if (this->data->params->DampingOn())
      this->ComputeDampingForce();
  }

  const Vector3& Hydrodynamics::Force() const
  {
    return this->data->force;
  }

  const Vector3& Hydrodynamics::Torque() const
  {
    return this->data->torque;
  }

  const std::vector<Line>& Hydrodynamics::GetWaterline() const
  {
    return this->data->waterline;
  }

  const std::vector<Triangle>& Hydrodynamics::GetSubmergedTriangles() const
  {
    return this->data->submergedTriangles;
  }

  void Hydrodynamics::UpdateSubmergedTriangles()
  {
    this->data->submergedTriangles.clear();
    this->data->triangleProperties.clear();
    this->data->submergedTriangleProperties.clear();
    this->data->waterline.clear();
    
    auto& linkMesh = *this->data->linkMesh;
    auto& wavefieldSampler = *this->data->wavefieldSampler;

    auto& ncLinkMesh = const_cast<Mesh&>(*this->data->linkMesh);
    auto pair = ncLinkMesh.add_property_map<Mesh::Vertex_index, double>("v:depth", 0);
    this->data->depths = pair.first;
    for (auto&& v : linkMesh.vertices())
    {
      this->data->depths[v] = wavefieldSampler.ComputeDepth(linkMesh.point(v));
    }

    for (auto&& face : linkMesh.faces())
    {
      Triangle triangle = Geometry::MakeTriangle(linkMesh, face);

      TriangleProperties triProps;
      triProps.normal = Geometry::Normal(triangle);
      triProps.area = Geometry::TriangleArea(triangle);


      const auto& rng = CGAL::vertices_around_face(linkMesh.halfedge(face), linkMesh);
      for (
        auto&& it = std::make_pair(std::begin(rng), 0); 
        it.first != std::end(rng);
        ++it.first, ++it.second)
      {
        triProps.heightMap[it.second] = -this->data->depths[*it.first];
      }

      this->data->triangleProperties.push_back(triProps);

      this->PopulateSubmergedTriangle(triangle, triProps);

    }

  }

void Hydrodynamics::PopulateSubmergedTriangle(
  const Triangle& _triangle,
  TriangleProperties& _triProps)
{

const double MIN_AREA = 1.0e-6;  // 从 5.0e-6 改为 1.0e-6


  if (_triProps.area < MIN_AREA)
  {

    return;
  }
  

  double edge01 = std::sqrt(CGAL::squared_distance(_triangle[0], _triangle[1]));
  double edge12 = std::sqrt(CGAL::squared_distance(_triangle[1], _triangle[2]));
  double edge20 = std::sqrt(CGAL::squared_distance(_triangle[2], _triangle[0]));
  
  double maxEdge = std::max({edge01, edge12, edge20});
  double minEdge = std::min({edge01, edge12, edge20});
  
const double MAX_ASPECT_RATIO = 100.0;  // 从 50.0 改为 100.0


  if (minEdge > 1.0e-10)  
  {
    double aspectRatio = maxEdge / minEdge;
    if (aspectRatio > MAX_ASPECT_RATIO)
    {

      return;
    }
  }
  

  bool hasValidHeight = false;
  for (int i = 0; i < 3; ++i)
  {
    if (std::isfinite(_triProps.heightMap[i]))
    {
      hasValidHeight = true;
      break;
    }
  }
  
  if (!hasValidHeight)
  {

    return;
  }
  

  const size_t H=0, M=1, L=2;
  std::array<size_t, 3> idx = algorithm::sort_indexes(_triProps.heightMap);

  _triProps.hh = _triProps.heightMap[idx[H]];
  _triProps.hm = _triProps.heightMap[idx[M]];
  _triProps.hl = _triProps.heightMap[idx[L]];

  _triProps.vh = _triangle[idx[H]];
  _triProps.vm = _triangle[idx[M]];
  _triProps.vl = _triangle[idx[L]];
    
  if (_triProps.hh > 0)
  {
    if (_triProps.hm > 0)
    {
      if (_triProps.hl > 0)
        ; 
      else
        this->SplitPartiallySubmergedTriangle1(_triProps);        
    } 
    else
      this->SplitPartiallySubmergedTriangle2(_triProps);        
  }
  else
    this->AddFullySubmergedTriangle(_triProps);      
}


void Hydrodynamics::SplitPartiallySubmergedTriangle1(TriangleProperties& _triProps)
{
  Vector3& n = _triProps.normal;
  Point3& vh = _triProps.vh;
  Point3& vm = _triProps.vm;
  Point3& vl = _triProps.vl;
  double hh = _triProps.hh;
  double hm = _triProps.hm;
  double hl = _triProps.hl;
  
  
  double denom_m = hm - hl;
  double denom_h = hh - hl;
  

const double MIN_HEIGHT_DIFF = 1.0e-4;  

  if (std::abs(denom_m) < MIN_HEIGHT_DIFF || std::abs(denom_h) < MIN_HEIGHT_DIFF)
  {

    return;
  }
  
  double tm = -hl / denom_m;
  double th = -hl / denom_h;

const double MIN_T = -0.05;  
const double MAX_T = 1.05;   
  
  if (tm < MIN_T || tm > MAX_T || th < MIN_T || th > MAX_T)
  {

    return;
  }
  

  tm = std::clamp(tm, 0.0, 1.0);
  th = std::clamp(th, 0.0, 1.0);
  
  Point3 vmi = vl + (vm - vl) * tm;
  Point3 vhi = vl + (vh - vl) * th; 


const double MIN_VERTEX_DISTANCE = 1.0e-5; 
  if (CGAL::squared_distance(vmi, vl) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE ||
      CGAL::squared_distance(vhi, vl) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE ||
      CGAL::squared_distance(vmi, vhi) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE)
  {

    return;
  }
          

  Triangle tri0(vl, vmi, vhi);
  if (CGAL::scalar_product(n, Geometry::Normal(tri0)) < 0.0)
  {

    tri0 = Triangle(vl, vhi, vmi);
  }
  
  
  double newArea = Geometry::TriangleArea(tri0);
const double MIN_SUBMERGED_AREA = 5.0e-6; 
  
  if (newArea < MIN_SUBMERGED_AREA)
  {

    return;
  }
  
  if (newArea > _triProps.area * 1.01)  
  {

    return;
  }
  
  this->data->submergedTriangles.push_back(tri0);


  SubmergedTriangleProperties subTriProps0;
  subTriProps0.index = _triProps.index;
  subTriProps0.normal = Geometry::Normal(tri0);
  subTriProps0.centroid = Geometry::TriangleCentroid(tri0);    
  subTriProps0.area = newArea;
  this->data->submergedTriangleProperties.push_back(subTriProps0);


  _triProps.subArea = subTriProps0.area;


  Line line(vmi, vhi);
  this->data->waterline.push_back(line);
}


void Hydrodynamics::SplitPartiallySubmergedTriangle2(TriangleProperties& _triProps)
{
  Vector3& n = _triProps.normal;
  Point3& vh = _triProps.vh;
  Point3& vm = _triProps.vm;
  Point3& vl = _triProps.vl;
  double hh = _triProps.hh;
  double hm = _triProps.hm;
  double hl = _triProps.hl;

  
  double denom_m = hh - hm;
  double denom_l = hh - hl;
  
  const double MIN_HEIGHT_DIFF = 1.0e-6;
  if (std::abs(denom_m) < MIN_HEIGHT_DIFF || std::abs(denom_l) < MIN_HEIGHT_DIFF)
  {
    return;
  }
  
  double tm = -hm / denom_m;
  double tl = -hl / denom_l;
  
  const double MIN_T = -0.1;
  const double MAX_T = 1.1;
  
  if (tm < MIN_T || tm > MAX_T || tl < MIN_T || tl > MAX_T)
  {
    return;
  }
  
  tm = std::clamp(tm, 0.0, 1.0);
  tl = std::clamp(tl, 0.0, 1.0);
  
  Point3 vmi = vm + (vh - vm) * tm;
  Point3 vli = vl + (vh - vl) * tl;

  
  const double MIN_VERTEX_DISTANCE = 1.0e-6;
  if (CGAL::squared_distance(vmi, vm) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE ||
      CGAL::squared_distance(vli, vl) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE ||
      CGAL::squared_distance(vmi, vli) < MIN_VERTEX_DISTANCE * MIN_VERTEX_DISTANCE)
  {
    return;
  }

  Triangle tri0(vm, vl, vmi);
  if (CGAL::scalar_product(n, Geometry::Normal(tri0)) < 0.0)
  {
    tri0 = Triangle(vm, vmi, vl);
  }

  Triangle tri1(vl, vli, vmi);
  if (CGAL::scalar_product(n, Geometry::Normal(tri1)) < 0.0)
  {
    tri1 = Triangle(vl, vmi, vli);
  }
  
  
  double area0 = Geometry::TriangleArea(tri0);
  double area1 = Geometry::TriangleArea(tri1);
  double totalArea = area0 + area1;
  
  const double MIN_SUBMERGED_AREA = 1.0e-8;
  
  if (area0 < MIN_SUBMERGED_AREA || area1 < MIN_SUBMERGED_AREA)
  {
    return;
  }
  
  if (totalArea > _triProps.area * 1.01)
  {
    return;
  }

  this->data->submergedTriangles.push_back(tri0);
  this->data->submergedTriangles.push_back(tri1);


  SubmergedTriangleProperties subTriProps0;
  subTriProps0.index = _triProps.index;
  subTriProps0.normal = Geometry::Normal(tri0);
  subTriProps0.centroid = Geometry::TriangleCentroid(tri0);
  subTriProps0.area = area0;
  this->data->submergedTriangleProperties.push_back(subTriProps0);

  SubmergedTriangleProperties subTriProps1;
  subTriProps1.index = _triProps.index;
  subTriProps1.normal = Geometry::Normal(tri1);
  subTriProps1.centroid = Geometry::TriangleCentroid(tri1);
  subTriProps1.area = area1;
  this->data->submergedTriangleProperties.push_back(subTriProps1);


  _triProps.subArea = totalArea;


  Line line(vmi, vli);
  this->data->waterline.push_back(line);
}


  void Hydrodynamics::AddFullySubmergedTriangle(TriangleProperties& _triProps)
  {
    Vector3& n = _triProps.normal;
    Point3& vh = _triProps.vh;
    Point3& vm = _triProps.vm;
    Point3& vl = _triProps.vl;
    

    Triangle tri(vh, vm, vl);
    if (CGAL::scalar_product(n, Geometry::Normal(tri)) < 0.0)
    {
      tri = Triangle(vm, vh, vl);
    }
    this->data->submergedTriangles.push_back(tri);

    SubmergedTriangleProperties subTriProps;
    subTriProps.index = _triProps.index;
    subTriProps.normal = _triProps.normal;
    subTriProps.centroid = Geometry::TriangleCentroid(tri);    
    subTriProps.area = _triProps.area;
    this->data->submergedTriangleProperties.push_back(subTriProps);


    _triProps.subArea = subTriProps.area;
  }

  void Hydrodynamics::ComputeAreas()
  {
    double area=0.0;
    for (auto&& props : this->data->triangleProperties)
    {
      area += props.area;
    }
    this->data->area = area;

    double subArea=0.0;
    for (auto&& props : this->data->submergedTriangleProperties)
    {
      subArea += props.area;
    }
    this->data->submergedArea = subArea;
  }

  void Hydrodynamics::ComputeWaterlineLength()
  {

    Vector3 xaxis = ToVector3(this->data->pose.Rot().RotateVector(
      ignition::math::Vector3d(1, 0, 0)));


    double length = 0.0;
    for (auto&& line : this->data->waterline)
    {
      length += std::abs(CGAL::scalar_product(line.to_vector(), xaxis));
    }
    length *= 0.5;
    this->data->waterlineLength = length;

  }


  void Hydrodynamics::ComputePointVelocities()
  {
    auto& position = this->data->position;
    auto& v = this->data->linVelocity;
    auto& omega = this->data->angVelocity;

    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {

      subTriProps.xr = subTriProps.centroid - position;
      

      subTriProps.vp = v + CGAL::cross_product(omega, subTriProps.xr);


      subTriProps.up = Geometry::Normalize(subTriProps.vp);

      subTriProps.cosTheta = CGAL::scalar_product(subTriProps.up, subTriProps.normal);


      subTriProps.vn = subTriProps.normal * subTriProps.cosTheta;
      

      subTriProps.vt = subTriProps.vp - subTriProps.vn;


      subTriProps.ut = Geometry::Normalize(subTriProps.vt);

      subTriProps.uf = - subTriProps.ut;

      subTriProps.vf = subTriProps.uf * std::sqrt(subTriProps.vp.squared_length());
    }
  }


  double Hydrodynamics::ComputeReynoldsNumber() const 
  {

    auto& v = this->data->linVelocity;
    double u = std::sqrt(v.squared_length());


    double L = this->data->waterlineLength;


    double kv = PhysicalConstants::WaterKinematicViscosity();
    double Rn = u * L / kv;

    return Rn;
  }

void Hydrodynamics::ComputeBuoyancyForce()
{
  Vector3 sumForce  = CGAL::NULL_VECTOR;
  Vector3 sumTorque = CGAL::NULL_VECTOR;

  this->data->fBuoyancy.clear();
  this->data->cBuoyancy.clear();

  // ✅ 第一步：计算原始浮力和估算排水体积
  double buoyancyScale = this->data->params->BuoyancyScale();
  double estimatedVolume = 0.0;  // 累计估算排水体积
  
  std::vector<Vector3> rawForces;      // 存储原始浮力
  std::vector<Point3>  rawCenters;     // 存储作用点
  
  auto& position = this->data->position;
  auto& wavefieldSampler = *this->data->wavefieldSampler;
  
  for (auto&& subTri : this->data->submergedTriangles)
  {
    Point3 center = CGAL::ORIGIN;
    Vector3 force = CGAL::NULL_VECTOR;
    Physics::BuoyancyForceAtCenterOfPressure(
      wavefieldSampler, subTri, center, force);
    
    // 应用浮力缩放系数
    force = force * buoyancyScale;
    
    rawForces.push_back(force);
    rawCenters.push_back(center);
    
    // ✅ 估算该三角形贡献的排水体积
    // 方法：面积 × 三个顶点平均水下深度
    double area = Geometry::TriangleArea(subTri);
    double avgDepth = 0.0;
    for (int i = 0; i < 3; ++i)
    {
      double depth = wavefieldSampler.ComputeDepth(subTri[i]);
      avgDepth += std::max(0.0, depth);  // 只计算水下部分
    }
    avgDepth /= 3.0;
    
    double volumeContribution = area * avgDepth;
    estimatedVolume += volumeContribution;
  }

  // ✅ 第二步：根据体积限制计算缩放系数
  double volumeScale = 1.0;
  double maxVolume = this->data->params->MaxDisplacedVolume();
  
  if (maxVolume > 0.0 && estimatedVolume > 0.0)
  {
    if (estimatedVolume > maxVolume)
    {
      volumeScale = maxVolume / estimatedVolume;
      
      // 输出警告信息（每100帧输出一次）
      static int warnCounter = 0;
      if ((warnCounter++ % 100) == 0)
      {
        gzwarn << "╔════════════════════════════════════════╗" << std::endl;
        gzwarn << "║  Volume Limit Exceeded!               ║" << std::endl;
        gzwarn << "╠════════════════════════════════════════╣" << std::endl;
        gzwarn << "║  Estimated: " << std::setw(10) << std::fixed 
               << std::setprecision(4) << estimatedVolume << " m³        ║" << std::endl;
        gzwarn << "║  Limit:     " << std::setw(10) << maxVolume << " m³        ║" << std::endl;
        gzwarn << "║  Scale:     " << std::setw(10) << std::setprecision(3) 
               << volumeScale << "              ║" << std::endl;
        gzwarn << "╚════════════════════════════════════════╝" << std::endl;
      }
    }
  }

  // ✅ 第三步：应用体积缩放并累加力和力矩
  for (size_t i = 0; i < rawForces.size(); ++i)
  {
    Vector3 force = rawForces[i] * volumeScale;
    Point3 center = rawCenters[i];
    
    this->data->fBuoyancy.push_back(force);
    this->data->cBuoyancy.push_back(center);

    Vector3 xr = center - position;
    Vector3 torque = CGAL::cross_product(xr, force);
    sumForce += force;
    sumTorque += torque;
  }

  this->data->force  += sumForce;
  this->data->torque += sumTorque;
  
  // ✅ 调试输出（每50帧一次）
  static int debugCounter = 0;
  if ((debugCounter++ % 50) == 0)
  {
    gzmsg << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
    gzmsg << "🌊 Buoyancy Computation Summary [Frame " << debugCounter << "]" << std::endl;
    gzmsg << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
    gzmsg << "Estimated Volume:  " << std::fixed << std::setprecision(4) 
          << estimatedVolume << " m³" << std::endl;
    gzmsg << "Max Volume:        " << maxVolume << " m³" << std::endl;
    gzmsg << "Volume Scale:      " << std::setprecision(3) << volumeScale << std::endl;
    gzmsg << "Buoyancy Scale:    " << buoyancyScale << std::endl;
    gzmsg << "Total Force:       [" << std::setprecision(2)
          << sumForce.x() << ", " << sumForce.y() << ", " << sumForce.z() << "] N" << std::endl;
    gzmsg << "Total Torque:      [" 
          << sumTorque.x() << ", " << sumTorque.y() << ", " << sumTorque.z() << "] N·m" << std::endl;
    gzmsg << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
  }
}


  void Hydrodynamics::ComputeDampingForce()
  {
    auto& params = *this->data->params;

    double cDampL1 = params.CDampL1();
    double cDampR1 = params.CDampR1();

    double cDampL2 = params.CDampL2();
    double cDampR2 = params.CDampR2();

    double area = this->data->area;
    double subArea = this->data->submergedArea;
    double rs = subArea / area;
     
    auto& v = this->data->linVelocity;
    double linSpeed = std::sqrt(v.squared_length());
    double cL = - rs * (cDampL1 + cDampL2 * linSpeed);
    Vector3 force = v * cL;
 
    auto& omega = this->data->angVelocity;
    double angSpeed = std::sqrt(omega.squared_length());
    double cR = - rs * (cDampR1 + cDampR2 * angSpeed);    
    Vector3 torque = omega * cR;

    this->data->force  += force;
    this->data->torque += torque;


  }

  void Hydrodynamics::ComputeViscousDragForce()
  {    
    double rho = PhysicalConstants::WaterDensity();
    double Rn = this->ComputeReynoldsNumber();
    double cF = Physics::ViscousDragCoefficient(Rn);
 
    Vector3 sumForce  = CGAL::NULL_VECTOR;
    Vector3 sumTorque = CGAL::NULL_VECTOR;
    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {

      double fDrag = 0.5 * rho * cF * subTriProps.area 
        * std::sqrt(subTriProps.vf.squared_length());
      Vector3 force = subTriProps.vf * fDrag;  
      sumForce += force;


      Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
      sumTorque += torque;
    }

    this->data->force  += sumForce;
    this->data->torque += sumTorque;
  }


  void Hydrodynamics::ComputePressureDragForce()
  {
    auto& params = *this->data->params;


    double cPDrag1 = params.CPDrag1();
    double cPDrag2 = params.CPDrag2();
    double fPDrag  = params.FPDrag();

    double cSDrag1 = params.CSDrag1();
    double cSDrag2 = params.CSDrag2();
    double fSDrag  = params.FSDrag();


    double vRDrag  = params.VRDrag();

    Vector3 sumForce  = CGAL::NULL_VECTOR;
    Vector3 sumTorque = CGAL::NULL_VECTOR;
    for (auto&& subTriProps : this->data->submergedTriangleProperties)
    {

      double S    = subTriProps.area;
      double vp   = std::sqrt(subTriProps.vp.squared_length());
      double cosTheta = subTriProps.cosTheta;

      double v    = vp / vRDrag;
      double drag = 0.0;
      if (cosTheta >= 0.0)
      {
        drag = -(cPDrag1 * v + cPDrag2 * v * v) * S * std::pow( cosTheta, fPDrag);
      }
      else
      {
        drag =  (cSDrag1 * v + cSDrag2 * v * v) * S * std::pow(-cosTheta, fSDrag);
      }
      Vector3 force = subTriProps.normal * drag;
      sumForce += force;


      Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
      sumTorque += torque;
    }

    this->data->force  += sumForce;
    this->data->torque += sumTorque;


  }
Vector3 Hydrodynamics::GetBuoyancyForce() const
{
  Vector3 sumForce = CGAL::NULL_VECTOR;
  for (auto&& force : this->data->fBuoyancy)
  {
    sumForce += force;
  }
  return sumForce;
}

Vector3 Hydrodynamics::GetBuoyancyTorque() const
{
  Vector3 sumTorque = CGAL::NULL_VECTOR;
  auto& position = this->data->position;
  
  for (size_t i = 0; i < this->data->fBuoyancy.size(); ++i)
  {
    Vector3 xr = this->data->cBuoyancy[i] - position;
    Vector3 torque = CGAL::cross_product(xr, this->data->fBuoyancy[i]);
    sumTorque += torque;
  }
  return sumTorque;
}
Vector3 Hydrodynamics::GetWaveDragForce() const
{
  // 计算除浮力外的所有力
  Vector3 totalDrag = this->data->force - this->GetBuoyancyForce();
  return totalDrag;
}

// ✅ 新增：获取网格浮力
Vector3 Hydrodynamics::GetMeshBuoyancyForce() const
{
  return this->GetBuoyancyForce();  // 复用现有方法
}

// ✅ 新增：获取波浪拖拽力矩
Vector3 Hydrodynamics::GetWaveDragTorque() const
{
  // 计算除浮力力矩外的所有力矩
  Vector3 totalDragTorque = this->data->torque - this->GetBuoyancyTorque();
  return totalDragTorque;
}


} 

