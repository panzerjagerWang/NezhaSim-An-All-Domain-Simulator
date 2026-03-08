

#include "nasv_waveField.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Geometry.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Physics.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <CGAL/Aff_transformation_3.h>
#include <CGAL/number_utils.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>
#include <sstream>   
#include <iomanip>   
#include <cmath>    

#include <Eigen/Dense>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <tbb/tbb.h>

#include <array>
#include <iostream>
#include <cmath>
#include <string>

namespace asv 
{
  namespace internal
  {
  
    inline bool IsValidPoint(const Point3& _point, double _maxCoord = 1e6)
    {
        return std::isfinite(_point.x()) && 
               std::isfinite(_point.y()) && 
               std::isfinite(_point.z()) &&
               std::abs(CGAL::to_double(_point.x())) < _maxCoord &&
               std::abs(CGAL::to_double(_point.y())) < _maxCoord &&
               std::abs(CGAL::to_double(_point.z())) < _maxCoord;
    }


    inline std::string FormatPoint(const Point3& _point)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "(" << CGAL::to_double(_point.x()) << ", " 
          << CGAL::to_double(_point.y()) << ", " 
          << CGAL::to_double(_point.z()) << ")";
      return oss.str();
    }
  } 
  typedef CGAL::Aff_transformation_2<Kernel> TransformMatrix;


  std::ostream& operator<<(std::ostream& os, const std::vector<double>& _vec)
  { 
    for (auto&& v : _vec )
      os << v << ", ";
    return os;
  }


  class WaveParametersPrivate
  {

    public: WaveParametersPrivate():
      number(1), 
      scale(2.0),
      angle(2.0*M_PI/10.0),
      steepness(1.0),
      amplitude(0.0), 
      period(1.0), 
      phase(0.0), 
      direction(1, 0),
      angularFrequency(2.0*M_PI),
      wavelength(2*M_PI/Physics::DeepWaterDispersionToWavenumber(2.0*M_PI)), 
      wavenumber(Physics::DeepWaterDispersionToWavenumber(2.0*M_PI))
    {
    }

    public: size_t number;


    public: double scale;

    public: double angle;

    public: double steepness;


    public: double amplitude;

    public: double period;

    public: double phase;

    public: Vector2 direction;
  
    public: double angularFrequency;

    public: double wavelength;

    public: double wavenumber;

    public: std::vector<double> angularFrequencies;

    public: std::vector<double> amplitudes;

    public: std::vector<double> phases;

    public: std::vector<double> steepnesses;

    public: std::vector<double> wavenumbers;

    public: std::vector<Vector2> directions;

    public: void Recalculate()
    {

      this->direction = Geometry::Normalize(this->direction);

      this->angularFrequency = 2.0 * M_PI / this->period;
      this->wavenumber = Physics::DeepWaterDispersionToWavenumber(this->angularFrequency);
      this->wavelength = 2.0 * M_PI / this->wavenumber;

      this->angularFrequencies.clear();
      this->amplitudes.clear();
      this->phases.clear();
      this->wavenumbers.clear();
      this->steepnesses.clear();
      this->directions.clear();

      for (size_t i=0; i<this->number; ++i)
      {
        const int n = i - this->number/2;
        const double scaleFactor = std::pow(this->scale, n);
        const double a = scaleFactor * this->amplitude;
        const double k = this->wavenumber / scaleFactor;
        const double omega = Physics::DeepWaterDispersionToOmega(k);
        const double phi = this->phase;
        double q = 0.0;
        if (a != 0)
        {
          q = std::min(1.0, this->steepness / (a * k * this->number));
        }

        this->amplitudes.push_back(a);        
        this->angularFrequencies.push_back(omega);
        this->phases.push_back(phi);
        this->steepnesses.push_back(q);
        this->wavenumbers.push_back(k);
      

        const double c = std::cos(n * this->angle);
        const double s = std::sin(n * this->angle);
        const TransformMatrix T(
          c, -s,
          s,  c
        );
        const Vector2 d = T(this->direction);
        directions.push_back(d);
      }
    }
  };


  WaveParameters::~WaveParameters()
  {
  }

  WaveParameters::WaveParameters()
    : data(new WaveParametersPrivate())
  {
    this->data->Recalculate();
  }

  void WaveParameters::FillMsg(gazebo::msgs::Param_V& _msg) const
  {

    _msg.mutable_param()->Clear();

    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("number");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::INT32);
      nextParam->mutable_value()->set_int_value(this->data->number);
    }
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("scale");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->scale);
    }
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("angle");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->angle);
    }
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("steepness");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->steepness);
    }
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("amplitude");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->amplitude);
    }
    {
      auto nextParam = _msg.add_param();
      nextParam->set_name("period");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(this->data->period);
    }
    {
      const auto& direction = this->data->direction;
      auto nextParam = _msg.add_param();
      nextParam->set_name("direction");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::VECTOR3D);
      nextParam->mutable_value()->mutable_vector3d_value()->set_x(direction.x());
      nextParam->mutable_value()->mutable_vector3d_value()->set_y(direction.y());
      nextParam->mutable_value()->mutable_vector3d_value()->set_z(0);
    }
  }

  void WaveParameters::SetFromMsg(const gazebo::msgs::Param_V& _msg)
  {
    this->data->number    = Utilities::MsgParamSizeT(_msg,    "number",     this->data->number);
    this->data->amplitude = Utilities::MsgParamDouble(_msg,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::MsgParamDouble(_msg,   "period",     this->data->period);
    this->data->phase     = Utilities::MsgParamDouble(_msg,   "phase",      this->data->phase);
    this->data->direction = Utilities::MsgParamVector2(_msg,  "direction",  this->data->direction);
    this->data->scale     = Utilities::MsgParamDouble(_msg,   "scale",      this->data->scale);
    this->data->angle     = Utilities::MsgParamDouble(_msg,   "angle",      this->data->angle);
    this->data->steepness = Utilities::MsgParamDouble(_msg,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  void WaveParameters::SetFromSDF(sdf::Element& _sdf)
  {
    this->data->number    = Utilities::SdfParamSizeT(_sdf,    "number",     this->data->number);
    this->data->amplitude = Utilities::SdfParamDouble(_sdf,   "amplitude",  this->data->amplitude);
    this->data->period    = Utilities::SdfParamDouble(_sdf,   "period",     this->data->period);
    this->data->phase     = Utilities::SdfParamDouble(_sdf,   "phase",      this->data->phase);
    this->data->direction = Utilities::SdfParamVector2(_sdf,  "direction",  this->data->direction);
    this->data->scale     = Utilities::SdfParamDouble(_sdf,   "scale",      this->data->scale);
    this->data->angle     = Utilities::SdfParamDouble(_sdf,   "angle",      this->data->angle);
    this->data->steepness = Utilities::SdfParamDouble(_sdf,   "steepness",  this->data->steepness);

    this->data->Recalculate();
  }

  size_t WaveParameters::Number() const
  {
    return this->data->number;
  }

  double WaveParameters::Angle() const
  {
    return this->data->angle;
  }

  double WaveParameters::Scale() const
  {
    return this->data->scale;
  }

  double WaveParameters::Steepness() const
  {
    return this->data->steepness;
  }

  double WaveParameters::AngularFrequency() const
  {
    return this->data->angularFrequency;
  }

  double WaveParameters::Amplitude() const
  {
    return this->data->amplitude;
  }
  
  double WaveParameters::Period() const
  {
    return this->data->period;
  }
  
  double WaveParameters::Phase() const
  {
    return this->data->phase;
  }

  double WaveParameters::Wavelength() const
  {
    return this->data->wavelength;
  }

  double WaveParameters::Wavenumber() const
  {
    return this->data->wavenumber;
  }    

  Vector2 WaveParameters::Direction() const
  {
    return this->data->direction;
  }
  
  void WaveParameters::SetNumber(size_t _number)
  {
    this->data->number = _number;
    this->data->Recalculate();
  }

  void WaveParameters::SetAngle(double _angle)
  {
    this->data->angle = _angle;
    this->data->Recalculate();
  }

  void WaveParameters::SetScale(double _scale)
  {
    this->data->scale = _scale;
    this->data->Recalculate();
  }

  void WaveParameters::SetSteepness(double _steepness)
  {
    this->data->steepness = _steepness;
    this->data->Recalculate();
  }

  void WaveParameters::SetAmplitude(double _amplitude)
  {
    this->data->amplitude = _amplitude;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetPeriod(double _period)
  {
    this->data->period = _period;
    this->data->Recalculate();
  }
    
  void WaveParameters::SetPhase(double _phase)
  {
    this->data->phase = _phase;
    this->data->Recalculate();
  }
  
  void WaveParameters::SetDirection(const Vector2& _direction)
  {
    this->data->direction = _direction;
    this->data->Recalculate();
  }

  const std::vector<double>& WaveParameters::AngularFrequency_V() const
  {
    return this->data->angularFrequencies;
  }

  const std::vector<double>& WaveParameters::Amplitude_V() const
  {
    return this->data->amplitudes;
  }
  
  const std::vector<double>& WaveParameters::Phase_V() const
  {
    return this->data->phases;
  }
  
  const std::vector<double>& WaveParameters::Steepness_V() const
  {
    return this->data->steepnesses;
  }

  const std::vector<double>& WaveParameters::Wavenumber_V() const
  {
    return this->data->wavenumbers;
  }

  const std::vector<Vector2>& WaveParameters::Direction_V() const
  {
    return this->data->directions;
  }
 
  void WaveParameters::DebugPrint() const
  {
    gzmsg << "number:     " << this->data->number << std::endl;
    gzmsg << "scale:      " << this->data->scale << std::endl;
    gzmsg << "angle:      " << this->data->angle << std::endl;
    gzmsg << "period:     " << this->data->period << std::endl;
    gzmsg << "amplitude:  " << this->data->amplitudes << std::endl;
    gzmsg << "wavenumber: " << this->data->wavenumbers << std::endl;
    gzmsg << "omega:      " << this->data->angularFrequencies << std::endl;
    gzmsg << "phase:      " << this->data->phases << std::endl;
    gzmsg << "steepness:  " << this->data->steepnesses << std::endl;
    for (auto&& d : this->data->directions)
    {
      gzmsg << "direction:  " << d << std::endl;
    }
  }

class WavefieldPrivate
{
public: 

    WavefieldPrivate() :
        params(new WaveParameters()),
        size({ 1000, 1000 }),
        cellCount({10, 10}),     
        currentTime(0.0)         
    {
    }

    public: WavefieldPrivate(
        const std::array<double, 2>& _size,
        const std::array<size_t, 2>& _cellCount 
    ) :
        params(new WaveParameters()),
        size(_size),
        cellCount(_cellCount),
        currentTime(0.0)
    {
    }

    public: std::shared_ptr<WaveParameters> params;
  
    public: std::array<double, 2> size;

    public: std::array<size_t, 2> cellCount;

    public: double currentTime;

    public: std::shared_ptr<const Grid> initialGrid;

    public: std::shared_ptr<Grid> grid;
};

  Wavefield::~Wavefield()
  {
  }

  Wavefield::Wavefield(
    const std::string& _name) : 
    data(new WavefieldPrivate())
  {

    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));

    this->Update(0.0);
  }

  Wavefield::Wavefield(
    const std::string& _name,
    const std::array<double, 2>& _size,
    const std::array<size_t, 2>& _cellCount) : 
    data(new WavefieldPrivate(_size, _cellCount))
  {
    this->data->initialGrid.reset(new Grid(
      this->data->size, this->data->cellCount));
    this->data->grid.reset(new Grid(
      this->data->size, this->data->cellCount));
    
    this->Update(0.0);
  }

  std::shared_ptr<const Mesh> Wavefield::GetMesh() const
  {
    return this->data->grid->GetMesh();
  }

  std::shared_ptr<const Grid> Wavefield::GetGrid() const
  {
    return this->data->grid;
  }


  std::shared_ptr<const WaveParameters> Wavefield::GetParameters() const
  {
    return this->data->params;
  }


double Wavefield::SurfaceElevation(double _x, double _y, double _time) const
{

  const auto  number     = this->data->params->Number();
  const auto& amplitude  = this->data->params->Amplitude_V();
  const auto& wavenumber = this->data->params->Wavenumber_V();
  const auto& omega      = this->data->params->AngularFrequency_V();
  const auto& phase      = this->data->params->Phase_V();
  const auto& direction  = this->data->params->Direction_V();

  double surfaceHeight = 0.0;

  for (size_t i = 0; i < number; ++i)
  {
    const auto& amplitude_i = amplitude[i];
    const auto& wavenumber_i = wavenumber[i];
    const auto& omega_i = omega[i];
    const auto& phase_i = phase[i];
    const auto& direction_i = direction[i];


    const double dotProduct = direction_i.x() * _x + direction_i.y() * _y;
    const double angle = dotProduct * wavenumber_i - omega_i * _time + phase_i;
    

    surfaceHeight += amplitude_i * std::cos(angle);
  }

  return surfaceHeight;
}

double Wavefield::SurfaceElevation(double _x, double _y) const
{
  return SurfaceElevation(_x, _y, this->currentTime);
}


  void Wavefield::SetParameters(std::shared_ptr<WaveParameters> _params) const
  {
    GZ_ASSERT(_params != nullptr, "Invalid parameter _params");
    this->data->params = _params;    
  }

  void Wavefield::Update(double _time)
  {
    this->UpdateGerstnerWave(_time);
  

  }

  void Wavefield::UpdateGerstnerWave(double _time)
  {

    const auto  number     = this->data->params->Number();
    const auto& amplitude  = this->data->params->Amplitude_V();
    const auto& wavenumber = this->data->params->Wavenumber_V();
    const auto& omega      = this->data->params->AngularFrequency_V();
    const auto& phase      = this->data->params->Phase_V();
    const auto& q          = this->data->params->Steepness_V();
    const auto& direction  = this->data->params->Direction_V();

    const auto& initMesh = *this->data->initialGrid->GetMesh();
    auto& mesh = *this->data->grid->GetMesh();

    for (
      auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
      it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
      ++it.first, ++it.second)
    {
      auto& vtx0 = *it.first;
      auto& vtx1 = *it.second;
      mesh.point(vtx1) = initMesh.point(vtx0);
    }
    

    for (size_t i=0; i<number; ++i)

    {        
      const auto& amplitude_i = amplitude[i];
      const auto& wavenumber_i = wavenumber[i];
      const auto& omega_i = omega[i];
      const auto& phase_i = phase[i];
      const auto& direction_i = direction[i];
      const auto& q_i = q[i];

      for (
        auto&& it = std::make_pair(std::begin(initMesh.vertices()), std::begin(mesh.vertices()));
        it.first != std::end(initMesh.vertices()) && it.second != std::end(mesh.vertices());
        ++it.first, ++it.second)
      {
        auto& vtx0 = *it.first;
        auto& vtx1 = *it.second;

        const Point3& p0 = initMesh.point(vtx0);
        Vector2 v0(p0.x(), p0.y());      


        const double angle  = CGAL::to_double(direction_i * v0) * wavenumber_i - omega_i * _time + phase_i;
        const double s = std::sin(angle);
        const double c = std::cos(angle);
        Vector3 v1(
          - direction_i.x() * q_i * amplitude_i * s,
          - direction_i.y() * q_i * amplitude_i * s,
          + amplitude_i * c
        );

        mesh.point(vtx1) += v1;
      }
    }


  
  }


  class WavefieldSamplerPrivate
  {

    public: std::shared_ptr<const Wavefield> wavefield;

    public: std::shared_ptr<const Grid> initWaterPatch;

    public: std::shared_ptr<Grid> waterPatch;    
  };

  WavefieldSampler::~WavefieldSampler()
  {        
  }

  WavefieldSampler::WavefieldSampler(
    std::shared_ptr<const Wavefield> _wavefield,
    std::shared_ptr<const Grid> _waterPatch
  ) : data(new WavefieldSamplerPrivate())
  {
    this->data->wavefield = _wavefield;
    this->data->initWaterPatch = _waterPatch;
    this->data->waterPatch.reset(new Grid(*_waterPatch));
  }

  std::shared_ptr<const Grid> WavefieldSampler::GetWaterPatch() const
  {
    return this->data->waterPatch;
  }

void WavefieldSampler::ApplyPose(const ignition::math::Pose3d& _pose)
{

  const double MAX_VALID_POSE = 10000.0;  
  
  if (!std::isfinite(_pose.Pos().X()) || !std::isfinite(_pose.Pos().Y()) ||
      !std::isfinite(_pose.Pos().Z()) ||
      std::abs(_pose.Pos().X()) > MAX_VALID_POSE ||
      std::abs(_pose.Pos().Y()) > MAX_VALID_POSE ||
      std::abs(_pose.Pos().Z()) > MAX_VALID_POSE)
  {
    static int errorCount = 0;
    if (errorCount++ < 10)
    {
      gzerr << "╔════════════════════════════════════════╗" << std::endl;
      gzerr << "║  CRITICAL: Invalid Pose Rejected!     ║" << std::endl;
      gzerr << "╠════════════════════════════════════════╣" << std::endl;
      gzerr << "║  Pose: " << _pose << std::endl;
      gzerr << "║  Max valid distance: ±" << MAX_VALID_POSE << " m" << std::endl;
      gzerr << "╚════════════════════════════════════════╝" << std::endl;
    }
    

    return;
  }


  static int poseCallCount = 0;
  if (poseCallCount++ < 5)
  {
    gzmsg << "╔════════════════════════════════════════╗" << std::endl;
    gzmsg << "║  ApplyPose Call #" << std::setw(3) << poseCallCount << "                  ║" << std::endl;
    gzmsg << "╠════════════════════════════════════════╣" << std::endl;
    gzmsg << "║  Position: (" << std::fixed << std::setprecision(2)
          << _pose.Pos().X() << ", " 
          << _pose.Pos().Y() << ", "
          << _pose.Pos().Z() << ")" << std::endl;
    gzmsg << "║  Distance from origin: " 
          << std::sqrt(_pose.Pos().X()*_pose.Pos().X() + 
                      _pose.Pos().Y()*_pose.Pos().Y())
          << " m" << std::endl;
    gzmsg << "╚════════════════════════════════════════╝" << std::endl;
  }

  const Point3& c0 = this->data->initWaterPatch->GetCenter();
  Point3 c1(c0.x() + _pose.Pos().X(), c0.y() + _pose.Pos().Y(), c0.z());
  this->data->waterPatch->SetCenter(c1);


  auto& source = *this->data->initWaterPatch->GetMesh();
  auto& target = *this->data->waterPatch->GetMesh();
  for (
    auto&& it = std::make_pair(std::begin(source.vertices()), std::begin(target.vertices()));
    it.first != std::end(source.vertices()) && it.second != std::end(target.vertices());
    ++it.first, ++it.second)
  {
    auto& v0 = *it.first;
    auto& v1 = *it.second;
    const Point3& p0 = source.point(v0);

    Point3 p1(p0.x() + _pose.Pos().X(), p0.y() + _pose.Pos().Y(), p0.z());
    target.point(v1) = p1;
  }
}

void WavefieldSampler::UpdatePatch()
{

  if (!this->data->wavefield)
  {
    static bool warned = false;
    if (!warned)
    {
      gzerr << "[WavefieldSampler::UpdatePatch] Wavefield not initialized!" << std::endl;
      warned = true;
    }
    return;
  }

  if (!this->data->waterPatch || !this->data->waterPatch->GetMesh())
  {
    static bool warned = false;
    if (!warned)
    {
      gzerr << "[WavefieldSampler::UpdatePatch] WaterPatch mesh not initialized!" << std::endl;
      warned = true;
    }
    return;
  }

  Direction3 direction(0, 0, 1);
  const auto& target = this->data->waterPatch->GetMesh();
  auto& wavefieldGrid = *this->data->wavefield->GetGrid();
  
  if (!wavefieldGrid.GetMesh())
  {
    gzerr << "[WavefieldSampler] Wavefield grid mesh is null!" << std::endl;
    return;
  }
  

  const auto& waveCenter = wavefieldGrid.GetCenter();
  const auto& waveSize = wavefieldGrid.GetSize();
  const double waveMinX = CGAL::to_double(waveCenter.x()) - waveSize[0] / 2.0;
  const double waveMaxX = CGAL::to_double(waveCenter.x()) + waveSize[0] / 2.0;
  const double waveMinY = CGAL::to_double(waveCenter.y()) - waveSize[1] / 2.0;
  const double waveMaxY = CGAL::to_double(waveCenter.y()) + waveSize[1] / 2.0;
  
  static bool printedBounds = false;
  if (!printedBounds)
  {
    gzmsg << "╔════════════════════════════════════════╗" << std::endl;
    gzmsg << "║  Wavefield Bounds Information         ║" << std::endl;
    gzmsg << "╠════════════════════════════════════════╣" << std::endl;
    gzmsg << "║  Center: (" << std::fixed << std::setprecision(2)
          << CGAL::to_double(waveCenter.x()) << ", " 
          << CGAL::to_double(waveCenter.y()) << ", "
          << CGAL::to_double(waveCenter.z()) << ")" << std::endl;
    gzmsg << "║  Size: " << waveSize[0] << " x " << waveSize[1] << " m" << std::endl;
    gzmsg << "║  X range: [" << waveMinX << ", " << waveMaxX << "]" << std::endl;
    gzmsg << "║  Y range: [" << waveMinY << ", " << waveMaxY << "]" << std::endl;
    gzmsg << "╚════════════════════════════════════════╝" << std::endl;
    printedBounds = true;
  }
  

  int invalidVertexCount = 0;
  int outOfBoundsCount = 0;
  int successCount = 0;
  int vertexIndex = 0;
  int extremeValueCount = 0;  
  
  for (auto&& vb = std::begin(target->vertices()); 
       vb != std::end(target->vertices()); 
       ++vb)
  {
    auto& v1 = *vb;
    Point3& origin = target->point(v1);
    
    const double x = CGAL::to_double(origin.x());
    const double y = CGAL::to_double(origin.y());
    const double z = CGAL::to_double(origin.z());
    

    const double EXTREME_THRESHOLD = 1e10;  
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        std::abs(x) > EXTREME_THRESHOLD || 
        std::abs(y) > EXTREME_THRESHOLD || 
        std::abs(z) > EXTREME_THRESHOLD)
    {
      if (++extremeValueCount <= 3)
      {
        gzerr << "[WavefieldSampler] EXTREME VALUE DETECTED at vertex #" << vertexIndex << std::endl;
        gzerr << "  Coordinates: (" << x << ", " << y << ", " << z << ")" << std::endl;
        gzerr << "  This indicates memory corruption or uninitialized data!" << std::endl;
      }
      
      origin = Point3(
        waveCenter.x(), 
        waveCenter.y(), 
        0.0
      );
      vertexIndex++;
      continue;
    }
    
    if (x < waveMinX || x > waveMaxX || y < waveMinY || y > waveMaxY)
    {
      if (++outOfBoundsCount <= 5)
      {
        gzwarn << "[WavefieldSampler] Vertex #" << vertexIndex 
               << " out of wavefield bounds" << std::endl;
        gzwarn << "  Vertex: (" << x << ", " << y << ", " << z << ")" << std::endl;
        gzwarn << "  Distance from center: " 
               << std::sqrt(std::pow(x - CGAL::to_double(waveCenter.x()), 2) + 
                           std::pow(y - CGAL::to_double(waveCenter.y()), 2))
               << " m" << std::endl;
      }
      vertexIndex++;
      continue;
    }
    

    Point3 point = CGAL::ORIGIN;
    std::array<size_t, 3> cellIndex = { 0, 0, 0 };
    
    bool isFound = GridTools::FindIntersectionIndex(
      wavefieldGrid, origin.x(), origin.y(), cellIndex);
      
    if (!isFound)
    {
      if (++invalidVertexCount <= 3)
      {
        gzwarn << "[WavefieldSampler] Cannot find cell index for vertex #" << vertexIndex << std::endl;
      }
      vertexIndex++;
      continue;
    }
    
    isFound = GridTools::FindIntersectionGrid(
      wavefieldGrid, origin, direction, cellIndex, point);
      
    if (isFound)
    {

      const double px = CGAL::to_double(point.x());
      const double py = CGAL::to_double(point.y());
      const double pz = CGAL::to_double(point.z());
      
      if (std::isfinite(px) && std::isfinite(py) && std::isfinite(pz) &&
          std::abs(px) < EXTREME_THRESHOLD && 
          std::abs(py) < EXTREME_THRESHOLD && 
          std::abs(pz) < EXTREME_THRESHOLD)
      {
        target->point(v1) = point;
        successCount++;
      }
    }
    
    vertexIndex++;
  }
  

  static bool summarized = false;
  if (!summarized || extremeValueCount > 0)
  {
    gzmsg << "╔════════════════════════════════════════╗" << std::endl;
    gzmsg << "║  WavefieldSampler Update Summary      ║" << std::endl;
    gzmsg << "╠════════════════════════════════════════╣" << std::endl;
    gzmsg << "║  Total vertices:       " << std::setw(8) << vertexIndex << "      ║" << std::endl;
    gzmsg << "║  Extreme values:       " << std::setw(8) << extremeValueCount << "      ║" << std::endl;
    gzmsg << "║  Out of bounds:        " << std::setw(8) << outOfBoundsCount << "      ║" << std::endl;
    gzmsg << "║  Invalid intersections:" << std::setw(8) << invalidVertexCount << "      ║" << std::endl;
    gzmsg << "║  Successfully updated: " << std::setw(8) << successCount << "      ║" << std::endl;
    gzmsg << "╚════════════════════════════════════════╝" << std::endl;
    
    
    summarized = true;
  }
}





double WavefieldSampler::ComputeDepth(const Point3& _point) const
{

  if (!internal::IsValidPoint(_point))
  {
    static int errorCount = 0;
    if (errorCount++ < 10) 
    {
      gzerr << "[WavefieldSampler::ComputeDepth] Invalid input point: " 
            << internal::FormatPoint(_point) << std::endl;
      gzerr << "  Max valid coordinate: ±1e6 m" << std::endl;
      gzerr << "  Possible causes:" << std::endl;
      gzerr << "    1. Plugin accessing uninitialized link pose" << std::endl;
      gzerr << "    2. TF lookup failure returning garbage data" << std::endl;
      gzerr << "    3. Plugin Update() called before robot fully loaded" << std::endl;
    }
    return 0.0;
  }


  if (!this->data->waterPatch)
  {
    static bool warned = false;
    if (!warned)
    {
      gzerr << "[WavefieldSampler::ComputeDepth] Water patch not initialized!" << std::endl;
      warned = true;
    }
    return 0.0;
  }

  auto& grid = *this->data->waterPatch;
  return WavefieldSampler::ComputeDepth(grid, _point);
}

  
double WavefieldSampler::ComputeDepth(  
  const Grid& _patch,
  const Point3& _point
)
{

  if (!internal::IsValidPoint(_point))
  {
    static int errorCount = 0;
    if (errorCount++ < 5)
    {
      gzerr << "[WavefieldSampler::ComputeDepth] Invalid point: " 
            << internal::FormatPoint(_point) << std::endl;
      gzerr << "  Max valid coordinate: ±1e6 m" << std::endl;
    }
    return 0.0;
  }


  Direction3 direction(0, 0, 1);
  Point3 wavePoint = CGAL::ORIGIN;
  std::array<size_t, 3> index;
  
  bool isFound = GridTools::FindIntersectionIndex(
    _patch, _point.x(), _point.y(), index);
    
  if (!isFound)
  {

    static int warnCount = 0;
    if (warnCount++ < 5)
    {
      gzwarn << "[WavefieldSampler] Point outside water patch bounds" << std::endl;
      gzwarn << "  Point: " << internal::FormatPoint(_point) << std::endl;
      
      const auto& center = _patch.GetCenter();
      const auto& size = _patch.GetSize();
      gzwarn << "  Patch center: (" 
             << CGAL::to_double(center.x()) << ", " 
             << CGAL::to_double(center.y()) << ", "
             << CGAL::to_double(center.z()) << ")" << std::endl;
      gzwarn << "  Patch size: " << size[0] << " x " << size[1] << " m" << std::endl;
    }
    return 0.0;
  }
  
  isFound = GridTools::FindIntersectionGrid(
    _patch, _point, direction, index, wavePoint);
    
  if (!isFound)
  {
    static int warnCount = 0;
    if (warnCount++ < 5)
    {
      gzwarn << "[WavefieldSampler] No wave surface intersection found" << std::endl;
      gzwarn << "  Point: " << internal::FormatPoint(_point) << std::endl;
    }
    return 0.0;
  }
  
  double h = wavePoint.z() - _point.z();
  return h;
}



double WavefieldSampler::ComputeDepthDirectly(  
    const WaveParameters& _waveParams,
    const Point3& _point,
    double time
  )
  {

    struct WaveParams
    {
      WaveParams(
        const std::vector<double>& _a,
        const std::vector<double>& _k,
        const std::vector<double>& _omega,
        const std::vector<double>& _phi,
        const std::vector<double>& _q,
        const std::vector<Vector2>& _dir) :
        a(_a), k(_k), omega(_omega), phi(_phi), q(_q), dir(_dir) {}

      const std::vector<double>& a;
      const std::vector<double>& k;
      const std::vector<double>& omega;
      const std::vector<double>& phi;
      const std::vector<double>& q;
      const std::vector<Vector2>& dir;
    };


    auto wave_fdf = [=](auto x, auto p, auto t, auto& wp, auto& F, auto& J)
    {
      double pz = 0;
      F(0) = p.x() - x.x();
      F(1) = p.y() - x.y();
      J(0, 0) = -1;
      J(0, 1) =  0;
      J(1, 0) =  0;
      J(1, 1) = -1;
      const size_t n = wp.a.size();
      for (auto&& i=0; i<n; ++i)
      {
        const double dx = wp.dir[i].x();
        const double dy = wp.dir[i].y();
        const double q = wp.q[i];
        const double a = wp.a[i];
        const double k = wp.k[i];
        const double dot = x.x() * dx + x.y() * dy;
        const double theta = k * dot - wp.omega[i] * t;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        const double qakc = q * a * k * c;
        const double df1x = qakc * dx * dx;
        const double df1y = qakc * dx * dy;
        const double df2x = df1y;
        const double df2y = qakc * dy * dy;
        pz += a * c;
        F(0) += a * dx * s;
        F(1) += a * dy * s;
        J(0, 0) += df1x;
        J(0, 1) += df1y;
        J(1, 0) += df2x;
        J(1, 1) += df2y;
      }
      return pz;
    };


    auto solver = [=](auto& fdfunc, auto x0, auto p, auto t, auto& wp, auto tol, auto nmax)
    {
      int n = 0;
      double err = 1;
      double pz = 0;
      auto xn = x0;
      Eigen::Vector2d F;
      Eigen::Matrix2d J;
      while (std::abs(err) > tol && n < nmax)
      {
        pz = fdfunc(x0, p, t, wp, F, J);
        xn = x0 - J.inverse() * F;
        x0 = xn;
        err = F.norm();
        n++;
      }
      return pz;
    };

    WaveParams wp(
      _waveParams.Amplitude_V(),
      _waveParams.Wavenumber_V(),
      _waveParams.AngularFrequency_V(),
      _waveParams.Phase_V(),
      _waveParams.Steepness_V(),
      _waveParams.Direction_V()
    );

    const double tol = 1.0E-10;
    const double nmax = 30;

    Eigen::Vector2d p2(_point.x(), _point.y());
    const double pz = solver(wave_fdf, p2, p2, time, wp, tol, nmax);
    const double h = pz - _point.z();
    return h;
  }


} 
