// WaveProbePlugin.cc
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include "Wavefield.hh" 
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"


namespace asv
{
class WaveProbePlugin : public gazebo::ModelPlugin
{
  struct Data
  {
    gazebo::physics::ModelPtr           model;
    gazebo::physics::WorldPtr           world;
    boost::shared_ptr<asv::WavefieldEntity> wavefield;
    gazebo::transport::NodePtr          gzNode;
    gazebo::transport::PublisherPtr     surfaceZPub;
  };

public:
  WaveProbePlugin() : data(new Data) {}
  virtual ~WaveProbePlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->data->model = _model;
    this->data->world = _model->GetWorld();

    // 1) 解析 SDF 参数
    std::string wavefieldName = "wavefield";
    if (_sdf->HasElement("wavefield_name"))
      wavefieldName = _sdf->Get<std::string>("wavefield_name");

    // 2) 获取 WavefieldEntity
    this->data->wavefield =
        boost::dynamic_pointer_cast<asv::WavefieldEntity>(
            this->data->world->EntityByName(wavefieldName));

    if (!this->data->wavefield)
      gzerr << "[WaveProbePlugin] Cannot find WavefieldEntity named \""
            << wavefieldName << "\". Plugin will be inactive.\n";

    // 3) transport node
    this->data->gzNode.reset(new gazebo::transport::Node());
    this->data->gzNode->Init(this->data->world->Name());


// 4) 发布器 - 使用 Vector3d 类型
std::string boatName = this->data->model->GetName();
std::string topic = "/wave/" + boatName + "/surface_z";
this->data->surfaceZPub =
    this->data->gzNode->Advertise<gazebo::msgs::Vector3d>(topic);



    gzdbg << "[WaveProbePlugin] Advertised topic " << topic << '\n';

    // 5) 绑定更新回调
    this->updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&WaveProbePlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/)
  {
    if (!this->data->surfaceZPub            ||
        !this->data->surfaceZPub->HasConnections() ||
        !this->data->wavefield)
      return;

#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d pos = this->data->model->WorldPose().Pos();
#else
    gazebo::math::Vector3 pos = this->data->model->GetWorldPose().pos;
#endif
    double x = pos.X();
    double y = pos.Y();

    double z = this->data->wavefield->GetWavefield()->SurfaceElevation(x, y);

    // 使用 Vector3d 消息类型，只设置 z 分量
    gazebo::msgs::Vector3d msg;
    msg.set_x(0.0);
    msg.set_y(0.0);
    msg.set_z(z);
    this->data->surfaceZPub->Publish(msg);
  }



private:
  std::unique_ptr<Data>               data;
  gazebo::event::ConnectionPtr        updateConn;
};

GZ_REGISTER_MODEL_PLUGIN(WaveProbePlugin)
} // namespace asv

