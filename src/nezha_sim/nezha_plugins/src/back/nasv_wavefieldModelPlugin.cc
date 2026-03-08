
#include "nasv_wavefieldModelPlugin.hh"
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include "asv_wave_sim_gazebo_plugins/Convert.hh"
#include "asv_wave_sim_gazebo_plugins/Grid.hh"
#include "asv_wave_sim_gazebo_plugins/Wavefield.hh"
#include "asv_wave_sim_gazebo_plugins/WavefieldEntity.hh"
#include "asv_wave_sim_gazebo_plugins/Utilities.hh"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/msgs/any.pb.h>
#include <gazebo/msgs/empty.pb.h>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/msgs/param.pb.h>
#include <gazebo/msgs/param_v.pb.h>

#include <ignition/math/Vector3.hh>
#include "nezha_getWavefield.hh"  

#include <algorithm>
#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(WavefieldModelPlugin)


  class WavefieldModelPluginPrivate
  {

    public: physics::WorldPtr world;

    public: physics::ModelPtr model;

    public: boost::shared_ptr<::asv::WavefieldEntity> wavefieldEntity;

    public: ignition::msgs::Marker wavefieldMsg;

    public: bool isStatic;

    public: double updateRate;

    public: bool showWavePatch;

    public: Vector2 wavePatchSize;

    public: common::Time prevTime;

    public: event::ConnectionPtr updateConnection;

    public: ignition::transport::Node ignNode;

    public: transport::NodePtr gzNode;

    public: transport::PublisherPtr responsePub;

    public: transport::SubscriberPtr requestSub;

    public: transport::SubscriberPtr waveSub;
  };


  WavefieldModelPlugin::~WavefieldModelPlugin()
  {

    this->data->requestSub.reset();
    this->data->waveSub.reset();
    this->data->responsePub.reset();
    this->data->gzNode->Fini();
    this->Fini();
  }

  WavefieldModelPlugin::WavefieldModelPlugin() : 
    ModelPlugin(), 
    data(new WavefieldModelPluginPrivate())
  {
  }

  void WavefieldModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {


    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    this->data->model = _model;
    this->data->world = _model->GetWorld();
    GZ_ASSERT(this->data->world != nullptr, "Model has invalid World");

    this->data->gzNode = transport::NodePtr(new transport::Node());
    this->data->gzNode->Init(this->data->world->Name());

    this->data->responsePub 
      = this->data->gzNode->Advertise<msgs::Response>("~/response");

    this->data->requestSub = this->data->gzNode->Subscribe(
      "~/request", &WavefieldModelPlugin::OnRequest, this);

    this->data->waveSub = this->data->gzNode->Subscribe(
      "~/wave", &WavefieldModelPlugin::OnWaveMsg, this);

    this->data->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WavefieldModelPlugin::OnUpdate, this));

    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);
    this->data->updateRate = Utilities::SdfParamDouble(*_sdf, "update_rate", 30.0);
    if (_sdf->HasElement("markers"))
    {
      sdf::ElementPtr sdfMarkers = _sdf->GetElement("markers");
      this->data->showWavePatch = Utilities::SdfParamBool(*sdfMarkers, "wave_patch", false);
      this->data->wavePatchSize = Utilities::SdfParamVector2(*sdfMarkers, "wave_patch_size", Vector2(4, 4));
    }

    this->data->wavefieldEntity.reset(new ::asv::WavefieldEntity(this->data->model));
    this->data->wavefieldEntity->Load(_sdf);
    this->data->wavefieldEntity->Init();

    this->data->wavefieldEntity->SetName(
      WavefieldEntity::MakeName(this->data->model->GetName()));
    this->data->model->AddChild(this->data->wavefieldEntity);
    gzdbg << "[WavefieldModelPlugin] Created WavefieldEntity with name: "
      << this->data->wavefieldEntity->GetName()
      << " under model: " << this->data->model->GetName() << "\n";

if (this->data->wavefieldEntity && this->data->wavefieldEntity->GetWavefield())
{

  asv::RegisterWavefield(this->data->model->GetName(),
                         this->data->wavefieldEntity->GetWavefield());
  gzdbg << "[WavefieldModelPlugin] Registered Wavefield for model: "
        << this->data->model->GetName() << "\n";
}
else
{
  gzerr << "[WavefieldModelPlugin] wavefieldEntity or its Wavefield is null, register failed.\n";
}

  }

  void WavefieldModelPlugin::Init()
  {


    if (this->data->showWavePatch)
      this->InitMarker();
  }

  void WavefieldModelPlugin::Fini()
  {
    if (this->data->showWavePatch)
      this->FiniMarker();
     asv::UnregisterWavefield(this->data->model->GetName());
  }

  void WavefieldModelPlugin::Reset()
  {

    this->data->prevTime = this->data->world->SimTime(); 

    if (this->data->showWavePatch)
      this->ResetMarker();
  }

  void WavefieldModelPlugin::OnUpdate()
  {
    GZ_ASSERT(this->data->world != nullptr, "World is NULL");
    GZ_ASSERT(this->data->model != nullptr, "Model is NULL");
    GZ_ASSERT(this->data->wavefieldEntity != nullptr, "Wavefield Entity is NULL");

    if (!this->data->isStatic)
    {
      auto updatePeriod = 1.0/this->data->updateRate;
      auto currentTime = this->data->world->SimTime();
      if ((currentTime - this->data->prevTime).Double() < updatePeriod)
      {
        return;
      }
      this->data->prevTime = currentTime; 

      this->data->wavefieldEntity->Update();
      if (this->data->showWavePatch)
        this->UpdateMarker();
    }
  }

  void WavefieldModelPlugin::OnRequest(ConstRequestPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Request message must not be null");
    
    if (_msg->request() == "wave_param")
    {
      auto waveParams = this->data->wavefieldEntity->GetWavefield()->GetParameters();

      msgs::Param_V waveMsg;
      waveParams->FillMsg(waveMsg);

      msgs::Response response;
      response.set_id(_msg->id());
      response.set_request(_msg->request());
      response.set_response("success");
      std::string *serializedData = response.mutable_serialized_data();
      response.set_type(waveMsg.GetTypeName());
      waveMsg.SerializeToString(serializedData);
      this->data->responsePub->Publish(response);
    }
  }

  void WavefieldModelPlugin::OnWaveMsg(ConstParam_VPtr &_msg)
  {
    GZ_ASSERT(_msg != nullptr, "Wave message must not be null");

    auto constWaveParams = this->data->wavefieldEntity->GetWavefield()->GetParameters();
    GZ_ASSERT(constWaveParams != nullptr, "WaveParameters must not be null");
    auto& waveParams = const_cast<WaveParameters&>(*constWaveParams);
    waveParams.SetFromMsg(*_msg);

    gzmsg << "Wavefield Model received message on topic ["
      << this->data->waveSub->GetTopic() << "]" << std::endl;
    waveParams.DebugPrint();
  }

  void WavefieldModelPlugin::InitMarker()
  {
    auto& wavefieldMsg = this->data->wavefieldMsg;

    int markerId = 0;
    wavefieldMsg.set_ns("wave");
    wavefieldMsg.set_id(markerId++);
    wavefieldMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    wavefieldMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
    std::string waveMat("Gazebo/RedTransparent");
    ignition::msgs::Material *waveMatMsg = wavefieldMsg.mutable_material();
    GZ_ASSERT(waveMatMsg != nullptr, "Invalid Material pointer from wavefieldMsg");
    waveMatMsg->mutable_script()->set_name(waveMat);
  }

  void WavefieldModelPlugin::FiniMarker()
  {
    std::string topicName("/marker");

    auto& ignNode = this->data->ignNode;
    auto& wavefieldMsg = this->data->wavefieldMsg;

    wavefieldMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
    ignNode.Request(topicName, wavefieldMsg);
  }

  void WavefieldModelPlugin::ResetMarker()
  {
    std::string topicName("/marker");

    auto& ignNode = this->data->ignNode;
    auto& wavefieldMsg = this->data->wavefieldMsg;
 
    wavefieldMsg.mutable_point()->Clear();
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignition::msgs::Set(wavefieldMsg.add_point(), ignition::math::Vector3d::Zero);
    ignNode.Request(topicName, wavefieldMsg);
  }

  void WavefieldModelPlugin::UpdateMarker()
  {
    std::string topicName("/marker");

    auto grid = this->data->wavefieldEntity->GetWavefield()->GetGrid();
    auto& wavefieldMsg = this->data->wavefieldMsg;
    auto& ignNode = this->data->ignNode;

    int xext = static_cast<int>(this->data->wavePatchSize[0])/2;
    int xmid = grid->GetCellCount()[0]/2;
    int xbeg = std::max(0, xmid-xext);
    int xend = std::min(static_cast<int>(grid->GetCellCount()[0]), xmid+xext);

    int yext = static_cast<int>(this->data->wavePatchSize[1])/2;
    int ymid = grid->GetCellCount()[1]/2;
    int ybeg = std::max(0, ymid-yext);
    int yend = std::min(static_cast<int>(grid->GetCellCount()[1]), ymid+yext);

    wavefieldMsg.mutable_point()->Clear();
    for (int ix=xbeg; ix<xend; ++ix)
    {
      for (int iy=ybeg; iy<yend; ++iy)
      {
        for (int k=0; k<2; ++k)
        {
          Triangle tri(grid->GetTriangle(ix, iy, k));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[0]));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[1]));
          ignition::msgs::Set(wavefieldMsg.add_point(), ToIgn(tri[2]));
        }
      }
    }
    ignNode.Request(topicName, wavefieldMsg);
  }

} 
