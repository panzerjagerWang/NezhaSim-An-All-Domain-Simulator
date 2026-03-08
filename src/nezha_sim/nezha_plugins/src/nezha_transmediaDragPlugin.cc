
#include "nezha_transmediaDragPlugin.hh"

#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>

#include <nezha_plugins/GetPhaseSample.h>


using nlohmann::json;

namespace gazebo {
namespace mini_expr {

  struct Token { enum Type { NUM, VAR, FUNC, OP, LP, RP, END } type; std::string s; double v{0}; };
  static inline bool A(char c){ return std::isalpha((unsigned char)c); }
  static inline bool N(char c){ return std::isalnum((unsigned char)c); }
  static inline bool S(char c){ return std::isspace((unsigned char)c); }
  struct Lexer {
    const char* p; explicit Lexer(const std::string& src):p(src.c_str()){}
    Token next(){ while(*p&&S(*p))++p; if(!*p)return{Token::END,"",0};
      if(std::isdigit(*p)||*p=='.'){ char* e; double val=strtod(p,&e); Token t{Token::NUM,"",val}; p=e; return t; }
      if(A(*p)){ const char* b=p; while(N(*p))++p; std::string id(b,p); if(id=="x0"||id=="x1") return {Token::VAR,id,0}; return {Token::FUNC,id,0}; }
      if(*p=='('){++p;return{Token::LP,"(",0};} if(*p==')'){++p;return{Token::RP,")",0};}
      return {Token::OP,std::string(1,*p++),0}; }
  };
static int prec(const std::string& o){
  if (o == "^") return 4;
  if (o == "*" || o == "/") return 3;
  if (o == "+" || o == "-") return 2;
  return 0;
}
  static bool right_assoc(const std::string& o){ return o=="^"; }
  using F1 = double(*)(double);
  static std::unordered_map<std::string,F1>& fmap(){ static std::unordered_map<std::string,F1> m{
    {"sin",sin},{"cos",cos},{"tanh",tanh},{"exp",exp},{"log",log},{"sqrt",sqrt},{"abs",fabs}
  }; return m; }
  struct Node{ enum K{NUM,VAR,UN,BIN}k; double val; std::string s; F1 f; Node*a=nullptr,*b=nullptr; Node(K k,double v=0):k(k),val(v),f(nullptr){} };
  struct AST{
    Node*root=nullptr; ~AST(){destroy(root);} static void destroy(Node*n){if(!n)return;destroy(n->a);destroy(n->b);delete n;}
    double eval(double x0,double x1)const{
std::function<double(Node*)> E = [&](Node* n)->double {
  if (!n) return 0.0;
  if (n->k == Node::NUM) return n->val;
  if (n->k == Node::VAR) return (n->s == "x0") ? x0 : x1;

  if (n->k == Node::UN) {
    return n->f(E(n->a));
  }

  if (n->k == Node::BIN) {
    const double L = E(n->a);
    const double R = E(n->b);
    if      (n->s == "+") return L + R;
    else if (n->s == "-") return L - R;
    else if (n->s == "*") return L * R;
    else if (n->s == "/") return (R == 0.0) ? 0.0 : (L / R);
    else if (n->s == "^") return std::pow(L, R);
    else                  return 0.0;
  }

  return 0.0;
};

      return E(root);
    }
  };
  static AST parse(const std::string& src){
    Lexer lex(src); std::vector<Token> out, ops;
    for(;;){ Token t=lex.next(); if(t.type==Token::END)break;
      if(t.type==Token::NUM||t.type==Token::VAR) out.push_back(t);
      else if(t.type==Token::FUNC) ops.push_back(t);
      else if(t.type==Token::OP){
        while(!ops.empty()){ auto&o=ops.back();
          if(o.type==Token::OP && (prec(o.s)>prec(t.s) || (prec(o.s)==prec(t.s)&&!right_assoc(t.s)))){ out.push_back(o); ops.pop_back(); } else break; }
        ops.push_back(t);
      } else if(t.type==Token::LP) ops.push_back(t);
      else if(t.type==Token::RP){
        while(!ops.empty()&&ops.back().type!=Token::LP){ out.push_back(ops.back()); ops.pop_back(); }
        if(!ops.empty()&&ops.back().type==Token::LP) ops.pop_back();
        if(!ops.empty()&&ops.back().type==Token::FUNC){ out.push_back(ops.back()); ops.pop_back(); }
      }
    }
    while(!ops.empty()){ out.push_back(ops.back()); ops.pop_back(); }
    std::vector<Node*> st;
    for(auto&t:out){
      if(t.type==Token::NUM) st.push_back(new Node(Node::NUM,t.v));
      else if(t.type==Token::VAR){ auto*n=new Node(Node::VAR); n->s=t.s; st.push_back(n); }
      else if(t.type==Token::FUNC){ auto*a=st.back(); st.pop_back(); auto*n=new Node(Node::UN); n->a=a; auto it=fmap().find(t.s); n->f=(it==fmap().end()?[](double x){return x;}:it->second); st.push_back(n); }
      else if(t.type==Token::OP){ auto*b=st.back(); st.pop_back(); auto*a=st.back(); st.pop_back(); auto*n=new Node(Node::BIN); n->a=a; n->b=b; n->s=t.s; st.push_back(n); }
    }
    AST ast; ast.root = st.empty()? nullptr : st.back(); return ast;
  }
} 


TransMediaDragPlugin::TransMediaDragPlugin() = default;

TransMediaDragPlugin::~TransMediaDragPlugin() {
  if (ros::isStarted())
    ros::shutdown();
}


double TransMediaDragPlugin::DragModel::Eval(double v_abs, double z_over_l) const {
switch (this->kind) {
    case DragModel::Kind::Poly3:

      return coeffs[0]
           + coeffs[1]*v_abs
           + coeffs[2]*z_over_l
           + coeffs[3]*v_abs*v_abs
           + coeffs[4]*v_abs*z_over_l
           + coeffs[5]*z_over_l*z_over_l
           + coeffs[6]*v_abs*v_abs*v_abs
           + coeffs[7]*v_abs*v_abs*z_over_l
           + coeffs[8]*v_abs*z_over_l*z_over_l
           + coeffs[9]*z_over_l*z_over_l*z_over_l;
    case DragModel::Kind::Quadratic: {
      const double base = std::pow(v_abs, exponent);
      return 0.5 * rho * Cd * A * base;
    }
    default:
      return 0.0;
  }
}


bool TransMediaDragPlugin::LoadModelFromJson(const std::string& path,
                                             DragModel& out,
                                             std::string* err) {
  try {
    std::ifstream ifs(path);
if (!ifs) {
  if (err) *err = "cannot open file";
  return false;
}
    json j; ifs >> j;

if (!j.contains("piecewise") || !j["piecewise"].contains("boundary_on_z_over_l")) {
  if (err) *err = "missing piecewise.boundary_on_z_over_l";
  return false;
}
    out.kind = DragModel::Kind::PiecewiseCTR;
    out.boundary_on_z_over_l = j["piecewise"]["boundary_on_z_over_l"].get<double>();

    if (!j.contains("theta")) { if (err) *err = "missing theta[22]"; return false; }
    auto th = j["theta"];
    if (!th.is_array() || th.size()!=22) { if (err) *err = "theta must be length-22"; return false; }
    for (size_t i=0;i<22;++i) out.theta[i] = th.at(i).get<double>();
    out.has_theta = true;

    out.use_residual = true;
    if (j.contains("equation_residual") && !j["equation_residual"].is_null()) {
      const std::string expr = j["equation_residual"].get<std::string>();
out.residual_ast = std::make_shared<mini_expr::AST>(mini_expr::parse(expr));

    } else {
      out.use_residual = false;
    }
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}

void TransMediaDragPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;

  std::string baseLinkName = "base_link";
  if (_sdf->HasElement("baseLinkName"))
    baseLinkName = _sdf->Get<std::string>("baseLinkName");

  if (_sdf->HasElement("waterSurfaceZ"))
    waterSurfaceZ = _sdf->Get<double>("waterSurfaceZ");   

  if (_sdf->HasElement("updateRate"))
    updateRate = _sdf->Get<double>("updateRate");

  if (_sdf->HasElement("robotNamespace"))
    robotNamespace = _sdf->Get<std::string>("robotNamespace");

  if (_sdf->HasElement("zTopOffset"))
    zTopOffset = _sdf->Get<double>("zTopOffset");
  if (_sdf->HasElement("characteristicLen"))
    characteristicLen = _sdf->Get<double>("characteristicLen");
  if (_sdf->HasElement("ctrScale"))
    ctrScale = _sdf->Get<double>("ctrScale");
  if (_sdf->HasElement("alwaysPublish"))
    alwaysPublish = _sdf->Get<bool>("alwaysPublish");

  if (_sdf->HasElement("surfaceZService"))
    surfaceZService = _sdf->Get<std::string>("surfaceZService");
  else
    surfaceZService = "/transmedia/get_phase_sample";


  std::string packageName = "nezha_plugins";
  if (_sdf->HasElement("packageName"))
    packageName = _sdf->Get<std::string>("packageName");
  const std::string pkgPath = ros::package::getPath(packageName);
  if (pkgPath.empty()) {
    gzerr << "[TransMediaDragPlugin] rospack find failed for package '"
          << packageName << "'. You can set <packageName> or explicit <entryJson/...>.\n";
  }

  entryJson = pkgPath + "/Settings/Transmeida_liftdrag_force/Water_Entry.json";
  exitJson  = pkgPath + "/Settings/Transmeida_liftdrag_force/Water_Exit.json";
  aboveJson = pkgPath + "/Settings/Transmeida_liftdrag_force/Water_above.json";
  if (_sdf->HasElement("entryJson")) entryJson = _sdf->Get<std::string>("entryJson");
  if (_sdf->HasElement("exitJson"))  exitJson  = _sdf->Get<std::string>("exitJson");
  if (_sdf->HasElement("aboveJson")) aboveJson = _sdf->Get<std::string>("aboveJson");

  baseLink = model->GetLink(baseLinkName);
  if (!baseLink) {
    gzerr << "[TransMediaDragPlugin] Link '" << baseLinkName << "' not found!\n";
    return;
  }

  std::string err;
  if (!LoadModelFromJson(entryJson, entryModel, &err))
    gzerr << "[TransMediaDragPlugin] Failed to load " << entryJson << ": " << err << "\n";
  err.clear();
  if (!LoadModelFromJson(exitJson, exitModel, &err))
    gzerr << "[TransMediaDragPlugin] Failed to load " << exitJson << ": " << err << "\n";
  err.clear();
  if (!LoadModelFromJson(aboveJson, aboveModel, &err))
    gzerr << "[TransMediaDragPlugin] Failed to load " << aboveJson << ": " << err << "\n";

  if (!ros::isInitialized()) {
    int argc = 0; char** argv = nullptr;
    ros::init(argc, argv, "transmedia_drag_plugin", ros::init_options::NoSigintHandler);
  }
  nh = std::make_unique<ros::NodeHandle>(robotNamespace);


surfaceClient = nh->serviceClient<nezha_plugins::GetPhaseSample>(surfaceZService, /*persistent=*/true);


  std::string dragTopic = "transmedia_drag";
  if (_sdf->HasElement("dragTopic"))
    dragTopic = _sdf->Get<std::string>("dragTopic");
  dragPub = nh->advertise<geometry_msgs::Vector3Stamped>(dragTopic, 10);

  lastUpdateTime = model->GetWorld()->SimTime();
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TransMediaDragPlugin::OnUpdate, this));

  gzdbg << "[TransMediaDragPlugin] Loaded (service-only). JSON:\n"
        << "  entry : " << entryJson << "\n"
        << "  exit  : " << exitJson  << "\n"
        << "  above : " << aboveJson << "\n"
        << "  surfaceZService: " << surfaceZService << "\n";
}


bool TransMediaDragPlugin::QueryPhaseSample(double x, double y,
                                            double& outSurfaceZ,
                                            double& outZOverL,
                                            Phase& outPhase)
{

  if (!surfaceClient.exists()) {
    surfaceClient.waitForExistence(ros::Duration(0.0));
  }

  nezha_plugins::GetPhaseSample srv;

  if (surfaceClient.call(srv)) {
    outSurfaceZ = srv.response.surface_z;
    outZOverL   = srv.response.z_over_l;

    const std::string& name = srv.response.phase_name;
    if      (name == "ABOVE")       outPhase = Phase::Above;
    else if (name == "WATER_ENTRY") outPhase = Phase::Entry;
    else if (name == "WATER_EXIT")  outPhase = Phase::Exit;
    else if (name == "BELOW") {
      outPhase = Phase::None;
    } else {
      outPhase = Phase::None;
    }

    lastSurfaceZ = outSurfaceZ;
    haveSurfaceZ = true;
    return true;
  }


  if (haveSurfaceZ) {
    outSurfaceZ = lastSurfaceZ;
  } else {
    outSurfaceZ = waterSurfaceZ;
  }
return false;
}


void TransMediaDragPlugin::OnUpdate() {
  if (!baseLink) {
    return;
  }

  gazebo::common::Time now = model->GetWorld()->SimTime();
  if ((now - lastUpdateTime).Double() < (1.0 / updateRate)) {
    return;
  }

  lastUpdateTime = now;

  const ignition::math::Pose3d pose = baseLink->WorldPose();
  const ignition::math::Vector3d vel = baseLink->WorldLinearVel();
  const double xPos = pose.Pos().X();
  const double yPos = pose.Pos().Y();
  const double vAbs = vel.Length();
  const double vZ   = vel.Z();


double surfaceZ = waterSurfaceZ;
double zOverL   = 0.0;
Phase  phase    = Phase::None;

QueryPhaseSample(xPos, yPos, surfaceZ, zOverL, phase);


if (phase == Phase::None) {
  if (zOverL > 0.0) {
    phase = (vZ <= 0.0) ? Phase::Entry : Phase::Exit;
  } else {
    phase = Phase::Above;
  }
}



  const DragModel* m = nullptr;
  switch (phase) {
    case Phase::Entry: m = &entryModel; break;
    case Phase::Exit:  m = &exitModel;  break;
    case Phase::Above: m = &aboveModel; break;
    default: break;
  }

  lastForce = ignition::math::Vector3d::Zero;
  if (m && m->kind != DragModel::Kind::None && vAbs > 1e-4) {
    const double predict_scalar = std::max(0.0, m->Eval(vAbs, zOverL));
    const double dragMag = ctrScale * predict_scalar;
    if (dragMag > 0.0) {
      const ignition::math::Vector3d dir = -vel.Normalized();
      lastForce = dir * dragMag;
      baseLink->AddForce(lastForce);
    }
  }

  if (alwaysPublish || lastForce != ignition::math::Vector3d::Zero) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = lastForce.X();
    msg.vector.y = lastForce.Y();
    msg.vector.z = lastForce.Z();
    dragPub.publish(msg);
  }
}

GZ_REGISTER_MODEL_PLUGIN(TransMediaDragPlugin)

} 


