/*
 * World plugin that records the wall-clock cost of every physics step, from
 * WorldUpdateBegin (all model-plugin callbacks) to WorldUpdateEnd (after the
 * physics update). Active only when NEZHA_PROFILE_DIR is set; see
 * nezha_step_profiler.hh.
 */
#include <chrono>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include "nezha_step_profiler.hh"

namespace gazebo {

class NezhaStepTimerPlugin : public WorldPlugin {
 public:
  NezhaStepTimerPlugin() : profiler_("world_step") {}

  void Load(physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/) override {
    if (!profiler_.Enabled()) return;
    begin_ = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo&) { start_ = std::chrono::steady_clock::now(); });
    end_ = event::Events::ConnectWorldUpdateEnd([this]() {
      const auto stop = std::chrono::steady_clock::now();
      profiler_.Add(std::chrono::duration<float, std::micro>(stop - start_).count());
    });
  }

 private:
  nezha::StepProfiler profiler_;
  std::chrono::steady_clock::time_point start_;
  event::ConnectionPtr begin_;
  event::ConnectionPtr end_;
};

GZ_REGISTER_WORLD_PLUGIN(NezhaStepTimerPlugin)

}  // namespace gazebo
