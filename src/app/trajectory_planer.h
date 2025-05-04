#ifndef TRAJECTORY_PLANER_H
#define TRAJECTORY_PLANER_H

#include <imgui.h>
#include <implot.h>
#include <vector>
#include <memory>

#include "sensfus/types.h"
#include "sensfus/sim/radar_sim.h"
#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class TrajectoryPlaner : public utils::Layer {
 public:
  explicit TrajectoryPlaner(std::shared_ptr<sim::RadarSim> radar_sim)
      : Layer(), plot_flags_(0), axis_flags_(0), radar_sim_(radar_sim) {}
  virtual ~TrajectoryPlaner() = default;

  virtual void OnAttach() override;
  virtual void OnUIRender() override;
  virtual void OnDetach() override;

 private:
  // Style
  ImGuiWindowFlags window_flags_ = ImGuiWindowFlags_NoCollapse;
  ImPlotFlags plot_flags_;
  ImPlotAxisFlags axis_flags_;

  // Safe tragectory
  std::vector<SensVec2D> trajectory_;
  bool drawing_ = false;  // Flag to indicate if the trajectory is being drawn

  // Simulation data
  std::shared_ptr<sim::RadarSim> radar_sim_;
};

}  // namespace app

}  // namespace sensfus

#endif  // TRAJECTORY_PLANER_H