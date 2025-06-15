#ifndef TRAJECTORY_PLANER_H
#define TRAJECTORY_PLANER_H

#include <imgui.h>
#include <implot.h>
#include <vector>
#include <memory>

#include "sensfus/types.h"
#include "sensfus/sim/sensor_simulator.h"
#include "sensfus/sim/trajectory.h"
#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class TrajectoryPlaner : public utils::Layer {
 public:
  explicit TrajectoryPlaner(std::shared_ptr<sim::SensorSimulator> sim)
      : Layer(), plot_flags_(0), axis_flags_(0), sim_(sim) {}
  virtual ~TrajectoryPlaner() = default;

  virtual void OnAttach() override;
  virtual void OnUIRender() override;
  virtual void OnDetach() override;

 protected:
  void WaveTrajSettings();

 private:
  // Style
  ImGuiWindowFlags window_flags_ = ImGuiWindowFlags_NoCollapse;
  ImPlotFlags plot_flags_;
  ImPlotAxisFlags axis_flags_;

  // Safe tragectory
  std::vector<ObjectPosition2D> trajectory_;
  bool drawing_ = false;  // Flag to indicate if the trajectory is being drawn

  // Simulation data
  std::shared_ptr<sim::SensorSimulator> sim_;
  std::vector<std::shared_ptr<sim::Trajectory<ObjectState2D>>> traj_;

  // temporary variables for wave trajectory settings
  float speed_ = 1.0f;         // Speed in m/s
  float acceleration_ = 0.0f;  // Acceleration in m/s^2
  float period_ = 2.0f;        // Period in seconds
};

}  // namespace app

}  // namespace sensfus

#endif  // TRAJECTORY_PLANER_H