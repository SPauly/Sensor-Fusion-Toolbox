#ifndef TRAJECTORY_PLANER_H
#define TRAJECTORY_PLANER_H

#include <imgui.h>
#include <implot.h>
#include <vector>

#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class TrajectoryPlaner : public utils::Layer {
 public:
  explicit constexpr TrajectoryPlaner()
      : Layer(), plot_flags_(0), axis_flags_(0) {}
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
  std::vector<ImVec2> trajectory_;
  bool drawing_ = false;  // Flag to indicate if the trajectory is being drawn
};

}  // namespace app

}  // namespace sensfus

#endif  // TRAJECTORY_PLANER_H