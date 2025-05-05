#ifndef RADAR_PLOT_H
#define RADAR_PLOT_H

#include <imgui.h>
#include <implot.h>
#include <memory>

#include "sensfus/sim/radar_sim.h"
#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class RadarPlot : public utils::Layer {
 public:
  explicit RadarPlot(std::shared_ptr<sim::RadarSim> radar_sim)
      : Layer(), plot_flags_(0), axis_flags_(0), radar_sim_(radar_sim) {}
  virtual ~RadarPlot() = default;

  virtual void OnAttach() override;
  virtual void OnUIRender() override;
  virtual void OnDetach() override;

 private:
  // Style
  ImGuiWindowFlags window_flags_ = ImGuiWindowFlags_NoCollapse;
  ImPlotFlags plot_flags_;
  ImPlotAxisFlags axis_flags_;

  std::shared_ptr<sim::RadarSim> radar_sim_;

  std::vector<double> x_truth, y_truth;
};

}  // namespace app

}  // namespace sensfus

#endif  // RADAR_PLOT_H