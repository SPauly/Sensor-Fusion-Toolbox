#ifndef RADAR_PLOT_H
#define RADAR_PLOT_H

#include <imgui.h>
#include <implot.h>

#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class RadarPlot : public utils::Layer {
 public:
  explicit constexpr RadarPlot() : Layer(), plot_flags_(0), axis_flags_(0) {}
  virtual ~RadarPlot() = default;

  virtual void OnAttach() override;
  virtual void OnUIRender() override;
  virtual void OnDetach() override;

 private:
  // Style
  ImGuiWindowFlags window_flags_ = ImGuiWindowFlags_NoCollapse;
  ImPlotFlags plot_flags_;
  ImPlotAxisFlags axis_flags_;
};

}  // namespace app

}  // namespace sensfus

#endif  // RADAR_PLOT_H