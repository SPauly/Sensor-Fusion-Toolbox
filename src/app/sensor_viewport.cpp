#include "app/sensor_viewport.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {

void SensorViewport::RegisterPlotCallback(const std::string name,
                                          const SensorPlotCallback cb) {
  plot_callbacks_.push_back(
      std::make_pair<const std::string, const SensorPlotCallback>(
          std::move(name), std::move(cb)));
  suspendet_.push_back(
      std::make_pair<const std::string, bool>(std::move(name), false));
}

void SensorViewport::OnAttach() {}

void SensorViewport::OnDetach() {}

void SensorViewport::OnUIRender() {
  ImGui::Begin("Sensor Fusion Viewport");

  ImPlot::SetNextAxesLimits(-300, 300, -300, 300, ImGuiCond_Once);

  if (ImPlot::BeginPlot("Fused Sensor Plot")) {
    // Call all registered sensor plot callbacks
    for (size_t i = 0; i < plot_callbacks_.size(); i++) {
      if (!suspendet_[i].second) plot_callbacks_[i].second();
    }
    ImPlot::EndPlot();
  }

  ImGui::Separator();

  ImGui::Text("Select which Plots are shown: ");

  // select which plots are actually shown:
  for (size_t i = 0; i < plot_callbacks_.size(); i++) {
    ImGui::Checkbox(suspendet_[i].first.c_str(), &suspendet_[i].second);
  }

  ImGui::End();
}

}  // namespace app
}  // namespace sensfus