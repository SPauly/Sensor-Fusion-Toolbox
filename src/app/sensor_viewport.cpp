#include "app/sensor_viewport.h"
#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {

void SensorViewport::RegisterPlotCallback(const SensorPlotCallback& cb) {
  plot_callbacks_.push_back(cb);
}

void SensorViewport::OnAttach() {}

void SensorViewport::OnDetach() {}

void SensorViewport::OnUIRender() {
  ImGui::Begin("Sensor Fusion Viewport");
  if (ImPlot::BeginPlot("Fused Sensor Plot")) {
    // Call all registered sensor plot callbacks
    for (auto& cb : plot_callbacks_) {
      if (cb) cb();
    }
    ImPlot::EndPlot();
  }
  ImGui::End();
}

}  // namespace app
}  // namespace sensfus