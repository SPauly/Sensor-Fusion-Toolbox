#include "app/radar_plot.h"

namespace sensfus {
namespace app {

void RadarPlot::OnAttach() {}

void RadarPlot::OnUIRender() {
  ImGui::Begin("Radar Plot");

  if (ImPlot::BeginPlot("Scatter Plot")) {
    ImPlot::EndPlot();
  }

  ImGui::End();
}

void RadarPlot::OnDetach() {}

}  // namespace app

}  // namespace sensfus
