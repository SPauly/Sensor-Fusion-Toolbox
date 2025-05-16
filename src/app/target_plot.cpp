#include "app/target_plot.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {
void TargetPlot::RunControllInterface() {
  /// TODO: Implement the control interface for the target plot to feature
  /// restart etc.

  // Display Target Positions and Velocities
  for (size_t i = 0; i < id_of_target_at_index_.size(); i++) {
    ImGui::Text("Status Target %f", id_of_target_at_index_.at(i));

    ImGui::Text("Cartesian Position: x: %f  y: %f", cart_x_.at(i).back(),
                cart_y_.at(i).back());

    // Add the rest
    ImGui::Separator();
  }
}

void TargetPlot::RunPlotInterface() {}
}  // namespace app

}  // namespace sensfus
