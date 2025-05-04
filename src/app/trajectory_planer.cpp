#include "app/trajectory_planer.h"

namespace sensfus {
namespace app {

void TrajectoryPlaner::OnAttach() {}

void TrajectoryPlaner::OnUIRender() {
  ImGui::Begin("Tragectory Planer");

  ImPlot::SetNextAxesLimits(0, 10, 0, 10, ImGuiCond_Once);

  if (ImPlot::BeginPlot("Plot Target Tragectory")) {
    ImPlotPoint mouse = ImPlot::GetPlotMousePos();

    // Handle left mouse button input inside plot
    if (ImPlot::IsPlotHovered()) {
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        trajectory_.clear();
        trajectory_.push_back(ImVec2((float)mouse.x, (float)mouse.y));
        drawing_ = true;
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && drawing_) {
        // Only add if the point is different from the last
        ImVec2 last = trajectory_.back();
        if (last.x != (float)mouse.x || last.y != (float)mouse.y) {
          trajectory_.push_back(ImVec2((float)mouse.x, (float)mouse.y));
        }
      } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && drawing_) {
        drawing_ = false;
      }
    }

    // Draw the trajectory as lines
    if (!trajectory_.empty()) {
      std::vector<float> xs, ys;
      for (const auto& pt : trajectory_) {
        xs.push_back(pt.x);
        ys.push_back(pt.y);
      }
      ImPlot::PlotLine("Trajectory", xs.data(), ys.data(), (int)xs.size());
    }

    ImPlot::EndPlot();
  }

  ImGui::Text("Saved Trajectory Points: %d", (int)trajectory_.size());
  if (ImGui::Button("Clear")) {
    trajectory_.clear();
  }

  ImGui::End();
}

void TrajectoryPlaner::OnDetach() {}

}  // namespace app

}  // namespace sensfus
