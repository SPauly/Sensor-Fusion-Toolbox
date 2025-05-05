#include "app/trajectory_planer.h"

#include "sensfus/types.h"
#include "sensfus/sim/trajectory.h"

namespace sensfus {
namespace app {

void TrajectoryPlaner::OnAttach() {}

void TrajectoryPlaner::OnUIRender() {
  ImGui::Begin("Tragectory Planer");

  ImPlot::SetNextAxesLimits(0, 10, 0, 10, ImGuiCond_Once);

  if (ImPlot::BeginPlot("Plot Target Tragectory")) {
    ImPlotPoint mouse = ImPlot::GetPlotMousePos();

    // Add a point on each left mouse click inside the plot
    if (ImPlot::IsPlotHovered() &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      trajectory_.emplace_back((float)mouse.x, (float)mouse.y);
    }

    // Draw the trajectory as lines between points
    if (trajectory_.size() >= 2) {
      std::vector<float> xs, ys;
      for (const auto& pt : trajectory_) {
        xs.push_back(pt.x);
        ys.push_back(pt.y);
      }
      ImPlot::PlotLine("Trajectory", xs.data(), ys.data(), (int)xs.size());
    }

    // Draw the points themselves
    if (!trajectory_.empty()) {
      std::vector<float> xs, ys;
      for (const auto& pt : trajectory_) {
        xs.push_back(pt.x);
        ys.push_back(pt.y);
      }
      ImPlot::PlotScatter("Trajectory Points", xs.data(), ys.data(),
                          (int)xs.size());
    }

    ImPlot::EndPlot();
  }

  ImGui::Text("Saved Trajectory Points: %d", (int)trajectory_.size());
  ImGui::Text("Loaded Points: %d", (int)traj_.GetSize());
  if (ImGui::Button("Clear")) {
    trajectory_.clear();
  }

  if (ImGui::Button("Load")) {
    traj_.FromLineVector(trajectory_);

    // Send trajectory to all registered Sensors
    for (auto& sim : *radar_sim_) {
      sim->PushTrajectory(traj_);
    }
  }

  ImGui::End();
}

void TrajectoryPlaner::OnDetach() {}

}  // namespace app

}  // namespace sensfus
