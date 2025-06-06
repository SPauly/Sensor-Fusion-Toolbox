#include "app/trajectory_planer.h"

#include "sensfus/types.h"

namespace sensfus {
namespace app {

void TrajectoryPlaner::OnAttach() {}

void TrajectoryPlaner::OnUIRender() {
  ImGui::Begin("Tragectory Planer", nullptr, window_flags_);

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
        xs.push_back(pt(0));
        ys.push_back(pt(1));
      }
      ImPlot::PlotLine("Trajectory", xs.data(), ys.data(), (int)xs.size());
    }

    // Draw the points themselves
    if (!trajectory_.empty()) {
      std::vector<float> xs, ys;
      for (const auto& pt : trajectory_) {
        xs.push_back(pt(0));
        ys.push_back(pt(1));
      }
      ImPlot::PlotScatter("Trajectory Points", xs.data(), ys.data(),
                          (int)xs.size());
    }

    ImPlot::EndPlot();
  }

  ImGui::Text("Saved Trajectory Points: %d", (int)trajectory_.size());
  if (traj_.size() > 0)
    ImGui::Text("Loaded Points: %d", (int)traj_.back()->GetSize());
  if (ImGui::Button("Clear")) {
    trajectory_.clear();
  }

  if (ImGui::Button("Load")) {
    traj_.push_back(sim_->CreateTrajectoryFromVec2D(trajectory_));
  }

  if (ImGui::Button("Create Wafe Pattern")) {
    traj_.push_back(sim_->CreateTrajectoryFromVec2D(
        trajectory_, sim::ObjectModelType::WaveModel));
  }

  ImGui::End();

  WaveTrajSettings();
}

void TrajectoryPlaner::OnDetach() {}

void TrajectoryPlaner::WaveTrajSettings() {}

}  // namespace app

}  // namespace sensfus
