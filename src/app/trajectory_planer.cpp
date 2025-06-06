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

void TrajectoryPlaner::WaveTrajSettings() {
  ImGui::Begin("Wave Trajectory Settings");
  static float speed = 1.0f;
  static float acceleration = 0.5f;
  static float period = 2.0f;

  ImGui::SliderFloat("Speed", &speed, 0.1f, 10.0f, "%.2f");
  ImGui::SliderFloat("Acceleration", &acceleration, 0.0f, 5.0f, "%.2f");
  ImGui::SliderFloat("Period", &period, 0.1f, 10.0f, "%.2f");

  if (!traj_.empty() &&
      (speed != speed_ || acceleration != acceleration_ || period != period_)) {
    speed_ = speed;
    acceleration_ = acceleration;
    period_ = period;

    std::static_pointer_cast<sim::WaveModel2D>(traj_.back()->GetObjectModel())
        ->SetSpeed(speed);
    std::static_pointer_cast<sim::WaveModel2D>(traj_.back()->GetObjectModel())
        ->SetAcceleration(speed);
    std::static_pointer_cast<sim::WaveModel2D>(traj_.back()->GetObjectModel())
        ->SetPeriod(period);
  }

  ImGui::End();
}

}  // namespace app

}  // namespace sensfus
