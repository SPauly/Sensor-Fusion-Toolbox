#include "app/radar_plot.h"

namespace sensfus {
namespace app {

void RadarPlot::OnAttach() { radar_sim_->Init(); }

void RadarPlot::OnUIRender() {
  ImGui::Begin("Radar Plot");

  static bool sim_running = false;

  // Start simulation on button press
  if (ImGui::Button("Start Radar Simulation")) {
    if (!sim_running) {
      sim_running = true;
      radar_sim_->StartSimulation();
    }
  }

  ImGui::Text("Trajectories: %d", (int)radar_sim_->GetTrajectorySize());
  ImGui::Text("State data: %d", (int)state_.truth.size());

  if (ImPlot::BeginPlot("Radar Frame")) {
    // Display the latest frame if available
    if (radar_sim_->HasUpdate()) {
      ImGui::Text("Has Update");
      state_ = radar_sim_->GetState();

      for (const auto& obj : state_.truth) {
        x_truth.push_back(obj(0));
        y_truth.push_back(obj(1));
      }
    }

    if (!x_truth.empty())
      ImPlot::PlotScatter("Truth", x_truth.data(), y_truth.data(),
                          (int)x_truth.size());

    ImPlot::EndPlot();
  }

  ImGui::End();
}

void RadarPlot::OnDetach() {}

}  // namespace app

}  // namespace sensfus
