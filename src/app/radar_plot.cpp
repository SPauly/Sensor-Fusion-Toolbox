#include "app/radar_plot.h"

namespace sensfus {
namespace app {

void RadarPlot::OnAttach() {}

void RadarPlot::OnUIRender() {
  ImGui::Begin("Radar Plot");

  static bool sim_running = false;

  // Display the iteration step of each sensor
  for (int i = 0; i < radar_sim_->size(); i++) {
    ImGui::Text("Iteration Step of Sensor: %d -> %d", i,
                radar_sim_->at(i)->GetStepIndex());
  }

  if (ImPlot::BeginPlot("Radar Data")) {
    // Display the Truth only once
    if (radar_sim_->at(0)->HasUpdate()) {
      state_[0] = radar_sim_->at(0)->GetState();

      // check if that we have enough room for trajectoris
      x_truth.resize(radar_sim_->at(0)->GetTrajectoryCount());
      y_truth.resize(radar_sim_->at(0)->GetTrajectoryCount());

      // safe the x and y attributes for each trajectory
      for (int i = 0; i < state_[0].truth.size(); i++) {
        x_truth[i].push_back(state_[0].truth[i](0));
        y_truth[i].push_back(state_[0].truth[i](1));
      }
    }

    /// TODO: Update all the sensor data

    // Display the truth
    if (!x_truth.empty()) {
      DisplayTargets();
    }

    /// TODO: Plot all the sensor data

    ImPlot::EndPlot();
  }

  ImGui::End();
}

void RadarPlot::OnDetach() {
  x_truth.clear();
  y_truth.clear();
}

void RadarPlot::DisplayTargets() {
  /// TODO: create a plot with different sign for each data point
  for (int i = 0; i < x_truth.size(); i++) {
    const char *label = ("Target" + (char)(i + 65));
    ImPlot::PlotScatter(label, x_truth.at(i).data(), y_truth.at(i).data(),
                        (int)x_truth.at(i).size());
  }
}

}  // namespace app

}  // namespace sensfus
