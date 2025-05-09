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

  ImPlot::SetNextAxesLimits(0, 10, 0, 10, ImGuiCond_Once);

  if (ImPlot::BeginPlot("Radar Data")) {
    // Display the Truth only once
    if (radar_sim_->at(0)->HasUpdate()) {
      state_[0] = radar_sim_->at(0)->GetState();

      // check if that we have enough room for trajectoris
      x_truth.resize(radar_sim_->at(0)->GetTrajectoryCount());
      y_truth.resize(radar_sim_->at(0)->GetTrajectoryCount());
      for (int i = 1; i < radar_sim_->size(); i++) {
        x_cartesian.resize(radar_sim_->at(i)->GetTrajectoryCount());
        y_cartesian.resize(radar_sim_->at(i)->GetTrajectoryCount());
        range_a_x_.resize(radar_sim_->at(i)->GetTrajectoryCount());
        range_a_y_.resize(radar_sim_->at(i)->GetTrajectoryCount());
        state_.resize(radar_sim_->size());
      }

      // safe the x and y attributes for each trajectory
      for (auto &pair : state_[0].truth) {
        x_truth[pair.first].push_back(pair.second(0));
        y_truth[pair.first].push_back(pair.second(1));
      }

      // Since we already fetched the state for radar 0 we now need to also
      // display the data fully directly

      // safe the x and y attributes for each trajectory
      for (int i = 0; i < state_[0].sensor.zx.size(); i++) {
        x_cartesian[0].push_back(state_[0].sensor.zx.at(i));
        y_cartesian[0].push_back(state_[0].sensor.zy.at(i));

        // Convert range and azimuth to cartesian coordinates
        // x = r*cos(a), y = r*sin(a) + sensor position
        range_a_x_[0].push_back(state_[0].sensor.range.at(i) *
                                    std::cos(state_[0].sensor.azimuth.at(i)) +
                                pos_x_.at(0));
        range_a_y_[0].push_back(state_[0].sensor.range.at(i) *
                                    std::sin(state_[0].sensor.azimuth.at(i)) +
                                pos_y_.at(0));
      }
    }

    // Update all the sensor data
    // Display the truth
    if (!x_truth.empty()) {
      DisplayTargets();
    }

    /// TODO: Plot all the sensor data
    std::string label = "Sensor Cartesian" + std::to_string(0);
    ImPlot::PlotScatter(label.c_str(), x_cartesian[0].data(),
                        y_cartesian[0].data(), (int)x_cartesian[0].size());

    std::string label2 = "Sensor Range Azimuth" + std::to_string(0);
    ImPlot::PlotScatter(label2.c_str(), range_a_x_[0].data(),
                        range_a_y_[0].data(), (int)range_a_x_[0].size());
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
    std::string label = "Target";
    label += static_cast<char>(i + 65);
    ImPlot::PlotScatter(label.c_str(), x_truth.at(i).data(),
                        y_truth.at(i).data(), (int)x_truth.at(i).size());
  }
}

void RadarPlot::RunRadarControl(int id) {
  ImGui::Text("Radar %d", id);
  ImGui::SameLine();
  std::string start_label = "Start##" + std::to_string(id);
  if (ImGui::Button(start_label.c_str())) {
    radar_sim_->at(id)->StartSimulation();
  }
  ImGui::SameLine();
  std::string stop_label = "Stop##" + std::to_string(id);
  if (ImGui::Button(stop_label.c_str())) {
    radar_sim_->at(id)->Stop();
  }

  // Display the current status of the sensor
  ImGui::Text("Sensor Status: %s",
              (radar_sim_->at(id)->IsRunning()) ? "Running" : "Stopped");

  // Check that we have enough space for the noise parameters
  if (stddev_cartesian_.size() <= id) {
    stddev_cartesian_.resize(id + 1);
    stddev_range_.resize(id + 1);
    stddev_azimuth_.resize(id + 1);

    pos_x_.resize(id + 1);
    pos_y_.resize(id + 1);
    pos_z_.resize(id + 1);
  }

  ImGui::Text("Noise Parameters:");

  std::string std_cartesian_label = "Std Cartesian##" + std::to_string(id);
  std::string std_range_label = "Std Range##" + std::to_string(id);
  std::string std_azimuth_label = "Std Azimuth##" + std::to_string(id);

  ImGui::SliderFloat(std_cartesian_label.c_str(), &stddev_cartesian_.at(id),
                     0.0f, 100.0f, "%.3f");
  ImGui::SliderFloat(std_range_label.c_str(), &stddev_range_.at(id), 0.0f,
                     10.0f, "%.3f");
  ImGui::SliderFloat(std_azimuth_label.c_str(), &stddev_azimuth_.at(id), 0.0f,
                     1.0f, "%.4f");

  std::string set_noise_label = "Apply Noise Settings##" + std::to_string(id);

  if (ImGui::Button(set_noise_label.c_str())) {
    radar_sim_->at(id)->SetStdCartesianDeviation(stddev_cartesian_.at(id));
    radar_sim_->at(id)->SetStdRangeDeviation(stddev_range_.at(id));
    radar_sim_->at(id)->SetStdAzimuthDeviation(stddev_azimuth_.at(id));
  }

  ImGui::Text("Position Settings:");

  std::string pos_x_label = "Pos X##" + std::to_string(id);
  std::string pos_y_label = "Pos Y##" + std::to_string(id);
  std::string pos_z_label = "Pos Z##" + std::to_string(id);

  ImGui::SliderFloat(pos_x_label.c_str(), &pos_x_.at(id), -100.0f, 100.0f,
                     "%.2f");
  ImGui::SliderFloat(pos_y_label.c_str(), &pos_y_.at(id), -100.0f, 100.0f,
                     "%.2f");
  ImGui::SliderFloat(pos_z_label.c_str(), &pos_z_.at(id), -10.0f, 10.0f,
                     "%.2f");

  std::string apply_pos_label = "Apply Position##" + std::to_string(id);

  if (ImGui::Button(apply_pos_label.c_str())) {
    SensVec2D pos = {pos_x_.at(id), pos_y_.at(id)};
    radar_sim_->at(id)->SetSensorPosition(pos);
  }
}

}  // namespace app

}  // namespace sensfus
