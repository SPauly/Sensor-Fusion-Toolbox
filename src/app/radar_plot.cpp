#include "app/radar_plot.h"

namespace sensfus {
namespace app {

RadarPlot::RadarPlot(size_t id, std::shared_ptr<sim::SensorSimulator> sim,
                     std::shared_ptr<sim::SensorRadar> radar)
    : id_(id), sim_(sim), radar_(radar) {
  // Init all the labels for the gui
  std_cartesian_label_ = "Std Cartesian##" + std::to_string(id_);
  std_range_label_ = "Std Range##" + std::to_string(id_);
  std_azimuth_label_ = "Std Azimuth##" + std::to_string(id_);
  pos_x_label_ = "Pos X##" + std::to_string(id_);
  pos_y_label_ = "Pos Y##" + std::to_string(id_);
  start_label_ = "Start##" + std::to_string(id_);
  stop_label_ = "Stop##" + std::to_string(id_);
  apply_label_ = "Apply Changes##" + std::to_string(id_);

  plot_cartesian_label_ =
      "Radar " + std::to_string(id_) + " Cartesian##" + std::to_string(id_);
  plot_range_label_ =
      "Radar " + std::to_string(id_) + " Range##" + std::to_string(id_);
}

void RadarPlot::OnUIRender() {
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

      // Update all the sensor data
      // Display the truth
      if (!x_truth.empty()) {
        DisplayTargets();
      }

      ImPlot::EndPlot();
    }

    ImGui::End();
  }
}

void RadarPlot::DisplayTargets() {
  /// TODO: create a plot with different sign for each data point
  for (int i = 0; i < x_truth.size(); i++) {
    std::string label = "Target";
    label += static_cast<char>(i + 65);
    ImPlot::PlotScatter(label.c_str(), x_truth.at(i).data(),
                        y_truth.at(i).data(), (int)x_truth.at(i).size());
  }
#

  void RadarPlot::RunPlotInterface() {
    if (show_cartesian_) {
      ImPlot::PlotScatter(plot_cartesian_label_.c_str(), x_cartesian.data(),
                          y_cartesian.data(), (int)x_cartesian.size());
    }
    if (show_range_) {
      ImPlot::PlotScatter(plot_range_label_.c_str(), range_a_x_.data(),
                          range_a_y_.data(), (int)range_a_x_.size());
    }
  }

  void RadarPlot::AddSensorUpdate(
      const std::shared_ptr<RadarSensorInfo2D> sensor_update) {
    // Add the data for fast access to the plot
    for (int i = 0; i < sensor_update->cart_x.size(); i++) {
      x_cartesian.push_back(sensor_update->cart_x.at(i));
      y_cartesian.push_back(sensor_update->cart_y.at(i));
    }

    for (int i = 0; i < sensor_update->range.size(); i++) {
      // Convert range and azimuth to cartesian coordinates
      // x = r*cos(a), y = r*sin(a) + sensor position
      range_a_x_.push_back(sensor_update->range.at(i) *
                               std::cos(sensor_update->azimuth.at(i)) +
                           pos_x_);
      range_a_y_.push_back(sensor_update->range.at(i) *
                               std::sin(sensor_update->azimuth.at(i)) +
                           pos_y_);
    }
  }

  void RadarPlot::RunControllInterface() {
    ImGui::Text("Radar %d", id_);
    ImGui::SameLine();
    std::string start_label = "Start##" + std::to_string(id_);
    if (ImGui::Button(start_label.c_str())) {
      radar_->StartSensor();
    }
    ImGui::SameLine();
    std::string stop_label = "Stop##" + std::to_string(id_);
    if (ImGui::Button(stop_label.c_str())) {
      radar_->HaltSensor();
    }

    ImGui::Separator();

    ImGui::Checkbox("Show Cartesian", &show_cartesian_);
    ImGui::SameLine();
    ImGui::Checkbox("Show Range And Azimuth", &show_range_);

    // Display the current status of the sensor
    ImGui::Text("Sensor Status: %s",
                (radar_->IsRunning()) ? "Running" : "Stopped");
    ImGui::Text("Iteration Step: %d", radar_->GetStepIndex());

    ImGui::Text("Noise Parameters:");

    ImGui::SliderFloat(std_cartesian_label_.c_str(), &stddev_cartesian_, 0.0f,
                       100.0f, "%.3f");
    ImGui::SliderFloat(std_range_label_.c_str(), &stddev_range_, 0.0f, 10.0f,
                       "%.3f");
    ImGui::SliderFloat(std_azimuth_label_.c_str(), &stddev_azimuth_, 0.0f, 1.0f,
                       "%.4f");

    ImGui::Text("Position Settings:");

    ImGui::SliderFloat(pos_x_label_.c_str(), &pos_x_, -100.0f, 100.0f, "%.2f");
    ImGui::SliderFloat(pos_y_label_.c_str(), &pos_y_, -100.0f, 100.0f, "%.2f");

    if (ImGui::Button(apply_label_.c_str())) {
      radar_->SetStdCartesianDeviation(stddev_cartesian_);
      radar_->SetStdRangeDeviation(stddev_range_);
      radar_->SetStdAzimuthDeviation(stddev_azimuth_);
      radar_->SetSensorPosition({pos_x_, pos_y_});
    }
  }

}  // namespace app

}  // namespace sensfus
