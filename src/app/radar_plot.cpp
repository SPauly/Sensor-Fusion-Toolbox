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

void RadarPlot::RunPlotInterface() {
  if (show_cartesian_) {
    // small scatter points (visual clarity)
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.0f);
    ImPlot::PlotScatter(plot_cartesian_label_.c_str(), x_cartesian.data(),
                        y_cartesian.data(), (int)x_cartesian.size());
  }
  if (show_range_) {
    // small scatter points (visual clarity)
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.0f);
    ImPlot::PlotScatter(plot_range_label_.c_str(), range_a_x_.data(),
                        range_a_y_.data(), (int)range_a_x_.size());
  }
}

void RadarPlot::AddSensorUpdate(
    const std::shared_ptr<const RadarSensorInfo2D> sensor_update) {
  // Add the data for fast access to the plot
  for (int i = 0; i < sensor_update->cartesian.size(); i++) {
    x_cartesian.push_back(sensor_update->cartesian.at(i)(0));
    y_cartesian.push_back(sensor_update->cartesian.at(i)(1));
  }

  for (int i = 0; i < sensor_update->range_azimuth.size(); i++) {
    // Convert range and azimuth to cartesian coordinates
    // x = r*cos(a), y = r*sin(a) + sensor position
    range_a_x_.push_back(sensor_update->range_azimuth.at(i)(0) *
                             std::cos(sensor_update->range_azimuth.at(i)(1)) +
                         pos_x_);
    range_a_y_.push_back(sensor_update->range_azimuth.at(i)(0) *
                             std::sin(sensor_update->range_azimuth.at(i)(1)) +
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
  ImGui::SliderFloat(std_range_label_.c_str(), &stddev_range_, 0.0f, 100.0f,
                     "%.3f");
  ImGui::SliderFloat(std_azimuth_label_.c_str(), &stddev_azimuth_, 0.0f, 2.0f,
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
