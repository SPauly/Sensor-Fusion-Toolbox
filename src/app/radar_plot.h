#ifndef RADAR_PLOT_H
#define RADAR_PLOT_H

#include <imgui.h>
#include <implot.h>
#include <memory>
#include <vector>

#include "sensfus/sim/radar_sim.h"
#include "app/utils/layer.h"

namespace sensfus {
namespace app {
class RadarPlot : public utils::Layer {
 public:
  explicit RadarPlot(
      std::shared_ptr<std::vector<std::shared_ptr<sim::RadarSim>>> radar_sim)
      : Layer(), plot_flags_(0), axis_flags_(0), radar_sim_(radar_sim) {
    state_.resize(radar_sim->size());
    x_truth.resize(1);
    y_truth.resize(1);
    x_cartesian.resize(1);
    y_cartesian.resize(1);
    range_a_x_.resize(1);
    range_a_y_.resize(1);
  }
  virtual ~RadarPlot() = default;

  virtual void OnAttach() override;
  virtual void OnUIRender() override;
  virtual void OnDetach() override;

  /// @brief Interface that provides the imgui control of each radar sim
  /// @param id id of the sensor to control
  void RunRadarControl(int id);

 protected:
  virtual void DisplayTargets();

 private:
  // Style
  ImGuiWindowFlags window_flags_ = ImGuiWindowFlags_NoCollapse;
  ImPlotFlags plot_flags_;
  ImPlotAxisFlags axis_flags_;

  // one radar sim per sensor
  std::shared_ptr<std::vector<std::shared_ptr<sim::RadarSim>>> radar_sim_;
  // one state per sensor
  std::vector<sim::RadarSimState> state_;

  /// Holds one vector per trajectory, with all the x,y values
  std::vector<std::vector<double>> x_truth, y_truth;
  std::vector<std::vector<double>> x_cartesian, y_cartesian, range_a_x_,
      range_a_y_;  // Cartesian coordinates as estimated by the sensor

  // Control variables for gui
  std::vector<float> stddev_cartesian_, stddev_range_,
      stddev_azimuth_;  // Standard deviation of the sensor measurements
  std::vector<float> pos_x_, pos_y_, pos_z_;  // Position of the sensor
};

}  // namespace app

}  // namespace sensfus

#endif  // RADAR_PLOT_H