#ifndef RADAR_PLOT_H
#define RADAR_PLOT_H

#include <imgui.h>
#include <implot.h>
#include <memory>
#include <vector>
#include <functional>
#include <utility>

#include "sensfus/sim/sensor_simulator.h"
#include "sensfus/sim/sensor_radar.h"
#include "sensfus/types.h"

namespace sensfus {
namespace app {
class RadarPlot {
 public:
  explicit RadarPlot(size_t id, std::shared_ptr<sim::SensorSimulator> sim,
                     std::shared_ptr<sim::SensorRadar> radar);
  virtual ~RadarPlot() = default;

  /// @brief Interface that provides the imgui control for this sensor
  void RunControllInterface();

  /// @brief Interface that provides the implot output of the sensor
  void RunPlotInterface();

  /// @brief Returns the callback for the sensor plot
  /// @return Callback for the sensor plot
  std::function<void()> GetCallback() {
    return std::bind(&RadarPlot::RunPlotInterface, this);
  }

  void AddSensorUpdate(
      const std::shared_ptr<const RadarSensorInfo2D> sensor_update);

 private:
  // ID of the sensor
  size_t id_ = 0;

  // one radar sim per sensor
  std::shared_ptr<sim::SensorRadar> radar_;
  std::shared_ptr<sim::SensorSimulator> sim_;

  // Safe the state of the radar sim for fast display
  std::vector<double> x_cartesian, y_cartesian, range_a_x_,
      range_a_y_;  // Cartesian coordinates as estimated by the sensor

  // Control variables for gui
  float stddev_cartesian_ = 5.0, stddev_range_ = 2.0,
        stddev_azimuth_ =
            0.02;  // Standard deviation of the sensor measurements
  float pos_x_ = 0.0, pos_y_ = 0.0;  // Position of the sensor

  bool show_cartesian_ = true,
       show_range_ = true;  // Show the cartesian and range plot

  // Gui stuff
  std::string std_cartesian_label_, std_range_label_,
      std_azimuth_label_;  // Labels for the standard deviation
  std::string pos_x_label_,
      pos_y_label_;  // Labels for the position of the sensor
  std::string start_label_, stop_label_, apply_label_;
  std::string plot_cartesian_label_,
      plot_range_label_;  // Labels for the plot
};

}  // namespace app

}  // namespace sensfus

#endif  // RADAR_PLOT_H