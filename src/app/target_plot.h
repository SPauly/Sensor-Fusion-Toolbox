#ifndef TARGET_PLOT_H
#define TARGET_PLOT_H

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "sensfus/sim/sensor_simulator.h"
#include "sensfus/types.h"

namespace sensfus {
namespace app {

class TargetPlot {
 public:
  explicit TargetPlot(size_t id, std::shared_ptr<sim::SensorSimulator> sim)
      : id_(id), sim_(std::move(sim)) {};
  virtual ~TargetPlot() = default;

  /// @brief Interface that provides the imgui control for this sensor
  void RunControllInterface();

  /// @brief Interface that provides the implot output of the sensor
  void RunPlotInterface();

  /// @brief Returns the callback for the sensor plot
  /// @return Callback for the sensor plot
  std::function<void()> GetCallback() {
    return std::bind(&TargetPlot::RunPlotInterface, this);
  }

  /// @brief Add a new Target update to the Plot
  /// @param update New Data
  void AddTargetUpdate(const std::shared_ptr<TrueTargetState2D> update);

 protected:
  virtual void DisplayTargets();

 private:
  // ID of the sensor
  size_t id_ = 0;

  std::shared_ptr<sim::SensorSimulator> sim_;

  // Safe the targets id at index i
  std::unordered_map<TargetIdType, size_t> id_of_target_at_index_;

  // Safes all the cart positions
  std::vector<std::vector<double>> cart_x_, cart_y_;

  // Only safe the current velocity and acceleration
  std::vector<double> velo_x_, velo_y_, acc_x_, acc_y_;

  std::vector<std::string> labels_;
};

}  // namespace app

}  // namespace sensfus

#endif  // TARGET_PLOT_H