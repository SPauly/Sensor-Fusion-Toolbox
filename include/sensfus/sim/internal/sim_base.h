#ifndef SIM_INTERNAL_SENSOR_SIM_BASE_H
#define SIM_INTERNAL_SENSOR_SIM_BASE_H

namespace sensfus {
namespace sim {
namespace internal {

/// @brief Base class for sensor simulation. This class is used to create a
/// simulation environment for sensors. The simulation environment is used to
/// coordinate the simulation input for the sensors, create and handle different
/// Sensor types and controll the simulation flow.
class SimBase {
 public:
  explicit SimBase() = default;
  virtual ~SimBase() noexcept = default;

  /// @brief Starts the actual simulation. This must be called after Init() and
  /// should be called in the main thread.
  virtual void StartSimulation() = 0;

  /// @brief Stops the simulation but leaves the environment running.
  virtual void HaltSimulation() = 0;

  /// @brief Reset the Simulation environment. Call StartSimulation to restart
  /// it. This does not ensure the same simulation outcome as the previous run
  /// but simply resets to the starting point.
  virtual void ResetSimulation() = 0;
};

}  // namespace internal
}  // namespace sim
}  // namespace sensfus

#endif  // SIM_INTERNAL_SENSOR_SIM_BASE_H