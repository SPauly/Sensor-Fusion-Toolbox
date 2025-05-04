#ifndef SIM_SENSOR_SIM_BASE_H
#define SIM_SENSOR_SIM_BASE_H

namespace sensfus {
namespace sim {

template <typename TargetStates, typename SensorState>
struct SimState {
 public:
  TargetStates truth;
  SensorState sensor;
};

/// @brief Base class for sensor simulation. This class is used to create a
/// simulation environment for sensors. The simulation environment is used to
/// simulate the sensor data and to provide a way to test the sensor fusion
/// algorithms.
class SimBase {
 public:
  explicit SimBase() = default;
  virtual ~SimBase() = default;

  /// @brief Starts the simulation environment. (This might start a thread or a
  /// process)
  virtual void Init() = 0;

  /// @brief Starts the actual simulation. This must be called after Init() and
  /// should be called in the main thread.
  virtual void StartSimulation() = 0;

  /// @brief Stops the simulation but leaves the environment running.
  virtual void HaltSimulation() = 0;

  /// @brief Stops the simulation and the environment. After this call, a new
  /// call to Init is required.
  virtual void Stop() = 0;

  /// @brief Changes the update rate of the simulation.
  /// @param rate_in_ns The new update rate in nanoseconds.
  virtual void ChangeUpdateRate(double rate_in_ns) = 0;
};

}  // namespace sim

}  // namespace sensfus

#endif  // SIM_SENSOR_SIM_BASE_H