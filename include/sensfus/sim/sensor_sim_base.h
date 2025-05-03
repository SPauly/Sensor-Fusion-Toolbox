#ifndef SENSFUS_SIM_SENSOR_SIM_BASE_H
#define SENSFUS_SIM_SENSOR_SIM_BASE_H

namespace sensfus {
namespace sim {

/// @brief Base class for sensor simulation
class SensorSimBase {
 public:
  SensorSimBase() = default;
  virtual ~SensorSimBase() = default;

  // Method to initialize the sensor simulation
  virtual void initialize() = 0;

  // Method to simulate sensor data
  virtual void simulateData() = 0;

  // Method to get the simulated data
  virtual void getData() const = 0;

 protected:
  // Protected members can be added here
};

}  // namespace sim

}  // namespace sensfus

#endif  // SENSFUS_SIM_SENSOR_SIM_BASE_H