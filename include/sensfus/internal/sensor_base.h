#ifndef SENSFUS_INTERNAL_SENSOR_BASE_H
#define SENSFUS_INTERNAL_SENSOR_BASE_H

#include "sensfus/types.h"

namespace sensfus {
namespace internal {

template <typename ObjectType = ObjectState2D>
class SensorBase {
 public:
  explicit SensorBase(const SensorIdType &id) : kId(id) {}
  virtual ~SensorBase() = default;

  /// @brief Starts the Sensor throughput (whether real or simulation does not
  /// matter). This will start a thread that publishes new data to the eventbus.
  virtual void StartSensor() = 0;

  /// @brief Stops the simulation but leaves the environment running.
  virtual void HaltSensor() = 0;

  /// @brief Resets the sensor to its initial state. This will stop the
  /// simulation and clear all data.
  virtual void ResetSensor() = 0;

  /// @brief Returns the ID of the sensor.
  /// @return ID of the sensor.
  const SensorIdType &GetId() const { return kId; }

 protected:
  const SensorIdType kId;
};
}  // namespace internal

}  // namespace sensfus

#endif  // SENSFUS_INTERNAL_SENSOR_BASE_H