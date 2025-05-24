#ifndef SIM_SENSOR_RADAR_H
#define SIM_SENSOR_RADAR_H

#include <vector>
#include <thread>
#include <mutex>
#include <utility>  // std::pair

#include "sensfus/types.h"
#include "sensfus/utils/eventbus.h"
#include "sensfus/internal/sensor_base.h"

namespace sensfus {
namespace sim {

class SensorRadar : public internal::SensorBase<ObjectState2D> {
 public:
  explicit SensorRadar(const SensorIdType& id,
                       std::shared_ptr<utils::EventBus> event_bus);
  virtual ~SensorRadar() noexcept;

  /// @brief Starts the Sensor throughput (whether real or simulation does not
  /// matter). This will start a thread that publishes new data to the eventbus.
  virtual void StartSensor() override;

  /// @brief Stops the simulation but leaves the environment running.
  virtual void HaltSensor() override;

  /// @brief Resets the sensor to its initial state. This will stop the
  /// simulation and clear all data.
  virtual void ResetSensor() override;

  // Getters

  /// @brief Returns true if the simulation is running.
  /// @return true if the simulation is running, false otherwise.
  bool IsRunning() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return start_;
  }

  /// @brief Returns the current Simulation step
  /// @return Step of the simulation.
  inline TimeStepIdType GetStepIndex() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return curr_index_;
  }

  /// @brief Returns the current Sensor position
  /// @return Position of the sensor in 2D.
  inline const ObjectPosition2D GetSensorPosition() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return radar_position_;
  }

  /// @brief Returns the cartesian standard deviation of the sensor.
  /// @return Standard deviation of the cartesian measurement.
  inline const double GetStdCartesianDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return cartesian_std_dev_;
  }

  /// @brief Returns the standard deviation of the range measurement.
  /// @return Standard deviation of the range measurement.
  inline const double GetStdRangeDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return range_std_dev_;
  }

  /// @brief Returns the standard deviation of the azimuth measurement.
  /// @return standard deviation of the azimuth measurement.
  inline const double GetStdAzimuthDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return azimuth_std_dev_;
  }

  // Setters

  inline void SetStepIndex(TimeStepIdType step_index) {
    std::unique_lock<std::mutex> lock(mtx_);
    curr_index_ = step_index;
  }

  /// @brief Sets the update rate of the simulation. This is the rate at which
  /// the simulation will run.
  /// @param rate_ns Update rate in nanoseconds.
  inline void SetUpdateRate(double rate_ns) {
    std::unique_lock<std::mutex> lock(mtx_);
    update_rate_ = rate_ns;
  }

  /// @brief Sets the position of the sensor. This will be taken into account
  /// for range and azimuth but not cartesian coordinates.
  /// @param pos Position of the sensor in 2D.
  inline void SetSensorPosition(const ObjectPosition2D& pos) {
    std::unique_lock<std::mutex> lock(mtx_);
    radar_position_ = pos;
  }

  /// @brief Sets the standard deviation of the cartesian measurement.
  /// @param std_dev Standard deviation of the cartesian measurement.
  inline void SetStdCartesianDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    cartesian_std_dev_ = std_dev;
  }

  /// @brief Sets the standard deviation of the range measurement.
  /// @param std_dev Standard deviation of the range measurement.
  inline void SetStdRangeDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    range_std_dev_ = std_dev;
  }

  /// @brief Sets the standard deviation of the azimuth measurement.
  /// @param std_dev Standard deviation of the azimuth measurement.
  inline void SetStdAzimuthDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    azimuth_std_dev_ = std_dev;
  }

 protected:
  virtual void RunImpl();

 private:
  // Sharing sensor data
  std::shared_ptr<utils::EventBus> event_bus_;
  std::shared_ptr<utils::Publisher<RadarSensorInfo2D>> radar_pub_;
  std::shared_ptr<utils::Channel<TrueTargetState2D>::Subscription> target_sub_;

  // Simulation step
  double update_rate_ = 0.0;  // Update rate of the simulation in nanoseconds
  TimeStepIdType curr_index_ = 0;  // Current index of the trajectory

  bool start_ = false;        // Flag to indicate if the simulation is running
  bool should_stop_ = false;  // Flag to indicate if the simulation should stop

  // Thread control variables
  std::thread sim_thread_;
  std::condition_variable cv_start_;
  mutable std::mutex mtx_;

  // Sensor deviation
  double cartesian_std_dev_ =
      0.0;  // Standard deviation of the cartesian coordinates
  double range_std_dev_ = 0.0;  // Standard deviation of the range measurement
  double azimuth_std_dev_ =
      0.0;  // Standard deviation of the azimuth measurement
  Eigen::Matrix<ScalarType, 2, 6> H_;

  // Sensor position
  ObjectPosition2D radar_position_ =
      ObjectPosition2D::Zero();  // Position of the sensor in 2D
};
}  // namespace sim

}  // namespace sensfus

#endif  // SIM_SENSOR_RADAR_H