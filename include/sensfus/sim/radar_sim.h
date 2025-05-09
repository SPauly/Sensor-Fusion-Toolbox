#ifndef SENSFUS_SIM_RADAR_SIM_H
#define SENSFUS_SIM_RADAR_SIM_H

#include <vector>
#include <thread>
#include <mutex>
#include <utility>  // std::pair

#include "sensfus/types.h"
#include "sensfus/sim/sim_base.h"
#include "sensfus/sim/object_model.h"
#include "sensfus/sim/trajectory.h"

namespace sensfus {
namespace sim {

struct RadarDataType {
  std::vector<ScalarType> zx, zy;          // Measurement in 2D
  std::vector<ScalarType> range, azimuth;  // Measurement of range and azimuth

  void Clear() {
    zx.clear();
    zy.clear();
    range.clear();
    azimuth.clear();
  }
};

// Holds the truth as trajectory positions and the sensor data measured in this
// time frame
using RadarSimState =
    SimState<std::vector<std::pair<size_t, ObjectState2D>>, RadarDataType>;

class RadarSim : public SimBase {
 public:
  explicit RadarSim() = default;
  virtual ~RadarSim() noexcept = default;

  virtual void Init() override;
  virtual void StartSimulation() override;
  virtual void HaltSimulation() override;
  virtual void Stop() override;

  /// @brief Pushes a trajectory to the simulation. This trajectory will be used
  /// to simulate the sensor data.
  /// @param traj Trajectory will be started with the next simulation step.
  virtual void PushTrajectory(const Trajectory<ObjectState2D>& traj) {
    std::unique_lock<std::mutex> lock(mtx_);
    trajectories_.push_back(traj);

    // store the index offset of the trajectory
    traj_index_offset_.push_back(curr_index_);
  }

  // Getters

  /// @brief Returns the current simulation state. This includes the truth and
  /// the sensor simulated data.
  virtual RadarSimState GetState();

  /// @brief Returns true if the simulation is running.
  /// @return true if the simulation is running, false otherwise.
  bool IsRunning() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return start_;
  }

  /// @brief Returns true if the simulation has an update.
  /// @return true if the simulation has an update, false otherwise.
  virtual bool HasUpdate() const;

  /// @brief Returns the current Simulation step
  /// @return Step of the simulation.
  virtual unsigned long long GetStepIndex() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return curr_index_;
  }

  /// @brief Returns the number of different trajectories in the simulation.
  /// @return
  virtual const size_t GetTrajectoryCount() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return trajectories_.size();
  }

  /// @brief Returns the current Sensor position
  /// @return Position of the sensor in 2D.
  const SensVec2D& GetSensorPosition() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return radar_position_;
  }

  /// @brief Returns the cartesian standard deviation of the sensor.
  /// @return Standard deviation of the cartesian measurement.
  const double GetStdCartesianDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return cartesian_std_dev_;
  }

  /// @brief Returns the standard deviation of the range measurement.
  /// @return Standard deviation of the range measurement.
  const double GetStdRangeDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return range_std_dev_;
  }

  /// @brief Returns the standard deviation of the azimuth measurement.
  /// @return standard deviation of the azimuth measurement.
  const double GetStdAzimuthDeviation() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return azimuth_std_dev_;
  }

  // Setters

  /// @brief Sets the update rate of the simulation. This is the rate at which
  /// the simulation will run.
  /// @param rate_ns Update rate in nanoseconds.
  void SetUpdateRate(double rate_ns) {
    std::unique_lock<std::mutex> lock(mtx_);
    update_rate_ = rate_ns;
  }

  /// @brief Sets the position of the sensor. This will be taken into account
  /// for range and azimuth but not cartesian coordinates.
  /// @param pos Position of the sensor in 2D.
  void SetSensorPosition(const SensVec2D& pos) {
    std::unique_lock<std::mutex> lock(mtx_);
    radar_position_ = pos;
  }

  /// @brief Sets the standard deviation of the cartesian measurement.
  /// @param std_dev Standard deviation of the cartesian measurement.
  void SetStdCartesianDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    cartesian_std_dev_ = std_dev;
  }

  /// @brief Sets the standard deviation of the range measurement.
  /// @param std_dev Standard deviation of the range measurement.
  void SetStdRangeDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    range_std_dev_ = std_dev;
  }

  /// @brief Sets the standard deviation of the azimuth measurement.
  /// @param std_dev Standard deviation of the azimuth measurement.
  virtual void SetStdAzimuthDeviation(double std_dev) {
    std::unique_lock<std::mutex> lock(mtx_);
    azimuth_std_dev_ = std_dev;
  }

 protected:
  virtual void RunImpl();

 private:
  /// TODO: Implement different update rate of model and sensor data
  // config
  double update_rate_;

  // Flags
  bool start_ = false;       // Flag to indicate if the simulation is running
  bool has_update_ = false;  // Flag to indicate if there is a new update
  bool should_stop_ = false;
  bool rec_update_ =
      false;  // Flag to indicate if the simulation received data during update
  unsigned long long curr_index_ = 0;  // Current index of the trajectory

  // Thread control variables
  std::jthread sim_thread_;
  std::condition_variable cv_start_;
  mutable std::mutex mtx_;

  // Simulation data
  std::vector<Trajectory<ObjectState2D>> trajectories_;
  std::vector<unsigned long long> traj_index_offset_;
  std::vector<ObjectPosition2D> cart_positions_, true_pos;
  SensVec2D radar_position_;
  std::vector<ObjectState2D> rang_azimuth_states_;

  RadarSimState curr_state_;

  // Sensor deviation
  double cartesian_std_dev_ =
      0.0;  // Standard deviation of the cartesian coordinates
  double range_std_dev_ = 0.0;  // Standard deviation of the range measurement
  double azimuth_std_dev_ =
      0.0;  // Standard deviation of the azimuth measurement
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_RADAR_SIM_H