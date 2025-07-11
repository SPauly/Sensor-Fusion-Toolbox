#ifndef SENSFUS_SIM_RADAR_SIM_H
#define SENSFUS_SIM_RADAR_SIM_H

#include <vector>
#include <thread>
#include <mutex>
#include <utility>  // std::pair

#include "sensfus/types.h"
#include "sensfus/utils/eventbus.h"
#include "sensfus/sim/internal/sim_base.h"
#include "sensfus/sim/sensor_radar.h"
#include "sensfus/sim/object_model.h"
#include "sensfus/sim/trajectory.h"
#include "sensfus/sim/internal/trajectory_impl.h"

namespace sensfus {
namespace sim {

class SensorSimulator : public internal::SimBase {
 public:
  explicit SensorSimulator();
  virtual ~SensorSimulator() noexcept;

  virtual void StartSimulation() override;
  virtual void HaltSimulation() override;
  virtual void ResetSimulation() override;

  /// @brief Pushes a trajectory to the simulation. This trajectory will be used
  /// to simulate the sensor data.
  /// @param traj Trajectory will be started with the next simulation step.
  [[nodiscard]] std::shared_ptr<Trajectory<ObjectState2D>>
  CreateTrajectoryFromVec2D(
      const std::vector<Vector2D>& line_vector,
      const ObjectModelType type = ObjectModelType::BasicVelocityModel);

  /// @brief Adds another sensor to the simulation but does not start it.
  /// @return New Sensor that was added
  /// TODO: make a version that can set the size etc. immediately
  virtual std::shared_ptr<SensorRadar> AddRadarSensor() {
    std::unique_lock<std::mutex> lock(mtx_);
    radar_sensors_.push_back(
        std::make_shared<SensorRadar>(radar_sensors_.size(), event_bus_));
    return radar_sensors_.back();
  }

  // Getters

  inline const std::shared_ptr<utils::EventBus> GetEventBus() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return event_bus_;
  }

  /// @brief Returns true if the simulation is running.
  /// @return true if the simulation is running, false otherwise.
  inline const bool IsRunning() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return start_;
  }

  /// @brief Returns the current Simulation step
  /// @return Step of the simulation.
  inline const TimeStepIdType GetStepIndex() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return curr_index_;
  }

  /// @brief Returns the number of different trajectories in the simulation.
  /// @return
  inline const size_t GetTrajectoryCount() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return trajectories_.size();
  }

  inline const std::vector<std::shared_ptr<SensorRadar>> GetRadarSensors()
      const {
    std::unique_lock<std::mutex> lock(mtx_);
    return radar_sensors_;
  }

  // Setters

  /// @brief Sets the update rate of the simulation. This is the rate at which
  /// the simulation will run.
  /// @param rate_ns Update rate in nanoseconds.
  void SetUpdateRate(TimeStepIdType rate_ns);

 protected:
  virtual void RunImpl();

 private:
  TimeStepIdType update_rate_, glob_sensor_update_rate_;

  // Flags
  bool start_ = false;  // Flag to indicate if the simulation is running
  bool should_stop_ = false;
  bool rec_update_ =
      false;  // Flag to indicate if the simulation received data during update
  TimeStepIdType curr_index_ = 0;  // Current index of the trajectory

  // Thread control variables
  std::jthread sim_thread_;
  std::condition_variable cv_start_;
  mutable std::mutex mtx_;

  // Simulation data
  std::vector<std::shared_ptr<internal::TrajectoryImpl<ObjectState2D>>>
      trajectories_;
  std::vector<std::shared_ptr<internal::TrajectoryImpl<ObjectState3D>>>
      trajectories_3d_;
  std::vector<TimeStepIdType> traj_index_offset_, traj_index_offset_3d_;

  std::vector<TrueTargetState2D> true_states_;  // True target states

  // Store the sensors separately (so not as their base) to handle different
  // behaviors and for ease of use.
  std::vector<std::shared_ptr<SensorRadar>> radar_sensors_;

  // Event bus for communication
  std::shared_ptr<utils::EventBus> event_bus_;
  std::shared_ptr<utils::Publisher<TrueTargetState2D>> target_pub_;

  // We also need to publish simulated time
  std::shared_ptr<utils::Publisher<TimeStepIdType>> simulated_time_pub_;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_RADAR_SIM_H