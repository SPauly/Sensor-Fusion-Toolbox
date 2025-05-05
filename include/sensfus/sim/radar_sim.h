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
  virtual void ChangeUpdateRate(double rate_in_ns) override;

  virtual RadarSimState GetState();
  virtual bool IsRunning() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return start_;
  }
  virtual bool HasUpdate() const;

  virtual void PushTrajectory(const Trajectory<ObjectState2D>& traj) {
    std::unique_lock<std::mutex> lock(mtx_);
    trajectories_.push_back(traj);

    // store the index offset of the trajectory
    traj_index_offset_.push_back(curr_index_);
  }

  virtual void SetUpdateRate(double rate) {
    std::unique_lock<std::mutex> lock(mtx_);
    update_rate_ = rate;
  }
  virtual void SetSensorPosition(const ObjectPosition2D& pos) {
    std::unique_lock<std::mutex> lock(mtx_);
    radar_position_ = pos;
  }

  /// @brief Returns the current Simulation step
  /// @return Step of the simulation.
  virtual unsigned long long GetStepIndex() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return curr_index_;
  }

  virtual const size_t GetTrajectoryCount() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return trajectories_.size();
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
  ObjectPosition2D radar_position_;
  std::vector<ObjectState2D> rang_azimuth_states_;

  RadarSimState curr_state_;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_RADAR_SIM_H