#include "sensfus/sim/radar_sim.h"

#include <stop_token>
#include <iostream>
#include <cmath>

#include "sensfus/utils/timers.h"
#include "sensfus/utils/rand_generator.h"

namespace sensfus {
namespace sim {

RadarSim::RadarSim() {
  // (I, 0, 0)
  H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
}

void RadarSim::Init() {
  // Init all the simulation data to zero
  trajectories_.clear();
  cart_positions_.clear();
  radar_position_ = SensVec2D{0.0, 0.0};
  rang_azimuth_states_.clear();

  sim_thread_ = std::jthread([this](std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_start_.wait(lock, [this] { return start_; });

      while (!should_stop_) {
        utils::RateTimer timer(
            std::chrono::nanoseconds(static_cast<long long>(update_rate_)));
        RunImpl();
        lock.unlock();
        timer.WaitRemaining();  // Wait for the next update
        lock.lock();
      }
    }
  });
}

void RadarSim::StartSimulation() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = true;
  cv_start_.notify_all();
  std::cout << "Starting Simulation" << std::endl;
}

void RadarSim::HaltSimulation() {
  /// TODO: Imeplement this
}

void RadarSim::Stop() {
  {
    std::unique_lock<std::mutex> lock(mtx_);
    should_stop_ = true;
    start_ = true;  // Ensure the thread wakes up if waiting
    cv_start_.notify_all();
  }
  if (sim_thread_.joinable()) {
    sim_thread_.request_stop();
    sim_thread_.join();

    start_ = false;  // Reset the start flag
  }
}

RadarSimState RadarSim::GetState() {
  std::unique_lock<std::mutex> lock(mtx_);
  has_update_ = false;  // Reset the update flag
  return curr_state_;
}

bool RadarSim::HasUpdate() const {
  std::unique_lock<std::mutex> lock(mtx_);
  return has_update_;
}

void RadarSim::RunImpl() {
  // update the current state
  curr_state_.truth.clear();
  curr_state_.sensor.Clear();

  // Update the simulation data for all the trajectories
  for (size_t i = 0; i < trajectories_.size(); i++) {
    if (trajectories_.at(i).GetSize() >=
        curr_index_ - traj_index_offset_.at(i)) {
      // For later use it is good to extract the target position here
      ObjectState2D target_pos =
          trajectories_.at(i).GetState(curr_index_ - traj_index_offset_.at(i));

      // Update the true trajectory position -> account for the offset that the
      // trajectory was added to a running simulation
      true_pos.push_back(target_pos.head<2>());
      curr_state_.truth.push_back(std::make_pair(i, target_pos));

      // Update the cartesian position: zck = H*xk + sigma*rand
      ObjectPosition2D temp =
          (H_ * target_pos +
           cartesian_std_dev_ *
               utils::StdNormalGenerator<ScalarType, 2>().sample())
              .eval();

      cart_positions_.push_back(temp);
      // Also update the current state
      curr_state_.sensor.zx.push_back(temp(0));
      curr_state_.sensor.zy.push_back(temp(1));

      // update the rang and azimuth states
      // zpk = ((sqrt((xk - xs)^2 + (yk - ys)^2), arctan(yk - ys/xk - xs)) +
      // noise
      ObjectPosition2D temp2;

      temp2 << sqrt((target_pos(0) - radar_position_.x) *
                        (target_pos(0) - radar_position_.x) +
                    (target_pos(1) - radar_position_.y) *
                        (target_pos(1) - radar_position_.y)),
          std::atan2((target_pos(1) - radar_position_.y),
                     (target_pos(0) - radar_position_.x));

      temp2 += Eigen::Matrix<ScalarType, 2, 1>(
          range_std_dev_ *
              utils::StdNormalGenerator<ScalarType, 1>().sample_raw(),
          azimuth_std_dev_ *
              utils::StdNormalGenerator<ScalarType, 1>().sample_raw());

      rang_azimuth_states_.push_back(temp2);
      curr_state_.sensor.range.push_back(temp2(0));
      curr_state_.sensor.azimuth.push_back(temp2(1));

      has_update_ = true;  // Set the update flag to true
    }
  }

  // Also update random noise or static objects

  curr_index_++;
}

}  // namespace sim

}  // namespace sensfus
