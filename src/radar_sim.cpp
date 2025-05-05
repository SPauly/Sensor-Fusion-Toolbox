#include "sensfus/sim/radar_sim.h"

#include <stop_token>
#include <iostream>

#include "sensfus/utils/timers.h"

namespace sensfus {
namespace sim {

void RadarSim::Init() {
  // Init all the simulation data to zero
  trajectories_.clear();
  cart_positions_.clear();
  radar_position_ = ObjectPosition2D::Zero();
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
  }
}

void RadarSim::ChangeUpdateRate(double rate_in_ns) {
  std::unique_lock<std::mutex> lock(mtx_);
  update_rate_ = rate_in_ns;
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
      // Update the true trajectory position -> account for the offset that the
      // trajectory was added to a running simulation
      true_pos.push_back(trajectories_.at(i)
                             .GetState(curr_index_ - traj_index_offset_.at(i))
                             .head<2>());

      // The trajectories position in the vector is its id
      curr_state_.truth.push_back(std::make_pair(
          i, trajectories_.at(i).GetState(curr_index_ -
                                          traj_index_offset_.at(i))));

      // Update the cartesian position
      cart_positions_.push_back(
          trajectories_.at(i)
              .GetState(curr_index_ - traj_index_offset_.at(i))
              .head<2>());
      // update the rang and azimuth states

      has_update_ = true;  // Set the update flag to true
    }
  }

  // Also update random noise or static objects

  curr_index_++;
}

}  // namespace sim

}  // namespace sensfus
