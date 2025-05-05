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
      std::cout << "Running impl now" << std::endl;

      while (!should_stop_) {
        utils::RateTimer timer(std::chrono::seconds(5));
        RunImpl();
        lock.unlock();
        timer.WaitRemaining();
        lock.lock();
      }
    }
  });
}

void RadarSim::StartSimulation() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = true;
  cv_start_.notify_one();
  std::cout << "Starting Simulation" << std::endl;
}

void RadarSim::HaltSimulation() {
  /// TODO: Imeplement this
}

void RadarSim::Stop() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = false;
  should_stop_ = true;
  cv_start_.notify_one();
  sim_thread_.request_stop();
  sim_thread_.join();
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
  std::cout << "in impp" << std::endl;

  // update the current state
  curr_state_.truth.clear();
  curr_state_.sensor.Clear();

  // Update the simulation data for all the trajectories
  for (auto& trajectory : trajectories_) {
    // Update the true trajectory position
    true_pos.push_back(trajectory.GetState(cart_positions_.size()).head<2>());

    // Update the cartesian position
    // update the rang and azimuth states

    // Create the current state data
    curr_state_.truth.push_back(trajectory.GetState(cart_positions_.size()));
  }

  has_update_ = true;
}

}  // namespace sim

}  // namespace sensfus
