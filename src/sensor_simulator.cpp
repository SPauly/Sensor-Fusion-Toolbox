#include "sensfus/sim/sensor_simulator.h"

#include <stop_token>
#include <iostream>
#include <cmath>

#include "sensfus/utils/timers.h"
#include "sensfus/utils/rand_generator.h"

namespace sensfus {
namespace sim {

SensorSimulator::SensorSimulator() {
  event_bus_ = std::make_shared<utils::EventBus>();
  radar_sensors_ =
      std::make_shared<std::vector<std::shared_ptr<SensorRadar>>>();

  // Get the event bus and publisher
  target_pub_ = event_bus_->AddChannel<TrueTargetState2D>("TrueTargetState2D");

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

SensorSimulator::~SensorSimulator() {
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

  // stop the radar sensors
  radar_sensors_->clear();
}

void SensorSimulator::StartSimulation() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = true;
  should_stop_ = false;  // Reset the stop flag
  cv_start_.notify_all();
}

void SensorSimulator::HaltSimulation() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = false;
  should_stop_ = true;  // Stop the simulation thread
}

void SensorSimulator::ResetSimulation() {
  HaltSimulation();

  // Implement this later
}

void SensorSimulator::RunImpl() {
  // Retrieve the current state of all the simulated objects
  true_states_.push_back(TrueTargetState2D());
  true_states_.back().id = curr_index_;

  for (size_t i = 0; i < trajectories_.size(); i++) {
    // Determine the relative index for that trajectory
    auto curr = curr_index_ - traj_index_offset_.at(i);

    // Check if the trajectory is valid and has enough data
    if (trajectories_.at(i).GetSize() > curr) {
      // For later use it is good to extract the target position here -> account
      // for the offset that the trajectory was added to a running simulation
      ObjectState2D target_pos = trajectories_.at(i).GetState(curr);

      true_states_.back().states.push_back(
          std::make_pair(static_cast<TargetIdType>(i), target_pos));

      // Provide metadata if there is any
      true_states_.back().tangentials.push_back(
          std::make_pair(static_cast<TargetIdType>(i),
                         trajectories_.at(i).GetTangentialAt(curr)));
      true_states_.back().normvecs.push_back(
          std::make_pair(static_cast<TargetIdType>(i),
                         trajectories_.at(i).GetNormVecAt(curr)));
    }
  }

  // Publish the current state to the event bus
  target_pub_->Publish(true_states_.back());

  // Get the updates from the sensors here to safe them
  /// TODO: Implement this

  curr_index_++;
}

}  // namespace sim

}  // namespace sensfus
