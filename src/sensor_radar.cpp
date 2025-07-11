#include "sensfus/sim/sensor_radar.h"

#include <cmath>

#include "sensfus/utils/timers.h"
#include "sensfus/utils/rand_generator.h"

namespace sensfus {
namespace sim {

SensorRadar::SensorRadar(const SensorIdType& id,
                         std::shared_ptr<utils::EventBus> event_bus)
    : SensorBase<ObjectState2D>(id), event_bus_(event_bus) {
  // Initialize the event bus and publisher
  radar_pub_ = event_bus_->AddChannel<RadarSensorInfo2D>("RadarSensorInfo2D");
  target_sub_ = event_bus_->Subscribe<TrueTargetState2D>("TrueTargetState2D");

  // (I, 0, 0)
  H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  // Initiate the simulation thread
  sim_thread_ = std::thread([this]() {
    while (!should_stop_) {
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

SensorRadar::~SensorRadar() {
  if (sim_thread_.joinable()) {
    should_stop_ = true;
    start_ = true;  // Ensure the thread wakes up if waiting
    cv_start_.notify_all();
    sim_thread_.join();
  }
}

void SensorRadar::StartSensor() {
  std::unique_lock<std::mutex> lock(mtx_);
  start_ = true;
  cv_start_.notify_all();
}

void SensorRadar::HaltSensor() {
  std::unique_lock<std::mutex> lock(mtx_);
  should_stop_ = true;
  start_ = true;  // Ensure the thread wakes up if waiting
  cv_start_.notify_all();
}

void SensorRadar::ResetSensor() {
  // Implement the reset logic here
}

void SensorRadar::RunImpl() {
  auto update = target_sub_->FetchLatest();
  if (!update) return;

  // Update the current state
  RadarSensorInfo2D sensor_info;
  sensor_info.id = kId;
  sensor_info.step = update->id;  // Update the step to be the same as the data
  curr_index_ = update->id;

  // Update according to the position data

  for (const auto& state : update->states) {
    // Get the position of the target
    ObjectPosition2D target_pos = H_ * state.second;

    // Update the cartesian coordinates
    ObjectPosition2D temp =
        (target_pos + cartesian_std_dev_ *
                          utils::StdNormalGenerator<ScalarType, 2>().sample())
            .eval();

    sensor_info.cartesian.push_back(temp);

    // Update the range and azimuth
    ObjectPosition2D temp2;

    temp2 << sqrt(std::pow((target_pos(0) - radar_position_(0)), 2) +
                  std::pow((target_pos(1) - radar_position_(1)), 2)),
        std::atan2((target_pos(1) - radar_position_(1)),
                   (target_pos(0) - radar_position_(0)));

    temp2 +=
        Vector2D(range_std_dev_ *
                     utils::StdNormalGenerator<ScalarType, 1>().sample_raw(),
                 azimuth_std_dev_ *
                     utils::StdNormalGenerator<ScalarType, 1>().sample_raw());

    sensor_info.range_azimuth.push_back(temp2);
  }

  // Publish the sensor information
  radar_pub_->Publish(sensor_info);
}

}  // namespace sim

}  // namespace sensfus
