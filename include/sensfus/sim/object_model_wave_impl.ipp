#ifndef SENSFUS_SIM_OBJECT_MODEL_WAVE_IMPL_IPP
#define SENSFUS_SIM_OBJECT_MODEL_WAVE_IMPL_IPP

#include "sensfus/sim/object_model_wave.h"

#include <cmath>

namespace sensfus {
namespace sim {

template <typename StateType>
void WaveModel<StateType>::ApplyToTrajectory() {
  std::unique_lock<std::mutex> lock(mtx_);

  TimeStepIdType num_points = 0;
  double sampling_interval = 0.0;

  if (!use_fixed_params_) {
    // First calculate the period of the wave
    const double period = 2 * M_PI / omega_;  // Period in seconds

    // Calculate the number of points in the trajectory based on the sampling
    // interval and ensure the last point is at or just past the end of one
    // period
    sampling_interval = time_between_points_ns_ * 1e-9;  // ns to seconds
    num_points =
        static_cast<unsigned long long>(std::ceil(period / sampling_interval));
  } else {
    num_points = 500;
    sampling_interval = time_between_points_ns_ * 1e-9;  // ns to seconds
    const double total_duration = (num_points - 1) * sampling_interval;
    omega_ = 2.0 * M_PI /
             total_duration;  // Set omega so one period fits in total_duration
  }

  states_->resize(num_points);
  tangentials_.resize(num_points);
  normvecs_.resize(num_points);

  // Calculate the Wave trajectory
  for (size_t i = 0; i < num_points; ++i) {
    double current_time = i * sampling_interval;  // Current time in seconds

    StateType& state = (*states_)[i];

    // Calcuilate the position of the object
    Vector3D position(
        amplitude_ * std::sin(omega_ * current_time),        // x position
        amplitude_ * std::sin(2.0 * omega_ * current_time),  // y position
        height_m_);

    // Calculate velocity (1. derivative of position)
    Vector3D velocity(speed_ms_ * std::cos(omega_ * current_time) / 2.0,
                      speed_ms_ * std::cos(2.0 * omega_ * current_time),
                      0.0);  // v = dx/dt

    // Calculate acceleration (2. derivative of position)
    Vector3D acceleration(
        -acceleration_ms2_ * std::sin(omega_ * current_time) / 4.0,
        -acceleration_ms2_ * std::sin(2.0 * omega_ * current_time), 0.0);

    // Calculate metadata like tangential, normal vectors
    Vector3D tangential = (1.0 / velocity.norm()) * velocity;
    Vector3D normal =
        1 / velocity.norm() *
        (Vector3D(-velocity[1], velocity[0], 0.0));  // Rotate 90 degrees

    if constexpr (kDim == 2) {
      // We only need the first two components for 2D
      state[0] = position[0];      // pos_x
      state[1] = position[1];      // pos_y
      state[2] = velocity[0];      // vel_x
      state[3] = velocity[1];      // vel_y
      state[4] = acceleration[0];  // acc_x
      state[5] = acceleration[1];  // acc_y

      tangentials_.at(i)[0] = tangential[0];  // Store tangential x in 2D
      tangentials_.at(i)[1] = tangential[1];  // Store tangential y in 2D
      normvecs_.at(i)[0] = normal[0];         // Store normal x in 2D
      normvecs_.at(i)[1] = normal[1];         // Store normal y in 2D

    } else if constexpr (kDim == 3) {
      // For 3D, we need all three components
      state.segment<3>(0) = position;      // pos_x, pos_y, pos_z
      state.segment<3>(3) = velocity;      // vel_x, vel_y, vel_z
      state.segment<3>(6) = acceleration;  // acc_x, acc_y, acc_z

      tangentials_.at(i) = tangential;
      normvecs_.at(i) = normal;
    }
  }
}

}  // namespace sim

}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_WAVE_IMPL_IPP