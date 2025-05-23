#include "sensfus/sim/object_model_wave.h"

#include <cmath>

#define M_PI 3.14159265358979323846

namespace sensfus {
namespace sim {

void WaveModel::ApplyToTrajectory(const double time_between_points_ns) {
  // First calculate the period of the wave
  const double period = 2 * M_PI / omega_;  // Period in seconds

  // Calculate the number of points in the trajectory based on the sampling rate
  // and Wave period
  const double sampling_rate =
      time_between_points_ns / 1e9;  // Convert ns to seconds
  const double num_points = period / sampling_rate;

  states_->resize(num_points);
  tangentials_.resize(num_points);
  normvecs_.resize(num_points);

  // Calculate the Wave trajectory
  for (size_t i = 0; i < num_points; ++i) {
    double current_time = i * sampling_rate;  // Current time in seconds

    ObjectState2D& state = (*states_)[i];

    // Calcuilate the position of the object
    ObjectPosition2D position(
        amplitude_ * std::sin(omega_ * current_time),         // x position
        amplitude_ * std::sin(2.0 * omega_ * current_time));  // y position

    // Calculate velocity (1. derivative of position)
    ObjectVelocity2D velocity(
        speed_ms_ * std::cos(omega_ * current_time) / 2.0,
        speed_ms_ * std::cos(2.0 * omega_ * current_time));  // v = dx/dt

    // Calculate acceleration (2. derivative of position)
    ObjectAcceleration2D acceleration(
        -acceleration_ms2_ * std::sin(omega_ * current_time) / 4.0,
        -acceleration_ms2_ * std::sin(2.0 * omega_ * current_time));

    // Set the state of the object: [pos_x, pos_y, vel_x, vel_y, acc_x, acc_y]
    state.segment<2>(0) = position;      // pos_x, pos_y
    state.segment<2>(2) = velocity;      // vel_x, vel_y
    state.segment<2>(4) = acceleration;  // acc_x, acc_y

    // Calculate metadata like tangential, normal vectors
    ObjectPosition2D tangential = (1.0 / velocity.norm()) * velocity;
    ObjectPosition2D normal =
        1 / velocity.norm() *
        (ObjectPosition2D(-velocity[1], velocity[0]));  // Rotate 90 degrees

    tangentials_.at(i) = tangential;
    normvecs_.at(i) = normal;
  }
}

}  // namespace sim

}  // namespace sensfus
