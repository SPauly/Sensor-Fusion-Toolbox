#ifndef SENSFUS_SIM_OBJECT_MODEL_WAVE_H
#define SENSFUS_SIM_OBJECT_MODEL_WAVE_H

#include "sensfus/sim/internal/object_model_base.h"

#include <cmath>
#include <mutex>

#include "sensfus/types.h"
#include "sensfus/utils/math.h"

namespace sensfus {
namespace sim {
// Forward declaration of the WaveModel class
template <typename StateType>
class WaveModel;

using WaveModel2D = WaveModel<ObjectState2D>;
using WaveModel3D = WaveModel<ObjectState3D>;

template <typename StateType>
class WaveModel : public ObjectModelBase<StateType> {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D; Use "
                "explicit types WaveModel2D or WaveModel3D for convenience.");

  using VecType = typename ObjectModelBase<StateType>::VecType;
  static constexpr int kDim =
      ObjectModelBase<StateType>::kDim;  // Dimension of the state

  // Type aliases for fast access to base class types
  using ObjectModelBase<StateType>::states_;
  using ObjectModelBase<StateType>::time_between_points_ns_;

 public:
  explicit WaveModel(std::shared_ptr<std::vector<StateType>> states)
      : ObjectModelBase<StateType>(states) {
    RecalculateParams();
  }
  ~WaveModel() override = default;

  /// @brief Populates the states with target positions, velocities,
  /// acceleration, tangentials and normalvectors based on the provided sampling
  /// rate
  virtual void ApplyToTrajectory() override;

  // Setters

  /// @brief Sets the speed of the object in m/s.
  /// @param speed_ms Speed in m/s.
  inline void SetSpeed(const double speed_ms) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (use_fixed_params_) {
      // If fixed parameters are used, we cannot change the speed
      return;
    }

    speed_ms_ = speed_ms;

    lock.unlock();
    RecalculateParams();
  }

  /// @brief Sets the acceleration of the object in m/s^2.
  /// @param acceleration_ms2 Acceleration in m/s^2.
  inline void SetAcceleration(const double acceleration_ms2) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (use_fixed_params_) {
      // If fixed parameters are used, we cannot change the acceleration
      return;
    }

    acceleration_ms2_ = acceleration_ms2;
    lock.unlock();
    RecalculateParams();
  }

  /// @brief Sets the time the wave takes to complete one period in seconds.
  /// @param period_s Period in seconds.
  inline void SetPeriod(double period_s) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (use_fixed_params_) {
      // If fixed parameters are used, we cannot change the period
      return;
    }

    period_s_ = period_s;
    omega_ = 2.0 * M_PI / period_s_;  // Recalculate omega_

    // Recalculate acceleration and amplitude based on the new period
    acceleration_ms2_ = 2 * M_PI * speed_ms_ / period_s_;
    amplitude_ = (speed_ms_ * speed_ms_) / acceleration_ms2_;  // A = v^2/q

    lock.unlock();
    ApplyToTrajectory();
  }

  inline void UseFixedParams(const bool use_fixed_params) {
    std::unique_lock<std::mutex> lock(mtx_);
    use_fixed_params_ = use_fixed_params;
    if (use_fixed_params_) {
      // If fixed parameters are used, we cannot change the speed or
      // acceleration
      speed_ms_ = 300.0;        // Default speed in m/s
      acceleration_ms2_ = 9.0;  // Default acceleration in m/s^2
      height_m_ = 1000.0;       // Default height of the wave in m

      lock.unlock();
      RecalculateParams();
    }
  }

 protected:
  /// @brief Get the tangential of the current position -> velocity direction
  /// @param timestamp Time at which the tangent will be returned -> will be
  /// chopped within the wave period
  /// @return Tangential of the velocity
  virtual VecType GetTangentialAtImpl(
      const TimeStepIdType timestamp) const override {
    std::unique_lock<std::mutex> lock(mtx_);
    return tangentials_.at(utils::fast_mod(timestamp, tangentials_.size()));
  }

  /// @brief Get the normal vector of the current position -> perpendicular to
  /// the velocity direction
  /// @param timestamp Time at which the normal vector will be returned ->
  /// will be chopped within the wave period
  /// @return Normal vector of the velocity
  virtual VecType GetNormVecAtImpl(
      const TimeStepIdType timestamp) const override {
    std::unique_lock<std::mutex> lock(mtx_);
    return normvecs_.at(utils::fast_mod(timestamp, normvecs_.size()));
  }

  inline void RecalculateParams() {
    std::unique_lock<std::mutex> lock(mtx_);

    omega_ = acceleration_ms2_ / 2 * speed_ms_;                // w = q/2 * v
    amplitude_ = (speed_ms_ * speed_ms_) / acceleration_ms2_;  // A = v^2/q

    lock.unlock();
    ApplyToTrajectory();  // Recalculate the trajectory based on new params
  }

 private:
  mutable std::mutex mtx_;
  std::vector<VecType> tangentials_, normvecs_;

  bool use_fixed_params_ = true;  // Use fixed parameters for the wave

  double speed_ms_ = 300.0;        // Speed in m/s
  double acceleration_ms2_ = 9.0;  // Acceleration in m/s^2
  double height_m_ = 1000.0;       // Height of the wave in m (for 3D waves)
  double period_s_ = 0.0;          // Period of the wave in seconds

  // Wave simulation parameters
  double omega_ = 0.0;      // w = acceleration/2 * speed
  double amplitude_ = 0.0;  // Amplitude of the wave in m
};

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

#endif  // SENSFUS_SIM_OBJECT_MODEL_WAVE_H