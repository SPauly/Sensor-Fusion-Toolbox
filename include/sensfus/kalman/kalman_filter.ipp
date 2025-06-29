#ifndef SENSFUS_KALMAN_KALMAN_FILTER_IPP
#define SENSFUS_KALMAN_KALMAN_FILTER_IPP

#include "sensfus/kalman/kalman_filter.h"
#include "sensfus/utils/time.h"

namespace sensfus {
namespace kalman {

template <KalmanStateType StateType, bool UseSimulatedTime>
KalmanFilter<StateType, UseSimulatedTime>::KalmanFilter()
    : evolution_model_(0.05, 0.01, false) {
  // Initialize the Kalman filter with default parameters
  states_.clear();
  updates_.clear();
  xk_ = KalmanState<kDim>();

  // Initialize the current state to 0 and add a large covariance
  xk_.x.setZero();
  xk_.P.setIdentity();

  xk_.P *= 1000.0;  // Large initial uncertainty

  if constexpr (!UseSimulatedTime) {
    // If not using simulated time, set the timestamp to the current time
    xk_.k_timestamp = utils::Time::now().toNanoseconds();
  } else {
    // If using simulated time, set the timestamp to 0
    xk_.k_timestamp = 0;
  }

  states_.push_back(xk_);
}

template <KalmanStateType StateType, bool UseSimulatedTime>
const KalmanState<kDim> KalmanFilter<StateType, UseSimulatedTime>::Predict(
    const TimeStamp& time) {
  KalmanState<kDim> prev = xk_;
  KalmanState<kDim> curr;

  // Check if the time is valid or whether we want to rerun a previous
  // prediction
  if (time < xk_.k_timestamp) {
    // search for the last valid state before the given time (use binary search)
    auto lower = std::lower_bound(
        states_.begin(), states_.end(), time,
        [](const KalmanState<kDim>& state, const TimeStamp& t) {
          return state.k_timestamp < t;
        });

    if (lower != states_.end())
      prev = *lower;
    else
      prev = states.front();  // return first state if no valid state found
  }

  // Predict the next state using the evolution model
  double delta = 0.0;

  if constexpr (UseSimulatedTime)
    delta =
        time_between_simulated_points_s_ * std::abs(time - prev.k_timestamp);
  else
    delta = (time - prev.k_timestamp) * 1e-9;  // Convert to seconds

  evolution_model_.SetDeltaTime(delta);

  curr.x = evolution_model_.F() * prev.x;
  curr.P = evolution_model_.F() * prev.P * evolution_model_.F().transpose() +
           evolution_model_.D();

  curr.k_timestamp = time;

  // only update the current state if the time is valid
  if (time >= xk_.k_timestamp)
    return xk_ = curr;
  else
    return curr;  // return the predicted state without updating
}

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_IPP