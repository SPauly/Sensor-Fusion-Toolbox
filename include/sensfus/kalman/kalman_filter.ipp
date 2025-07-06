#ifndef SENSFUS_KALMAN_KALMAN_FILTER_IPP
#define SENSFUS_KALMAN_KALMAN_FILTER_IPP

#include "sensfus/kalman/kalman_filter.h"
#include "sensfus/utils/time.h"

namespace sensfus {
namespace kalman {

template <KalmanStateType StateType, bool UseSimulatedTime>
KalmanFilter<StateType, UseSimulatedTime>::KalmanFilter()
    : evolution_model_(0.05, 10, false) {
  // Set H to the identity matrix for the state vector
  H_.setZero();
  H_.block<2, 2>(0, 0).setIdentity();  // Only position part is measured

  // Set R to a small constant matrix (assumed measurement noise covariance)
  R_.setIdentity();
  // R_ *= 0.03;  // Small measurement noise

  // Initialize the Kalman filter with default parameters
  states_.clear();
  updates_.clear();
  xk_ = KalmanState<StateType>();

  // Initialize the current state to 0 and add a large covariance
  xk_.x.setZero();
  xk_.P.setIdentity();

  xk_.P.block<2, 2>(0, 0) *= 1000;    // Large uncertainty in position
  xk_.P.block<2, 2>(2, 2) *= 100000;  // Small uncertainty in velocity
  xk_.P.block<2, 2>(4, 4) *= 100000;  // Small uncertainty in acceleration

  if constexpr (!UseSimulatedTime) {
    // If not using simulated time, set the timestamp to the current time
    xk_.k_timestamp = utils::Time::now().toNanoseconds();
  } else {
    // If using simulated time, set the timestamp to 0
    xk_.k_timestamp = 0;
  }

  states_.push_back(xk_);

  xk_update_.xk = xk_;
  xk_update_.k_timestamp = xk_.k_timestamp;
  xk_update_.innovation.setZero();
  xk_update_.inv_covariance.setZero();
  xk_update_.kalman_gain.setZero();
  updated_states_.clear();
}

template <KalmanStateType StateType, bool UseSimulatedTime>
const KalmanState<StateType> KalmanFilter<StateType, UseSimulatedTime>::Predict(
    const TimeStamp& time) {
  KalmanState<StateType> prev = xk_;
  KalmanState<StateType> curr;

  // Check if the time is valid or whether we want to rerun a previous
  // prediction
  if (time <= xk_.k_timestamp) {
    // We do not want to predict a state twice, so we return the current state
    if (time == xk_.k_timestamp) {
      return xk_;
    }

    // search for the last valid state before the given time (use binary search)
    auto lower = std::lower_bound(
        states_.begin(), states_.end(), time,
        [](const KalmanState<StateType>& state, const TimeStamp& t) {
          return state.k_timestamp < t;
        });

    if (lower != states_.end())
      prev = *lower;
    else
      prev = states_.front();  // return first state if no valid state found
  }

  // Predict the next state using the evolution model
  double delta = 0.0;

  if constexpr (UseSimulatedTime)
    delta = update_rate_s_ * (time - prev.k_timestamp);
  else
    delta = (time - prev.k_timestamp) * 1e-9;  // Convert to seconds

  evolution_model_.SetDeltaTime(delta);

  curr.x = evolution_model_.F() * prev.x;
  curr.P = evolution_model_.F() * prev.P * evolution_model_.F().transpose() +
           evolution_model_.D();

  curr.k_timestamp = time;

  // Also safe F and D for retrodiction and debugging
  curr.F = evolution_model_.F();
  curr.D = evolution_model_.D();

  // only update the current state if the time is valid
  if (time >= xk_.k_timestamp)
    return xk_ = curr;
  else
    return curr;  // return the predicted state without updating
}

template <KalmanStateType StateType, bool UseSimulatedTime>
const KalmanState<StateType> KalmanFilter<StateType, UseSimulatedTime>::Update(
    const UpdateType& _update, const TimeStamp& time) {
  UpdateType update = _update;
  // convert update from range_azimuth to cartesian
  update(0) = _update(0) * std::cos(_update(1));
  update(1) = _update(0) * std::sin(_update(1));

  xk_update_.xk = xk_;  // Start with the current state

  // Calculate the innovation
  xk_update_.innovation = update - H_ * xk_.x;  // zk - H*xk|xk-1

  // Calculate the innovation covariance
  xk_update_.inv_covariance =
      H_ * xk_.P * H_.transpose() + R_;  // S = H*P*H^T + R

  // Calculate the Kalman gain
  xk_update_.kalman_gain =
      xk_.P * H_.transpose() *
      xk_update_.inv_covariance.inverse();  // K = P * H^T * S^-1

  xk_update_.k_timestamp = time;

  // Update the state estimate
  xk_update_.xk.x += xk_update_.kalman_gain *
                     xk_update_.innovation;  // xk|xk-1 + K * innovation
  xk_update_.xk.P -=
      xk_update_.kalman_gain * xk_update_.inv_covariance *
      xk_update_.kalman_gain.transpose();  // Pk|xk-1 - K * S * K^T

  xk_update_.xk.k_timestamp = time;

  updated_states_.push_back(xk_update_);
  return xk_ =
             xk_update_.xk;  // Update the current state for the next prediction
}

template <KalmanStateType StateType, bool UseSimulatedTime>
void KalmanFilterWithEventBus<StateType,
                              UseSimulatedTime>::RunKalmanFilterLoop() {
  while (!stop_loop_) {
    std::unique_lock<std::mutex> lock(mtx_);

    TimeStamp time_now = 0;

    // Predict
    if constexpr (!UseSimulatedTime) {
      state_publisher_->Publish(
          KalmanFilter<StateType, UseSimulatedTime>::Predict(
              utils::Time::now().toNanoseconds()));
    } else {
      lock.unlock();  // Unlock the mutex to allow other threads to access
      auto time =
          simulated_time_sub_->WaitForData();  // This ensures we update only
                                               // with the simulated time
      lock.lock();  // Lock the mutex again to access shared resources
      if (time) {
        state_publisher_->Publish(
            KalmanFilter<StateType, UseSimulatedTime>::Predict(*time));
        time_now = *time;
      }
    }

    // Update
    if (predictions_left_-- == 0) {
      // Reset the predictions left counter
      predictions_left_ = update_in_steps_;
      // Update the Kalman filter with new sensor data
      auto update = sensor_info_sub_->FetchLatest();
      if (update) {
        if (update->range_azimuth.empty()) {
          // No valid update available, skip this iteration
          continue;
        }
        auto update_extracted = update->range_azimuth.back();

        if constexpr (UseSimulatedTime) {
          if (time_now != 0) {
            KalmanFilter<StateType, UseSimulatedTime>::Update(update_extracted,
                                                              time_now);
            // Publish the updated state
            update_publisher_->Publish(this->xk_update_);
          }
        } else {
          KalmanFilter<StateType, UseSimulatedTime>::Update(
              update_extracted, utils::Time::now().toNanoseconds());
          // Publish the updated state
          update_publisher_->Publish(this->xk_update_);
        }
      }
    }

    // Wait for the next iteration
    if constexpr (!UseSimulatedTime) {
      utils::RateTimer rate_timer(this->update_rate_s_);
      lock.unlock();  // Unlock the mutex to allow other threads to access
      rate_timer.WaitRemaining();  // Wait for the next iteration
    }
    // In simulated time we simply wait for the next tick
  }
}

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_IPP