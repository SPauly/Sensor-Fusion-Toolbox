#ifndef SENSFUS_KALMAN_KALMAN_FILTER_IPP
#define SENSFUS_KALMAN_KALMAN_FILTER_IPP

#include "sensfus/kalman/kalman_filter.h"

namespace sensfus {
namespace kalman {

template <typename StateType>
KalmanFilter<StateType>::KalmanFilter() : evolution_model_(0.05, 0.01, false) {
  // Initialize the Kalman filter with default parameters
  states_.clear();
  updates_.clear();
  xk_ = KalmanState<kDim>();

  // Initialize the current state to 0 and add a large covariance
  xk_.x.setZero();
  xk_.P.setIdentity();

  xk_.P *= 1000.0;  // Large initial uncertainty
}

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_IPP