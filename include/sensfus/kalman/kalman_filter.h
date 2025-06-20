#ifndef SENSFUS_KALMAN_KALMAN_FILTER_H
#define SENSFUS_KALMAN_KALMAN_FILTER_H

#include <Eigen/Dense>

#include "sensfus/types.h"

namespace sensfus {

namespace kalman {

using TimeStamp = size_t;
using TimeStep = long long;

template <size_t kDim = 2>
struct KalmanState {
  /// TODO: static_Assert that MatrixType is of type Eigen::MatrixXD
  Eigen::Matrix<ScalarType, kDim * 3, 1> x;
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> P;

  TimeStamp k_timestamp;
  TimeStep k;
};

template <typename MatrixType, size_t kDim = 2>
class KalmanFilter {
  /// TODO: static_assert that MatrixType provides the correct operators

 public:
  explicit KalmanFilter() = default;
  virtual ~KalmanFilter() = default;

 private:
};

template <typename MatrixType, size_t kDim = 2>
class GUIKalmanFilter : public KalmanFilter<MatrixType, kDim> {
 public:
 private:
};

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_H