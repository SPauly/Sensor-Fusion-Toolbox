#ifndef SENSFUS_KALMAN_KALMAN_FILTER_H
#define SENSFUS_KALMAN_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <ctime>

#include "sensfus/types.h"

namespace sensfus {

namespace kalman {

using TimeStamp = size_t;
using TimeStep = long long;

template <size_t kDim = 2>
struct KalmanState {
  /// TODO: static_Assert that StateType is of type Eigen::MatrixXD
  Eigen::Matrix<ScalarType, kDim * 3, 1> x;
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> P;

  TimeStamp k_timestamp;
};

template <typename StateType>
class KalmanFilterBase {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

  // Determine the dimension of the object state based on the type
  static constexpr int kDim =
      std::is_same<StateType, ObjectState2D>::value ? 2 : 3;

  using UpdateType =
      std::conditional_t<std::is_same<StateType, ObjectState2D>::value,
                         ObjectPosition2D, ObjectPosition3D>;

 public:
  explicit KalmanFilterBase() = default;
  virtual ~KalmanFilterBase() = default;

  virtual const KalmanState<kDim> Predict() = 0;
  virtual const KalmanState<kDim> Update(const UpdateType& update) = 0;
  virtual const std::vector<KalmanState<kDim>> UpdateWithSmooth() = 0;
  virtual const std::vector<KalmanState<kDim>> Retrodict() = 0;
};

template <typename StateType>
class KalmanFilter : public KalmanFilterBase<StateType> {
  using typename KalmanFilterBase<StateType>::UpdateType;
  static constexpr int kDim = KalmanFilterBase<StateType>::kDim;

 public:
  explicit KalmanFilter();
  virtual ~KalmanFilter() = default;

  const KalmanState<kDim> Predict() override;

  const KalmanState<kDim> Update(const UpdateType& update) override;

  const std::vector<KalmanState<kDim>> UpdateWithSmooth() override;

  const std::vector<KalmanState<kDim>> Retrodict() override;

 protected:
  // not the best design but for simplicity, the derived classes will have
  // access to the KalmanStates
  std::vector<KalmanState<kDim>> states_;
  std::vector<UpdateType> updates_;

  KalmanState<kDim> current_state_, previous_state_;
};

template <typename StateType>
class KalmanFilterWithEventBus : public KalmanFilter<StateType> {
 public:
  KalmanFilterWithEventBus() = default;
  virtual ~KalmanFilterWithEventBus() = default;

  // Override methods to integrate with the event bus
  const KalmanState<KalmanFilterBase<StateType>::kDim> Predict() override;

  const KalmanState<KalmanFilterBase<StateType>::kDim> Update(
      const typename KalmanFilterBase<StateType>::UpdateType& update) override;

  const std::vector<KalmanState<KalmanFilterBase<StateType>::kDim>>
  UpdateWithSmooth() override;

  const std::vector<KalmanState<KalmanFilterBase<StateType>::kDim>> Retrodict()
      override;
};

template <typename StateType>
class GUIKalmanFilter : public KalmanFilterBase<StateType> {
 public:
 private:
};

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_H