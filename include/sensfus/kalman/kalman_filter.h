#ifndef SENSFUS_KALMAN_KALMAN_FILTER_H
#define SENSFUS_KALMAN_KALMAN_FILTER_H

#include <concepts>
#include <ctime>

#include <Eigen/Dense>

#include "sensfus/types.h"
#include "sensfus/kalman/evolution_model.h"

namespace sensfus {

namespace kalman {

using TimeStamp = size_t;
using TimeStep = long long;

// Define a concept for KalmanStateType so that we can ensure that only
// ObjectState2D or ObjectState3D can be used as StateType in KalmanFilterBase
// and KalmanFilter
template <typename T>
concept KalmanStateType =
    std::same_as<T, ObjectState2D> || std::same_as<T, ObjectState3D>;

template <KalmanStateType StateType>
struct KalmanState {
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;

  /// TODO: static_Assert that StateType is of type Eigen::MatrixXD
  Eigen::Matrix<ScalarType, kDim * 3, 1>
      x;  // State vector: position, velocity, acceleration
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> P;  // Covariance matrix

  TimeStamp k_timestamp;
};

template <KalmanStateType StateType>
class KalmanFilterBase {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

 protected:
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;
  using UpdateType = std::conditional_t<std::same_as<StateType, ObjectState2D>,
                                        ObjectPosition2D, ObjectPosition3D>;

 public:
  explicit KalmanFilterBase() = default;
  virtual ~KalmanFilterBase() = default;

  virtual const KalmanState<StateType> Predict(const TimeStamp& time) = 0;
  virtual const KalmanState<StateType> Update(const UpdateType& update,
                                              const TimeStamp& time) = 0;
  virtual const std::vector<KalmanState<StateType>> UpdateWithSmooth(
      const UpdateType& update, const TimeStamp& time) = 0;
  virtual const std::vector<KalmanState<StateType>> Retrodict() = 0;

  virtual const EvolutionModel<kDim>& GetEvolutionModel() const = 0;
};

template <KalmanStateType StateType, bool UseSimulatedTime = false>
class KalmanFilter : public KalmanFilterBase<StateType> {
  using UpdateType = std::conditional_t<std::same_as<StateType, ObjectState2D>,
                                        ObjectPosition2D, ObjectPosition3D>;
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;

 public:
  explicit KalmanFilter();
  virtual ~KalmanFilter() = default;

  virtual const KalmanState<StateType> Predict(const TimeStamp& time) override;

  virtual const KalmanState<StateType> Update(const UpdateType& update,
                                              const TimeStamp& time) override {
    return KalmanState<StateType>();
  }
  virtual const std::vector<KalmanState<StateType>> UpdateWithSmooth(
      const UpdateType& update, const TimeStamp& time) override {
    return std::vector<KalmanState<StateType>>();
  }

  virtual const std::vector<KalmanState<StateType>> Retrodict() override {
    return std::vector<KalmanState<StateType>>();
  }

  virtual const EvolutionModel<kDim>& GetEvolutionModel() const override {
    return evolution_model_;
  }

  void SetTimeBetweenSimulatedPoints(double time_between_simulated_points_s) {
    time_between_simulated_points_s_ = time_between_simulated_points_s;

    // Update the evolution model with the new time step
    evolution_model_.SetDeltaTime(time_between_simulated_points_s_);
  }

 protected:
  // not the best design but for simplicity, the derived classes will have
  // access to the KalmanStates

  std::vector<KalmanState<StateType>> states_;
  std::vector<UpdateType> updates_;  // Store the updates for eventual smoothing

  KalmanState<StateType> xk_;  // Current state

  EvolutionModel<kDim> evolution_model_;

  // Metadata
  double time_between_simulated_points_s_ =
      0.05;  // Default time step in seconds
};

template <KalmanStateType StateType>
class KalmanFilterWithEventBus : public KalmanFilter<StateType> {
 public:
  KalmanFilterWithEventBus() = default;
  virtual ~KalmanFilterWithEventBus() = default;
};

template <KalmanStateType StateType>
class GUIKalmanFilter : public KalmanFilterBase<StateType> {
 public:
 private:
};

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_H

#include "sensfus/kalman/kalman_filter.ipp"