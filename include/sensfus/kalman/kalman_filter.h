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

/// @brief Contains the state vector and covariance matrix for the current state
/// as perceived by the Kalman filter. This might be used for both prediction
/// and updates steps.
/// @tparam StateType Either ObjectState2D or ObjectState3D. -> The siz of the
/// state vector is determined by the StateType and stored in kDim.
template <KalmanStateType StateType>
struct KalmanState {
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;
  TimeStamp k_timestamp;

  Eigen::Matrix<ScalarType, kDim * 3, 1>
      x;  // State vector: position, velocity, acceleration
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> P;  // Covariance matrix

  // Store data for retrodiction
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> F;  // State transition matrix
};

template <KalmanStateType StateType>
struct KalmanStateMetadata {
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;
  TimeStamp k_timestamp;      // Timestamp of the state
  KalmanState<StateType> xk;  // State vector and covariance

  Eigen::Matrix<ScalarType, kDim, 1> innovation;  // Innovation vector
  Eigen::Matrix<ScalarType, kDim, kDim>
      inv_covariance;  // Innovation covariance

  Eigen::Matrix<ScalarType, kDim * 3, kDim> kalman_gain;  // Kalman gain
};

template <KalmanStateType StateType, bool UseSimulatedTime = false>
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
class KalmanFilter : public KalmanFilterBase<StateType, UseSimulatedTime> {
  using UpdateType = std::conditional_t<std::same_as<StateType, ObjectState2D>,
                                        ObjectPosition2D, ObjectPosition3D>;
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;

 public:
  explicit KalmanFilter();
  virtual ~KalmanFilter() = default;

  virtual const KalmanState<StateType> Predict(const TimeStamp& time) override;

  virtual const KalmanState<StateType> Update(const UpdateType& update,
                                              const TimeStamp& time) override;

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
  std::vector<KalmanStateMetadata<StateType>> updated_states_;
  std::vector<UpdateType> updates_;  // Store the updates for eventual smoothing

  KalmanState<StateType> xk_;  // Current state
  KalmanStateMetadata<StateType>
      xk_update_;  // Metadata for the current state -> this must not match xk
                   // since it only referes to the updated step

  /// TODO: think about how to handle the case of arriving updates -> Problem:
  /// Filter has higher update rate so the simulated time does not work. updates
  /// will arrive with deprecated timestamps

  EvolutionModel<kDim> evolution_model_;

  // Metadata
  double time_between_simulated_points_s_ =
      0.05;  // Default time step in seconds

  // Helpers:
  Eigen::Matrix<ScalarType, kDim, kDim * 3> H_;  // Measurement matrix
  Eigen::Matrix<ScalarType, kDim, kDim>
      R_;  // Measurement noise covariance matrix (assumed constant)
};

template <KalmanStateType StateType, bool UseSimulatedTime = false>
class KalmanFilterWithEventBus
    : public KalmanFilter<StateType, UseSimulatedTime> {
 public:
  KalmanFilterWithEventBus() = default;
  virtual ~KalmanFilterWithEventBus() = default;
};

template <KalmanStateType StateType, bool UseSimulatedTime = false>
class GUIKalmanFilter
    : public KalmanFilterWithEventBus<StateType, UseSimulatedTime> {
 public:
 private:
};

}  // namespace kalman

}  // namespace sensfus

#endif  // SENSFUS_KALMAN_KALMAN_FILTER_H

#include "sensfus/kalman/kalman_filter.ipp"