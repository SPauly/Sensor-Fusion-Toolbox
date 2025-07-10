#ifndef SENSFUS_KALMAN_KALMAN_FILTER_H
#define SENSFUS_KALMAN_KALMAN_FILTER_H

#include <concepts>
#include <ctime>
#include <thread>
#include <memory>
#include <mutex>

#include <Eigen/Dense>

#include "sensfus/types.h"
#include "sensfus/utils/eventbus.h"
#include "sensfus/utils/timers.h"
#include "sensfus/kalman/evolution_model.h"

namespace sensfus {

namespace kalman {

using TimeStamp = TimeStepIdType;
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
  TimeStamp k_timestamp = 0;  // Timestamp of the state

  Eigen::Matrix<ScalarType, kDim * 3, 1>
      x;  // State vector: position, velocity, acceleration
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> P;  // Covariance matrix

  // Store data for retrodiction
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3> F;  // State transition matrix
  Eigen::Matrix<ScalarType, kDim * 3, kDim * 3>
      D;  // Evolution covariance matrix (process noise covariance)
};

template <KalmanStateType StateType>
struct KalmanStateMetadata {
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;
  TimeStamp k_timestamp = 0;  // Timestamp of the state
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
 protected:
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

  virtual const std::vector<KalmanState<StateType>> Retrodict() override;

  virtual const EvolutionModel<kDim>& GetEvolutionModel() const override {
    return evolution_model_;
  }

  /// @brief Set the update rate in seconds. This will update the evolution
  /// model with the new time step. If UseSimulatedTime is true, the
  /// update_rate_s_ will be interpreted as the time between two simulated
  /// steps.
  /// @param update_rate_s The update rate in seconds. (or time between steps in
  /// simulated time)
  virtual void SetUpdateRate(double update_rate_s) {
    update_rate_s_ = update_rate_s;

    // Update the evolution model with the new time step
    evolution_model_.SetDeltaTime(update_rate_s_);
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
  double update_rate_s_ = 0.01;  // Default time step in seconds

  size_t retrodict_steps_ = 10;  // Number of steps to retrodict
  size_t steps_to_prediction_ =
      10;  // Number of steps to predict in the future (for retrodiction)

  // Helpers:
  Eigen::Matrix<ScalarType, kDim, kDim * 3> H_;  // Measurement matrix
  Eigen::Matrix<ScalarType, kDim, kDim>
      R_;  // Measurement noise covariance matrix (assumed constant)
};

template <KalmanStateType StateType, bool UseSimulatedTime = false>
class KalmanFilterWithEventBus
    : public KalmanFilter<StateType, UseSimulatedTime> {
 protected:
  using typename KalmanFilter<StateType, UseSimulatedTime>::UpdateType;
  static constexpr int kDim = std::same_as<StateType, ObjectState2D> ? 2 : 3;
  using SensorInfoType = std::conditional_t<
      std::same_as<StateType, ObjectState2D>, RadarSensorInfo2D,
      RadarSensorInfo2D>;  // Type of sensor info used for updates

 public:
  explicit KalmanFilterWithEventBus(
      std::shared_ptr<::sensfus::utils::EventBus> event_bus)
      : KalmanFilter<StateType, UseSimulatedTime>(),
        event_bus_(std::move(event_bus)) {
    // Initialize the publishers for state and update
    state_publisher_ =
        event_bus_->AddChannel<KalmanState<StateType>>("KalmanState");
    update_publisher_ = event_bus_->AddChannel<KalmanStateMetadata<StateType>>(
        "KalmanStateMetadata");
    // Subscribe to the sensor info channel
    sensor_info_sub_ =
        event_bus_->Subscribe<SensorInfoType>("RadarSensorInfo2D");

    // Subscribe to the simulated time channel
    simulated_time_sub_ = event_bus_->Subscribe<TimeStamp>("SimulatedTime");

    // Update the rate of the kalman filter
    SetUpdateRate(5000000000.0 * 1e-9);

    // Start the Kalman filter loop
    loop_thread_ =
        std::thread(&KalmanFilterWithEventBus::RunKalmanFilterLoop, this);
  }
  virtual ~KalmanFilterWithEventBus() {
    stop_loop_ = true;
    // Stop the Kalman filter loop
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
  }

  virtual const KalmanState<StateType> Predict(const TimeStamp& time) override {
    std::unique_lock<std::mutex> lock(mtx_);
    // Predict the next state
    KalmanState<StateType> predicted_state =
        KalmanFilter<StateType, UseSimulatedTime>::Predict(time);

    // Publish the predicted state
    if (state_publisher_) {
      state_publisher_->Publish(predicted_state);
    }

    return predicted_state;
  }

  virtual const KalmanState<StateType> Update(const UpdateType& update,
                                              const TimeStamp& time) override {
    std::unique_lock<std::mutex> lock(mtx_);

    // Update the state with the new measurement
    KalmanState<StateType> updated_state =
        KalmanFilter<StateType, UseSimulatedTime>::Update(update, time);

    // Publish the updated state
    if (update_publisher_) {
      update_publisher_->Publish(this->xk_update_);
    }

    return updated_state;
  }

  virtual const std::vector<KalmanState<StateType>> UpdateWithSmooth(
      const UpdateType& update, const TimeStamp& time) override {
    return std::vector<KalmanState<StateType>>();
  }

  virtual const std::vector<KalmanState<StateType>> Retrodict() override {
    return std::vector<KalmanState<StateType>>();
  }

  virtual const EvolutionModel<kDim>& GetEvolutionModel() const override {
    std::unique_lock<std::mutex> lock(mtx_);
    return KalmanFilter<StateType, UseSimulatedTime>::GetEvolutionModel();
  }

  virtual void SetUpdateRate(double update_rate_s) final {
    std::unique_lock<std::mutex> lock(mtx_);
    KalmanFilter<StateType, UseSimulatedTime>::SetUpdateRate(update_rate_s);
  }

  /// @brief Set the update interval in steps. This will be used to determine
  /// when the next update should be performed in the Kalman filter loop in
  /// regard to the predicted time step.
  /// @param update_in_steps The number of steps till the next update.
  virtual void SetUpdateInterval(const TimeStep& update_in_steps) final {
    std::unique_lock<std::mutex> lock(mtx_);
    update_in_steps_ = update_in_steps;
    predictions_left_ = update_in_steps_;
  }

 protected:
  virtual void RunKalmanFilterLoop();

 private:
  mutable std::mutex mtx_;
  std::thread loop_thread_;
  std::condition_variable indicate_sim_step_;
  bool stop_loop_ = false;

  std::shared_ptr<::sensfus::utils::EventBus> event_bus_;

  std::shared_ptr<::sensfus::utils::Publisher<KalmanState<StateType>>>
      state_publisher_;
  std::shared_ptr<::sensfus::utils::Publisher<KalmanStateMetadata<StateType>>>
      update_publisher_;

  // We get the sensor info over the event bus
  std::shared_ptr<
      typename ::sensfus::utils::Channel<SensorInfoType>::Subscription>
      sensor_info_sub_;
  std::shared_ptr<::sensfus::utils::Channel<TimeStamp>::Subscription>
      simulated_time_sub_;

  // Store current situation
  TimeStamp update_in_steps_ = 4;  // Number of steps till next update
  TimeStamp predictions_left_ =
      4;  // Number of predictions left before next update
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