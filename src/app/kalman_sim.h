#ifndef SENSFUS_APP_KALMAN_SIM_H
#define SENSFUS_APP_KALMAN_SIM_H

#include <functional>
#include <vector>
#include <memory>

#include "utils/layer.h"
#include "sensfus/types.h"
#include "sensfus/utils/eventbus.h"
#include "sensfus/kalman/kalman_filter.h"
#include "sensfus/sim/sensor_simulator.h"

namespace sensfus {
namespace app {
class KalmanSim : public app::utils::Layer {
 public:
  explicit KalmanSim(size_t id, std::shared_ptr<sim::SensorSimulator> sim);
  virtual ~KalmanSim() = default;

  /// @brief Interface that provides the implot output of the sensor
  void RunPlotCallback();

  // Layer interface
  virtual void OnAttach() override {}
  virtual void OnDetach() override {}
  virtual void OnUIRender() override;

  std::shared_ptr<kalman::KalmanFilterWithEventBus<ObjectState2D, true>>
  GetKalmanFilter() const {
    return std::static_pointer_cast<
        kalman::KalmanFilterWithEventBus<ObjectState2D, true>>(kalman_);
  }

 private:
  // ID of the Filter
  size_t id_ = 0;

  // one kalman sim per sensor
  std::shared_ptr<kalman::KalmanFilterBase<ObjectState2D, true>> kalman_;
  std::shared_ptr<sim::SensorSimulator> sim_;

  // Event bus for the kalman filter
  std::shared_ptr<::sensfus::utils::EventBus> event_bus_;
  std::shared_ptr<::sensfus::utils::Channel<
      kalman::KalmanState<ObjectState2D>>::Subscription>
      state_sub_;
  std::shared_ptr<::sensfus::utils::Channel<
      kalman::KalmanStateMetadata<ObjectState2D>>::Subscription>
      metadata_sub_;

  // Store Kalman state for display
  std::vector<double> x_predicted_, y_predicted_, x_updated_, y_updated_;

  // Store some current metadata
  kalman::KalmanStateMetadata<ObjectState2D> latest_update_;
  kalman::KalmanState<ObjectState2D> latest_prediction_;
};

}  // namespace app

}  // namespace sensfus

#endif  // SENSFUS_APP_KALMAN_SIM_H