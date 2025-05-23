#ifndef SENSFUS_SIM_OBJECT_MODEL_WAFE_H
#define SENSFUS_SIM_OBJECT_MODEL_WAFE_H

#include "sensfus/internal/object_model_base.h"

#include "sensfus/types.h"

namespace sensfus {
namespace sim {

class WafeModel : public ObjectModelBase<ObjectState2D> {
 public:
  explicit WafeModel(std::shared_ptr<std::vector<ObjectState2D>> states)
      : ObjectModelBase<ObjectState2D>(states) {
    RecalculateParams();
  }
  ~WafeModel() override = default;

  /// @brief Populates the states with target positions, velocities,
  /// acceleration, tangentials and normalvectors based on the provided sampling
  /// rate
  /// @param time_between_points_ns Time between each simulation step. This
  /// essentialy determines the sample rate
  void ApplyToTrajectory(
      const double time_between_points_ns = 50000.0) override;

  // Setters

  /// @brief Sets the speed of the object in m/s.
  /// @param speed_ms Speed in m/s.
  inline void SetSpeed(const double speed_ms) {
    speed_ms_ = speed_ms;
    RecalculateParams();
  }

  /// @brief Sets the acceleration of the object in m/s^2.
  /// @param acceleration_ms2 Acceleration in m/s^2.
  inline void SetAcceleration(const double acceleration_ms2) {
    acceleration_ms2_ = acceleration_ms2;
    RecalculateParams();
  }

  /// @brief Get the tangential of the current position -> velocity direction
  /// @param timestamp Time at which the tangent will be returned -> will be
  /// chopped within the wave period
  /// @return Tangential of the velocity
  inline const ObjectPosition2D GetTangentialAt(const double timestamp) const {
    return tangentials_.at(static_cast<int>(timestamp) % tangentials_.size());
  }

  inline const ObjectPosition2D GetNormVecAt(const double timestamp) const {
    return normvecs_.at(static_cast<int>(timestamp) % normvecs_.size());
  }

 protected:
  inline void RecalculateParams() {
    omega_ = acceleration_ms2_ / 2 * speed_ms_;                // w = q/2 * v
    amplitude_ = (speed_ms_ * speed_ms_) / acceleration_ms2_;  // A = v^2/q
  }

 private:
  std::vector<ObjectPosition2D> tangentials_, normvecs_;

  double speed_ms_ = 300.0;        // Speed in m/s
  double acceleration_ms2_ = 9.0;  // Acceleration in m/s^2

  // Wafe simulation parameters
  double omega_ = 0.0;      // Angular velocity in rad/s
  double amplitude_ = 0.0;  // Amplitude of the wave in m
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_WAFE_H