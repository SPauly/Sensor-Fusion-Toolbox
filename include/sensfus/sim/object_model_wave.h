#ifndef SENSFUS_SIM_OBJECT_MODEL_WAVE_H
#define SENSFUS_SIM_OBJECT_MODEL_WAVE_H

#include "sensfus/internal/object_model_base.h"

#include "sensfus/types.h"

namespace sensfus {
namespace sim {
// Forward declaration of the WaveModel class
template <typename StateType>
class WaveModel;

using WaveModel2D = WaveModel<ObjectState2D>;
using WaveModel3D = WaveModel<ObjectState3D>;

template <typename StateType>
class WaveModel : public ObjectModelBase<StateType> {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D; Use "
                "explicit types WaveModel2D or WaveModel3D for convenience.");

  using VecType = typename ObjectModelBase<StateType>::VecType;
  static constexpr int kDim =
      ObjectModelBase<StateType>::kDim;  // Dimension of the state

  // Type aliases for fast access to base class types
  using ObjectModelBase<StateType>::states_;
  using ObjectModelBase<StateType>::time_between_points_ns_;

 public:
  explicit WaveModel(std::shared_ptr<std::vector<StateType>> states)
      : ObjectModelBase<StateType>(states) {
    RecalculateParams();
  }
  ~WaveModel() override = default;

  /// @brief Populates the states with target positions, velocities,
  /// acceleration, tangentials and normalvectors based on the provided sampling
  /// rate
  /// @param time_between_points_ns Time between each simulation step. This
  /// essentialy determines the sample rate
  virtual void ApplyToTrajectory(
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
  inline VecType GetTangentialAt(
      const TimeStepIdType timestamp) const override {
    return tangentials_.at(timestamp % tangentials_.size());
  }

  /// @brief Get the normal vector of the current position -> perpendicular to
  /// the velocity direction
  /// @param timestamp Time at which the normal vector will be returned ->
  /// will be chopped within the wave period
  /// @return Normal vector of the velocity
  inline VecType GetNormVecAt(const TimeStepIdType timestamp) const override {
    return normvecs_.at(timestamp % normvecs_.size());
  }

 protected:
  inline void RecalculateParams() {
    omega_ = acceleration_ms2_ / 2 * speed_ms_;                // w = q/2 * v
    amplitude_ = (speed_ms_ * speed_ms_) / acceleration_ms2_;  // A = v^2/q
  }

 private:
  std::vector<VecType> tangentials_, normvecs_;

  double speed_ms_ = 300.0;        // Speed in m/s
  double acceleration_ms2_ = 9.0;  // Acceleration in m/s^2
  double height_m_ = 1000.0;       // Height of the wave in m (for 3D waves)

  // Wave simulation parameters
  double omega_ = 0.0;      // Angular velocity in rad/s
  double amplitude_ = 0.0;  // Amplitude of the wave in m
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_WAVE_H

#include "sensfus/sim/object_model_wave_impl.ipp"  // Include the implementation file to define the methods of WaveModel