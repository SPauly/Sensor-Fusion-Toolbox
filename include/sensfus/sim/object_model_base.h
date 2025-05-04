#ifndef SENSFUS_SIM_OBJECT_MODEL_BASE_H
#define SENSFUS_SIM_OBJECT_MODEL_BASE_H

#include <chrono>
#include <memory>
#include <vector>

#include "sensfus/types.h"

namespace sensfus {
namespace sim {

/// @brief Physics model for the object. This class is used to apply the physics
/// model to trajectories or calculate current position based on the time
/// passed.
/// @tparam ObjectStateType
template <typename ObjectStateType = ObjectState2D>
class ObjectModelBase {
 public:
  // Ensure ObjectStateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<ObjectStateType, ObjectState2D>::value ||
                    std::is_same<ObjectStateType, ObjectState3D>::value,
                "ObjectStateType must be ObjectState2D or ObjectState3D");

  explicit ObjectModelBase(std::shared_ptr<std::vector<ObjectStateType>> states)
      : states_(states) {}
  virtual ~ObjectModelBase() = default;

  /// @brief Corrects the position based trajectory of the object based on the
  /// underlying physics model. (Applies velocity and acceleration to each
  /// point). Can alter the position of the trajectory points if they do not
  /// match the physics model.
  /// @param time_between_points_ns Time between points in nanoseconds.
  virtual void ApplyToTrajectory(
      const double time_between_points_ns = 50000.0) = 0;

  /// @brief Sets the time that passes between each point in the trajectory.
  /// @param time_between_points_ns time in nanoseconds.
  inline void SetTimeBetweenPointsNs(const double time_between_points_ns) {
    time_between_points_ns_ = time_between_points_ns;
  }

 protected:
  double time_between_points_ns_ = 50000.0;  // Time between points in ns
  std::shared_ptr<std::vector<ObjectStateType>> states_ = nullptr;
};

/// @brief Applies a constant velocity model to the trajectory. This model
/// assumes constant velocity between to points without a change in
/// acceleration.
/// @tparam ObjectStateType 2D or 3D object state.
template <typename ObjectStateType = ObjectState2D>
class BasicVelocityModel : public ObjectModelBase<ObjectStateType> {
 public:
  explicit BasicVelocityModel(
      std::shared_ptr<std::vector<ObjectStateType>> states)
      : ObjectModelBase<ObjectStateType>(states) {}
  ~BasicVelocityModel() override = default;

  void ApplyToTrajectory(
      const double time_between_points_ns = 50000.0) override {
    /// TODO: Implement the basic velocity model
    return;
  }
};

}  // namespace sim

}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_BASE_H