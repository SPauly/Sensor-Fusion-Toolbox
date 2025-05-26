#ifndef SENSFUS_SIM_TRAJECTORY_H
#define SENSFUS_SIM_TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>

#include "sensfus/types.h"
#include "sensfus/utils/math.h"

namespace sensfus {
namespace sim {
// Forward declarations
class SensorSimulator;

/// @brief Wrapper to handle creation and access to a trajectory of an object
/// with a specified state type and physics model.
/// @tparam StateType
template <typename StateType = ObjectState2D>
class Trajectory {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

  // Only allow creation by the SensorSimulator
  friend class SensorSimulator;
  explicit Trajectory() = default;

 public:
  virtual ~Trajectory() = default;

  /// @brief Returns the number of points in the trajectory.
  /// @return Tragectory size
  virtual const TimeStepIdType GetSize() const = 0;

  /// @brief Returns the underlying ModelType of the trajectory
  /// @return Of type sim::ObjectModelType
  virtual const ObjectModelType GetObjectModelType() const = 0;

  // Setters

  /// @brief Sets a new object model for the trajectory. This will clear the
  /// former trajectory data and apply the new model to the current states.
  /// @param type Type of the object model to set. must be one of the
  /// ObjectModelType enum values.
  virtual void SetObjectModel(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel) = 0;

  /// @brief
  virtual void SetEnableWrapAround(bool wrap = true) = 0;

 private:
  bool enable_wrap_around_ = false;  // If true, the trajectory will wrap around
                                     // when accessing out of bounds indices.

  std::shared_ptr<std::vector<StateType>> states_;

  std::shared_ptr<ObjectModelBase<StateType>> object_model_ = nullptr;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_TRAJECTORY_H