#ifndef SENSFUS_SIM_TRAJECTORY_H
#define SENSFUS_SIM_TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>

#include "sensfus/types.h"
#include "sensfus/utils/math.h"
#include "sensfus/sim/object_model.h"

namespace sensfus {
namespace sim {

/// @brief Wrapper to handle creation and access to a trajectory of an object
/// with a specified state type and physics model.
/// @tparam StateType
template <typename StateType = ObjectState2D>
class Trajectory {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

 public:
  // This should only be called by SensorSimulator. Otherwise it is not assigned
  // to a simulator
  explicit Trajectory() = default;
  virtual ~Trajectory() = default;

  /// TODO: Make it non-copyable, non-movable

  /// @brief Returns the number of points in the trajectory.
  /// @return Tragectory size
  virtual const TimeStepIdType GetSize() const { return 0; }

  /// @brief Returns the underlying ModelType of the trajectory
  /// @return Of type sim::ObjectModelType
  virtual const ObjectModelType GetObjectModelType() const {
    return sim::ObjectModelType::BasicVelocityModel;
  }

  [[nodiscard]] virtual std::shared_ptr<ObjectModelBase<StateType>>
  GetObjectModel() const {
    return nullptr;
  }

  // Setters

  /// @brief Sets a new object model for the trajectory. This will clear the
  /// former trajectory data and apply the new model to the current states.
  /// @param type Type of the object model to set. must be one of the
  /// ObjectModelType enum values.
  /// @return The shared_ptr returned only guarantees access to this trajectory
  /// until the next SetObjectModel is called. This can happen from another
  /// thread. To verify if the objectmodel is active call IsActive().
  [[nodiscard]] virtual std::shared_ptr<ObjectModelBase<StateType>>
  SetObjectModel(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel) {
    return nullptr;
  }

  /// @brief This will set the trajectory on repeat until EnableWrapAround is
  /// set to false again
  /// @param wrap Set to true to enable wrap around
  virtual void SetEnableWrapAround(bool wrap = true) {}
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_TRAJECTORY_H