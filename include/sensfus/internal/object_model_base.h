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
/// @tparam StateType
template <typename StateType = ObjectState2D>
class ObjectModelBase {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

 protected:
  // Type alias for used position type based on the dimension of the state
  static constexpr int kDim =
      std::is_same<StateType, ObjectState2D>::value ? 2 : 3;
  using VecType =
      std::conditional_t<std::is_same<StateType, ObjectState2D>::value,
                         Vector2D,
                         Vector3D>;  // Position, velocity, acceleration type

 public:
  explicit ObjectModelBase(std::shared_ptr<std::vector<StateType>> states)
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

  // Getters

  virtual VecType GetTangentialAt(const TimeStepIdType timestamp) const = 0;
  virtual VecType GetNormVecAt(const TimeStepIdType timestamp) const = 0;

 protected:
  double time_between_points_ns_ = 50000.0;  // Time between points in ns
  std::shared_ptr<std::vector<StateType>> states_ = nullptr;
};

/// @brief Applies a constant velocity model to the trajectory. This model
/// assumes constant velocity between to points without a change in
/// acceleration.
/// @tparam StateType 2D or 3D object state.
template <typename StateType = ObjectState2D>
class BasicVelocityModel : public ObjectModelBase<StateType> {
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

  using VecType = typename ObjectModelBase<StateType>::VecType;

 public:
  explicit BasicVelocityModel(std::shared_ptr<std::vector<StateType>> states)
      : ObjectModelBase<StateType>(states) {}
  ~BasicVelocityModel() override = default;

  virtual void ApplyToTrajectory(
      const double time_between_points_ns = 50000.0) override {
    /// TODO: Implement the basic velocity model
    return;
  }

  virtual VecType GetTangentialAt(
      const TimeStepIdType timestamp) const override {
    if constexpr (std::is_same<StateType, ObjectState2D>::value) {
      return VecType(1, 0);  // Tangential in 2D is (1, 0)
    } else {
      return VecType(1, 0, 0);  // Tangential in 3D is (1, 0, 0)
    }
  }
  virtual VecType GetNormVecAt(const TimeStepIdType timestamp) const override {
    if constexpr (std::is_same<StateType, ObjectState2D>::value) {
      return VecType(0, 1);  // Normal vector in 2D is (0, 1)
    } else {
      return VecType(0, 1, 0);  // Normal vector in 3D is (0, 1, 0)
    }
  }
};

}  // namespace sim

}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_BASE_H