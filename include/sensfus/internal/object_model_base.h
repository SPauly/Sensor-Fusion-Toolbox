#ifndef SENSFUS_SIM_OBJECT_MODEL_BASE_H
#define SENSFUS_SIM_OBJECT_MODEL_BASE_H

#include <memory>
#include <vector>
#include <mutex>

#include "sensfus/types.h"

namespace sensfus {
namespace internal {
template <typename StateType>
class TrajectoryImpl;
}
namespace sim {
class SensorSimulator;

/// @brief Physics model for the object. This class is used to apply the
/// physics model to trajectories or calculate current position based on the
/// time passed.
/// @tparam StateType
template <typename StateType = ObjectState2D>
class ObjectModelBase {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

 protected:
  // Allow these classes access to changing the active state
  friend class SensorSimulator;
  friend class internal::TrajectoryImpl<StateType>;

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
  virtual void ApplyToTrajectory() {
    static_assert(
        "Don't call ApplyToTrajectory from non specialized ObjectModelType -> "
        "convert to needed spezialisation first");
  }

  /// @brief Sets the time that passes between each point in the trajectory.
  /// @param time_between_points_ns time in nanoseconds.
  inline void SetTimeBetweenPointsNs(const double time_between_points_ns) {
    std::unique_lock<std::mutex> lock(mtx_);
    time_between_points_ns_ = time_between_points_ns;

    lock.unlock();
    ApplyToTrajectory();
  }

  // Getters
  /// @brief Active state of the model
  /// @return true if tied to a Trajectory
  bool IsActive() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return is_active_;
  }

 protected:
  /// @brief Set the active state of the Model -> used to indicate that this
  /// object is tied to an active trajectory
  /// @param active true for 'is tied to trajectory'
  /// @return IsActive
  bool SetIsActive(bool active = true) { return is_active_ = active; }

  // Define these for global access via the SensorSimulator
  VecType GetTangentialAt(const TimeStepIdType timestamp) const {
    return GetTangentialAtImpl(timestamp);
  }
  VecType GetNormVecAt(const TimeStepIdType timestamp) const {
    return GetNormVecAtImpl(timestamp);
  }

  virtual VecType GetTangentialAtImpl(const TimeStepIdType timestamp) const {
    static_assert(
        "Don't call GetTangentialImpl from non specialized ObjectModelType -> "
        "convert to needed spezialisation first");
  }
  virtual VecType GetNormVecAtImpl(const TimeStepIdType timestamp) const {
    static_assert(
        "Don't call GetNormVecAtImpl from non specialized ObjectModelType -> "
        "convert to needed spezialisation first");
  }

 protected:
  mutable std::mutex mtx_;

  bool is_active_ =
      true;  // Indicate wether this is tied to an active trajectory

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

  // Allow these classes access to changing the active state
  friend class SensorSimulator;
  friend class internal::TrajectoryImpl<StateType>;

 public:
  explicit BasicVelocityModel(std::shared_ptr<std::vector<StateType>> states)
      : ObjectModelBase<StateType>(states) {}
  ~BasicVelocityModel() override = default;

  virtual void ApplyToTrajectory() override {
    /// TODO: Implement the basic velocity model
    return;
  }

 protected:
  virtual VecType GetTangentialAtImpl(
      const TimeStepIdType timestamp) const override {
    if constexpr (std::is_same<StateType, ObjectState2D>::value) {
      return VecType(1, 0);  // Tangential in 2D is (1, 0)
    } else {
      return VecType(1, 0, 0);  // Tangential in 3D is (1, 0, 0)
    }
  }
  virtual VecType GetNormVecAtImpl(
      const TimeStepIdType timestamp) const override {
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