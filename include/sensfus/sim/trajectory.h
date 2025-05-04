#ifndef SENSFUS_SIM_TRAJECTORY_H
#define SENSFUS_SIM_TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "sensfus/types.h"
#include "sensfus/sim/object_model.h"

namespace sensfus {
namespace sim {

/// @brief Wrapper to handle creation and access to a trajectory of an object
/// with a specified state type and physics model.
/// @tparam ObjectType
template <typename ObjectType = ObjectState2D>
class Trajectory {
 public:
  // Ensure ObjectType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<ObjectType, ObjectState2D>::value ||
                    std::is_same<ObjectType, ObjectState3D>::value,
                "ObjectType must be ObjectState2D or ObjectState3D");

  // Determine the dimension of the object state based on the type
  static constexpr int kDim =
      std::is_same<ObjectType, ObjectState2D>::value
          ? 2
          : (std::is_same<ObjectType, ObjectState3D>::value ? 3 : -1);

  // Type aliases needed
  using RawPosType =
      std::conditional_t<std::is_same<ObjectType, ObjectState2D>::value,
                         SensVec2D, SensVec3D>;

  explicit Trajectory(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel)
      : points_(std::make_shared<std::vector<ObjectType>>()),
        object_model_(
            ObjectModelFactory<ObjectType>::CreateObjectModel(type, points_)) {}
  /// @brief Initializes the trajectory with a vector of states.
  /// @param points precomputed trajectory states.
  explicit Trajectory(
      const std::vector<ObjectType>& points,
      const ObjectModelType type = ObjectModelType::BasicVelocityModel)
      : Trajectory(type), points_(points) {}
  /// @brief Creates a trajectory based on the given points using the underlying
  /// physics model for the target. (This will decide velocity and acceleration
  /// during each point transition.)
  /// @param line_vector Vector of 2D or 3D points representing the trajectory.
  explicit Trajectory(
      const std::vector<RawPosType>& line_vector,
      const ObjectModelType type = ObjectModelType::BasicVelocityModel)
      : Trajectory(type) {
    FromLineVector(line_vector);
  }
  virtual ~Trajectory() = default;

  /// @brief Creates a trajectory based on the given points using the underlying
  /// physics model for the target. (This will decide velocity and acceleration
  /// during each point transition.)
  /// @param line_vector Vector of 2D or 3D points representing the trajectory.
  void FromLineVector(const std::vector<RawPosType>& line_vector) {
    points_->clear();
    points_->reserve(line_vector.size());
    for (const auto& point : line_vector) {
      ObjectType state;
      if constexpr (kDim == 2) {
        state.head<2>() = ObjectPosition2D(point.x, point.y);
      } else if constexpr (kDim == 3) {
        state.head<3>() = ObjectPosition3D(point.x, point.y, point.z);
      }
      points_->emplace_back(state);
    }
    object_model_->ApplyToTrajectory();
  }

  /// @brief Returns the object state at the given index.
  /// @param index Timestep index of the object state.
  /// @return ObjectType at the given index. When index is out of bounds, it
  /// returns the last valid state.
  const ObjectType& GetState(unsigned long long index) const {
    if (index >= points_->size()) {
      index = points_->size() - 1;
    }
    return (*points_)[index];
  }

  /// @brief Returns the object state at the given index.
  /// @return Tragectory size
  inline const unsigned long long GetSize() const { return points_->size(); }

  inline void SetObjectModel(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel) {
    object_model_ = ObjectModelFactory<ObjectType>::CreateObjectModel(type);
  }

 private:
  std::shared_ptr<std::vector<ObjectType>> points_;

  std::shared_ptr<ObjectModelBase<ObjectType>> object_model_ = nullptr;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_TRAJECTORY_H