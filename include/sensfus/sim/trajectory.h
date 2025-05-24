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
  // Ensure ObjectType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<ObjectType, ObjectState2D>::value ||
                    std::is_same<ObjectType, ObjectState3D>::value,
                "ObjectType must be ObjectState2D or ObjectState3D");

  // Determine the dimension of the object state based on the type
  static constexpr int kDim =
      std::is_same<ObjectType, ObjectState2D>::value ? 2 : 3;

  // Type aliases needed
  using RawPosType =
      std::conditional_t<std::is_same<ObjectType, ObjectState2D>::value,
                         ObjectPosition2D, ObjectPosition3D>;

 public:
  explicit Trajectory(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel)
      : states_(std::make_shared<std::vector<ObjectType>>()),
        object_model_(
            ObjectModelFactory<ObjectType>::CreateObjectModel(type, states_)) {}
  /// @brief Initializes the trajectory with a vector of states.
  /// @param points precomputed trajectory states.
  explicit Trajectory(
      const std::vector<ObjectType>& points,
      const ObjectModelType type = ObjectModelType::BasicVelocityModel)
      : states_(std::make_shared<std::vector<ObjectType>>(std::move(points))),
        object_model_(
            ObjectModelFactory<ObjectType>::CreateObjectModel(type, states_)) {}
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
    states_->clear();
    states_->reserve(line_vector.size());
    for (const auto& point : line_vector) {
      ObjectType state;
      static_assert(
          ObjectType::RowsAtCompileTime == RawPosType::RowsAtCompileTime * 3,
          "Dimension mismatch between ObjectType and RawPosType");
      state.head<kDim>() = point;
      states_->emplace_back(state);
    }
    object_model_->ApplyToTrajectory();
  }

  /// @brief Returns the object state at the given index.
  /// @param index Timestep index of the object state.
  /// @return ObjectType at the given index. When index is out of bounds, it
  /// returns the last valid state.
  const ObjectType& GetState(TimeStepIdType index) const {
    if (index >= states_->size()) {
      index = states_->size() - 1;
    }
    return (*states_)[index];
  }

  /// @brief Returns the number of points in the trajectory.
  /// @return Tragectory size
  inline const unsigned long long GetSize() const { return states_->size(); }

  inline const RawPosType GetTangentialAt(TimeStepIdType timestamp) const {
    return static_cast<RawPosType>(object_model_->GetTangentialAt(timestamp));
  }

  inline const RawPosType GetNormVecAt(TimeStepIdType timestamp) const {
    return static_cast<RawPosType>(object_model_->GetNormVecAt(timestamp));
  }

  inline void SetObjectModel(
      const ObjectModelType type = ObjectModelType::BasicVelocityModel) {
    object_model_ =
        ObjectModelFactory<ObjectType>::CreateObjectModel(type, states_);
    object_model_->ApplyToTrajectory();
  }

 private:
  std::shared_ptr<std::vector<ObjectType>> states_;

  std::shared_ptr<ObjectModelBase<ObjectType>> object_model_ = nullptr;
};

}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_TRAJECTORY_H