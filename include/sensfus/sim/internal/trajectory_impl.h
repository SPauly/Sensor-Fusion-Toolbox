#ifndef SENSFUS_SIM_INTERNAL_TRAJECTORY_IMPL_H
#define SENSFUS_SIM_INTERNAL_TRAJECTORY_IMPL_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>

#include "sensfus/types.h"
#include "sensfus/utils/math.h"
#include "sensfus/sim/object_model.h"
#include "sensfus/sim/trajectory.h"

namespace sensfus {
namespace sim {
namespace internal {

/// @brief Wrapper to handle creation and access to a trajectory of an object
/// with a specified state type and physics model.
/// @tparam StateType
template <typename StateType = ObjectState2D>
class TrajectoryImpl : public sensfus::sim::Trajectory<StateType> {
  // Ensure StateType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<StateType, ObjectState2D>::value ||
                    std::is_same<StateType, ObjectState3D>::value,
                "StateType must be ObjectState2D or ObjectState3D");

  // Determine the dimension of the object state based on the type
  static constexpr int kDim =
      std::is_same<StateType, ObjectState2D>::value ? 2 : 3;

  // Type aliases needed
  using RawPosType =
      std::conditional_t<std::is_same<StateType, ObjectState2D>::value,
                         ObjectPosition2D, ObjectPosition3D>;

 public:
  explicit TrajectoryImpl(const sim::ObjectModelType type =
                              sim::ObjectModelType::BasicVelocityModel,
                          TimeStepIdType time_between_points_ns = 1000.0)
      : states_(std::make_shared<std::vector<StateType>>()),
        object_model_(sim::ObjectModelFactory<StateType>::CreateObjectModel(
            type, states_)) {
    // also set the time between points for the model
    object_model_->SetTimeBetweenPointsNs(time_between_points_ns);
    object_model_->ApplyToTrajectory();
  }

  /// @brief Initializes the trajectory with a vector of states.
  /// @param points precomputed trajectory states.
  explicit TrajectoryImpl(const std::vector<StateType>& points,
                          const sim::ObjectModelType type =
                              sim::ObjectModelType::BasicVelocityModel,
                          TimeStepIdType time_between_points_ns = 1000.0)
      : states_(std::make_shared<std::vector<StateType>>(std::move(points))),
        object_model_(sim::ObjectModelFactory<StateType>::CreateObjectModel(
            type, states_)) {
    object_model_->SetTimeBetweenPointsNs(time_between_points_ns);
    object_model_->ApplyToTrajectory();
  }
  /// @brief Creates a trajectory based on the given points using the underlying
  /// physics model for the target. (This will decide velocity and acceleration
  /// during each point transition.)
  /// @param line_vector Vector of 2D or 3D points representing the trajectory.
  explicit TrajectoryImpl(const std::vector<RawPosType>& line_vector,
                          const sim::ObjectModelType type =
                              sim::ObjectModelType::BasicVelocityModel,
                          TimeStepIdType time_between_points_ns = 1000.0)
      : TrajectoryImpl(type, time_between_points_ns) {
    FromLineVector(line_vector);
  }
  virtual ~TrajectoryImpl() = default;

  /// @brief Creates a trajectory based on the given points using the underlying
  /// physics model for the target. (This will decide velocity and acceleration
  /// during each point transition.)
  /// @param line_vector Vector of 2D or 3D points representing the trajectory.
  void FromLineVector(const std::vector<RawPosType>& line_vector) {
    std::unique_lock<std::mutex> lock(mtx_);
    states_->clear();
    states_->reserve(line_vector.size());
    for (const auto& point : line_vector) {
      StateType state;
      static_assert(
          StateType::RowsAtCompileTime == RawPosType::RowsAtCompileTime * 3,
          "Dimension mismatch between StateType and RawPosType");
      state.head<kDim>() = point;
      states_->emplace_back(state);
    }
    object_model_->ApplyToTrajectory();
  }

  /// @brief Returns the object state at the given index.
  /// @param index Timestep index of the object state.
  /// @return StateType at the given index. When index is out of bounds, it
  /// returns the last valid state.
  const StateType& GetState(TimeStepIdType index) const {
    std::unique_lock<std::mutex> lock(mtx_);

    if (!enable_wrap_around_ && index >= states_->size()) {
      index = states_->size() - 1;
    }
    return (*states_)[utils::fast_mod(index, states_->size())];
  }

  inline const RawPosType GetTangentialAt(TimeStepIdType timestamp) const {
    std::unique_lock<std::mutex> lock(mtx_);
    return static_cast<RawPosType>(object_model_->GetTangentialAt(timestamp));
  }

  inline const RawPosType GetNormVecAt(TimeStepIdType timestamp) const {
    std::unique_lock<std::mutex> lock(mtx_);
    return static_cast<RawPosType>(object_model_->GetNormVecAt(timestamp));
  }

  /// @brief Return a copy of the trajectory states.
  /// @return .
  std::vector<StateType> GetStates() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return std::copy(*states_);
  }

  // Implement Functions from the client interface here
  //------------------------------------------------------

  /// @brief Returns the number of points in the trajectory.
  /// @return Tragectory size
  virtual const TimeStepIdType GetSize() const override {
    std::unique_lock<std::mutex> lock(mtx_);
    if (enable_wrap_around_ && !states_->empty())
      return -1;  // Simulate infinite trajectory
    return states_->size();
  }

  /// @brief Returns the underlying ModelType of the trajectory
  /// @return Of type sim::ObjectModelType
  virtual const sim::ObjectModelType GetObjectModelType() const override {
    std::unique_lock<std::mutex> lock(mtx_);
    return type_;
  }

  /// @brief Returns the object model of the trajectory.
  /// @return Shared Pointer to the model base. Get the type to interpret it
  /// properly
  [[nodiscard]] virtual std::shared_ptr<sim::ObjectModelBase<StateType>>
  GetObjectModel() const override {
    std::unique_lock<std::mutex> lock(mtx_);
    return object_model_;
  }

  bool GetEnableWrapAround() const {
    std::unique_lock<std::mutex> lock(mtx_);
    return enable_wrap_around_;
  }

  // Setters

  /// @brief Sets a new object model for the trajectory. This will clear the
  /// former trajectory data and apply the new model to the current states.
  /// @param type Type of the object model to set. must be one of the
  /// ObjectModelType enum values.
  /// @return The shared_ptr returned only guarantees access to this trajectory
  /// until the next SetObjectModel is called. This can happen from another
  /// thread. To verify if the objectmodel is active call IsActive().
  [[nodiscard]] virtual std::shared_ptr<sim::ObjectModelBase<StateType>>
  SetObjectModel(const sim::ObjectModelType type =
                     sim::ObjectModelType::BasicVelocityModel) override {
    std::unique_lock<std::mutex> lock(mtx_);

    states_->clear();
    // deactivate the old model
    object_model_->SetIsActive(false);

    TimeStepIdType time_between_points_ns =
        object_model_->GetTimeBetweenPointsNs();

    object_model_ =
        sim::ObjectModelFactory<StateType>::CreateObjectModel(type, states_);
    object_model_->SetTimeBetweenPointsNs(time_between_points_ns);
    object_model_->ApplyToTrajectory();

    // set object model to be the active one
    object_model_->SetIsActive(true);

    type_ = type;

    return std::static_pointer_cast<sim::ObjectModelBase<StateType>>(
        object_model_);
  }

  /// @brief This will set the trajectory on repeat until EnableWrapAround is
  /// set to false again
  /// @param wrap Set to true to enable wrap around
  virtual void SetEnableWrapAround(bool wrap = true) override {
    std::unique_lock<std::mutex> lock(mtx_);
    enable_wrap_around_ = wrap;
  }

 private:
  mutable std::mutex mtx_;

  bool enable_wrap_around_ = true;  // If true, the trajectory will wrap around
                                    // when accessing out of bounds indices.

  std::shared_ptr<std::vector<StateType>> states_;

  std::shared_ptr<sim::ObjectModelBase<StateType>> object_model_ = nullptr;
  sim::ObjectModelType type_;
};

}  // namespace internal
}  // namespace sim
}  // namespace sensfus

#endif  // SENSFUS_SIM_INTERNAL_TRAJECTORY_IMPL_H