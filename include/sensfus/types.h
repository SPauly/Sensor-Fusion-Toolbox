#ifndef SENSFUS_TYPES_H
#define SENSFUS_TYPES_H

#include <Eigen/Dense>

namespace sensfus {
// -------------------------------------------------------
// Basic Types
// -------------------------------------------------------

using ScalarType = double;
using TimeStepIdType = unsigned long long;

// -------------------------------------------------------
// ObjectState
// -------------------------------------------------------

using ObjectState2D =
    Eigen::Matrix<ScalarType, 6, 1>;  // x_k = (relative_position^T,
                                      // relative_velocity^t, acceleration^T)^T
using ObjectPosition2D = Eigen::Matrix<ScalarType, 2, 1>;  // x_k = (x, y)^T
using ObjectVelocity2D = Eigen::Matrix<ScalarType, 2, 1>;  // x_k = (vx, vy)^T
using ObjectAcceleration2D =
    Eigen::Matrix<ScalarType, 2, 1>;  // x_k = (ax, ay)^T

using ObjectState3D =
    Eigen::Matrix<ScalarType, 9, 1>;  // x_k = (relative_position^T,
                                      // relative_velocity^T, acceleration^T)^T
using ObjectPosition3D = Eigen::Matrix<ScalarType, 3, 1>;  // x_k = (x, y, z)^T
using ObjectVelocity3D =
    Eigen::Matrix<ScalarType, 3, 1>;  // x_k = (vx, vy, vz)^T
using ObjectAcceleratio3D =
    Eigen::Matrix<ScalarType, 3, 1>;  // x_k = (ax, ay, az)^T

// -------------------------------------------------------
// Data Transmission types
// -------------------------------------------------------

using TargetIdType = unsigned int;
using SensorIdType = unsigned int;

/// @brief Struct to hold the true target state. This includes the position,
/// velocity, and acceleration of the target. The ID is used to identify the
/// targets timestep
struct TrueTargetState2D {
  // Store the raw target state together with the ID
  std::vector<std::pair<TargetIdType, ObjectState2D>> states;

  // Store the update id
  TimeStepIdType id = 0;
};

template <typename ObjectType = ObjectState2D>
struct SensorInfoBase {
  virtual void Clear() {}
};

struct RadarSensorInfo2D : public SensorInfoBase<ObjectState2D> {
  // Store the cartesian coordinates of the sensor
  std::vector<ObjectPosition2D>
      cartesian;  // Sensor position in cartesian coordinates

  std::vector<ObjectPosition2D>
      range_azimuth;  // Sensor position in polar coordinates

  // Store the Sensor and update step
  SensorIdType id = 0;
  TimeStepIdType step = 0;

  virtual void Clear() override {
    cartesian.clear();
    range_azimuth.clear();
  }
};

// -------------------------------------------------------
// ObjectStateWrapper
// -------------------------------------------------------

/// @brief Wrapper class for ObjectState to provide easier access to its
/// components. This class allows you to get and set the position, velocity, and
/// acceleration of an object in a more intuitive way.
template <typename ObjectType = ObjectState2D>
class ObjectStateWrapper {
 public:
  explicit ObjectStateWrapper() = default;
  explicit ObjectStateWrapper(const ObjectType& state) : state_(state) {}
  explicit ObjectStateWrapper(const ObjectStateWrapper& other)
      : state_(other.state_) {}

  // Ensure ObjectType is either ObjectState2D or ObjectState3D
  static_assert(std::is_same<ObjectType, ObjectState2D>::value ||
                    std::is_same<ObjectType, ObjectState3D>::value,
                "ObjectType must be ObjectState2D or ObjectState3D");

  // Determine the dimension of the object state based on the type
  static constexpr int kDim =
      std::is_same<ObjectType, ObjectState2D>::value
          ? 2
          : (std::is_same<ObjectType, ObjectState3D>::value ? 3 : -1);

  // Type aliases for position, velocity, acceleration based on ObjectType
  using ObjectState = ObjectType;
  using ObjectPosition =
      std::conditional_t<std::is_same<ObjectType, ObjectState2D>::value,
                         ObjectPosition2D, ObjectPosition3D>;
  using ObjectVelocity =
      std::conditional_t<std::is_same<ObjectType, ObjectState2D>::value,
                         ObjectVelocity2D, ObjectVelocity3D>;
  using ObjectAcceleration =
      std::conditional_t<std::is_same<ObjectType, ObjectState2D>::value,
                         ObjectAcceleration2D, ObjectAcceleratio3D>;

  ObjectState& GetState() { return state_; }
  const ObjectState& GetState() const { return state_; }

  void SetState(const ObjectState& state) { state_ = state; }
  void SetState(const ObjectStateWrapper& other) { state_ = other.state_; }

  inline ObjectState GetStateCopy() const { return state_; }
  inline ObjectPosition GetPosition() const { return state_.head<kDim>(); }
  inline ObjectVelocity GetVelocity() const {
    return state_.segment<kDim>(kDim);
  }
  inline ObjectAcceleration GetAcceleration() const {
    return state_.tail<kDim>();
  }

  void SetPosition(const ObjectPosition& position) {
    state_.head<3>() = position;
  }
  void SetVelocity(const ObjectVelocity& velocity) {
    state_.segment<3>(3) = velocity;
  }
  void SetAcceleration(const ObjectAcceleration& acceleration) {
    state_.tail<3>() = acceleration;
  }

  ObjectState& operator=(const ObjectState& state) {
    state_ = state;
    return state_;
  }

  ObjectState& operator=(const ObjectStateWrapper& other) {
    state_ = other.state_;
    return state_;
  }

  ObjectState operator+(const ObjectState& other) const {
    return state_ + other;
  }

  ObjectState operator-(const ObjectState& other) const {
    return state_ - other;
  }

  ObjectState operator*(const ScalarType& scalar) const {
    return state_ * scalar;
  }

  ObjectState operator/(const ScalarType& scalar) const {
    return state_ / scalar;
  }

  ObjectState operator+=(const ObjectState& other) {
    state_ += other;
    return state_;
  }
  ObjectState operator-=(const ObjectState& other) {
    state_ -= other;
    return state_;
  }

 private:
  ObjectState state_;
};

}  // namespace sensfus

#endif  // SENSFUS_TYPES_H