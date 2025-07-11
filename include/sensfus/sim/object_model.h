#ifndef SENSFUS_SIM_OBJECT_MODEL_H
#define SENSFUS_SIM_OBJECT_MODEL_H

#include "sensfus/types.h"
#include "sensfus/sim/internal/object_model_base.h"
#include "sensfus/sim/object_model_Wave.h"

namespace sensfus {
namespace sim {

/// @brief Enum class to define different object model types.
/// This enum is used to specify the type of object model to be created.
enum class ObjectModelType {
  BasicVelocityModel = 0,
  WaveModel = 1
  // Add more models as needed
};

/// @brief Creates the object model based on the given type. This class is a
/// factory method.
/// @tparam StateType 2D or 3D object state.
template <typename StateType = ObjectState2D>
class ObjectModelFactory {
 public:
  /// @brief Creates an object model based on the given type.
  /// @param type Type of the object model.
  /// @return Pointer to the created object model.
  static std::shared_ptr<ObjectModelBase<StateType>> CreateObjectModel(
      const ObjectModelType type,
      std::shared_ptr<std::vector<StateType>> states) {
    if (type == ObjectModelType::BasicVelocityModel) {
      return std::make_shared<BasicVelocityModel<StateType>>(states);
    } else if (type == ObjectModelType::WaveModel) {
      return std::make_shared<WaveModel<StateType>>(states);
    }
    // Add more models as needed
    return nullptr;
  }
};

}  // namespace sim

}  // namespace sensfus

#endif  // SENSFUS_SIM_OBJECT_MODEL_H