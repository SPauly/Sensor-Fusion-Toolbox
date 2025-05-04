#ifndef UTILS_LAYER_H
#define UTILS_LAYER_H

namespace sensfus {
namespace app {
namespace utils {

/// @brief Base class for all layers in the application.
/// @details This class provides a common interface for all the visual gui
/// layers
class Layer {
 public:
  explicit constexpr Layer() = default;
  virtual ~Layer() = default;

  /// @brief Called when the layer is attached to the application.
  virtual void OnAttach() = 0;

  /// @brief Called when the layer is detached from the application.
  virtual void OnDetach() = 0;

  /// @brief Called every frame to update the layer.
  virtual void OnUIRender() = 0;
};

}  // namespace utils
}  // namespace app
}  // namespace sensfus
#endif  // UTILS_LAYER_H