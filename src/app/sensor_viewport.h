#ifndef SENSOR_VIEWPORT_H
#define SENSOR_VIEWPORT_H

#include <vector>
#include <functional>
#include <utility>
#include <string>

#include "app/utils/layer.h"

namespace sensfus {
namespace app {

// Callback type for sensor plot overlays
using SensorPlotCallback = std::function<void()>;

// SensorViewport layer
class SensorViewport : public utils::Layer {
 public:
  /// @brief Register a callback to be called during fused plot rendering
  /// @param name Name of the callback to be shown and to identify it. THIS MUST
  /// BE UNIQUE
  /// @param cb Callback of type std::function<void()>
  void RegisterPlotCallback(const std::string name,
                            const SensorPlotCallback cb);

  /// @brief Suspends the callback with the specific name from beeing drawn
  /// @param name Identifier of the given callback
  void SuspendCallback(const std::string& name);

  /// @brief Returns a given callback from suspension. This does nothing if the
  /// callback is not found or is already shown.
  /// @param name Identifier of the given callback
  void ShowCallback(const std::string& name);

  // Layer interface
  virtual void OnAttach() override;
  virtual void OnDetach() override;
  virtual void OnUIRender() override;

 private:
  std::vector<std::pair<const std::string, const SensorPlotCallback>>
      plot_callbacks_;

  // hold the callbacks not shown for now
  std::vector<std::pair<const std::string, bool>> suspendet_;
};

}  // namespace app
}  // namespace sensfus

#endif  // SENSOR_VIEWPORT_H