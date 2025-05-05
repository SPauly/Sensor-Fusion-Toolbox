#ifndef SENSOR_VIEWPORT_H
#define SENSOR_VIEWPORT_H

#include <vector>
#include <functional>
#include <string>

#include "app/utils/layer.h"

namespace sensfus {
namespace app {

// Callback type for sensor plot overlays
using SensorPlotCallback = std::function<void()>;

// SensorViewport layer
class SensorViewport : public utils::Layer {
 public:
  // Register a callback to be called during fused plot rendering
  void RegisterPlotCallback(const SensorPlotCallback& cb);

  // Layer interface
  virtual void OnAttach() override;
  virtual void OnDetach() override;
  virtual void OnUIRender() override;

 private:
  std::vector<SensorPlotCallback> plot_callbacks_;
};

}  // namespace app
}  // namespace sensfus

#endif  // SENSOR_VIEWPORT_H