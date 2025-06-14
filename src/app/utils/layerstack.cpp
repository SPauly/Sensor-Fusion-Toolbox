#include "app/utils/layerstack.h"

#include <algorithm>

namespace sensfus {
namespace app {
namespace utils {

LayerStack::~LayerStack() { clear(); }

void LayerStack::PushLayer(const std::shared_ptr<Layer> &layer) {
  layers_.emplace(layers_.begin() + layer_insert_index_, layer);
  layer_insert_index_++;
  layer->OnAttach();
}

void LayerStack::PushOverlay(const std::shared_ptr<Layer> &overlay) {
  layers_.emplace_back(overlay);
  overlay->OnAttach();
}

void LayerStack::PopLayer(std::shared_ptr<Layer> layer) {
  auto it =
      std::find(layers_.begin(), layers_.begin() + layer_insert_index_, layer);
  if (it != layers_.begin() + layer_insert_index_) {
    layer->OnDetach();
    layers_.erase(it);
    layer_insert_index_--;
  }
}

void LayerStack::PopOverlay(std::shared_ptr<Layer> overlay) {
  auto it =
      std::find(layers_.begin() + layer_insert_index_, layers_.end(), overlay);
  if (it != layers_.end()) {
    overlay->OnDetach();
    layers_.erase(it);
  }
}

void LayerStack::HideLayer(std::shared_ptr<Layer> layer) {
  auto it =
      std::find(layers_.begin(), layers_.begin() + layer_insert_index_, layer);
  if (it != layers_.begin() + layer_insert_index_) {
    hidden_layers_.push_back(layer);
    layers_.erase(it);
    layer_insert_index_--;
  }
}

void LayerStack::ShowLayer(std::shared_ptr<Layer> layer) {
  auto it = std::find(hidden_layers_.begin(), hidden_layers_.end(), layer);
  if (it != hidden_layers_.end()) {
    layers_.emplace(layers_.begin() + layer_insert_index_, layer);
    layer_insert_index_++;
    hidden_layers_.erase(it);
  }
}

void LayerStack::clear() {
  for (std::shared_ptr<Layer> layer : layers_) {
    layer->OnDetach();
    layer.reset();
  }
  for (std::shared_ptr<Layer> layer : hidden_layers_) {
    layer->OnDetach();
    layer.reset();
  }
}

}  // namespace utils
}  // namespace app
}  // namespace sensfus