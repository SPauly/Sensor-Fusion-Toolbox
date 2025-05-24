#include "app/target_plot.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {
void TargetPlot::RunControllInterface() {
  /// TODO: Implement the control interface for the target plot to feature
  /// restart etc.

  // Display Target Positions and Velocities
  for (size_t i = 0; i < id_of_target_at_index_.size(); i++) {
    ImGui::Text("Status Target %f",
                id_of_target_at_index_.at(static_cast<TargetIdType>(i)));

    ImGui::Text("Cartesian Position: x: %f  y: %f", cart_x_.at(i).back(),
                cart_y_.at(i).back());

    /// TODO: Add the rest
    ImGui::Separator();
  }
}

void TargetPlot::RunPlotInterface() {
  for (size_t i = 0; i < id_of_target_at_index_.size(); i++) {
    ImPlot::PlotScatter(labels_.at(i).c_str(), cart_x_.at(i).data(),
                        cart_y_.at(i).data(), (int)(cart_x_.at(i).size()));
    /// TODO: Draw lines here
  }
}

void TargetPlot::AddTargetUpdate(
    const std::shared_ptr<const TrueTargetState2D> update) {
  for (size_t i = 0; i < update->states.size(); i++) {
    // First get the index of this target
    auto it = id_of_target_at_index_.find(update->states.at(i).first);
    size_t index = 0;
    if (it != id_of_target_at_index_.end()) {
      index = it->second;  // Retrieve the index of this target in the vectors
    } else {
      id_of_target_at_index_.emplace(
          std::make_pair(update->states.at(i).first, cart_x_.size()));

      // Make room for the new target
      cart_x_.push_back(std::vector<double>());
      cart_y_.push_back(std::vector<double>());

      velo_x_.push_back(0.0);
      velo_y_.push_back(0.0);
      acc_x_.push_back(0.0);
      acc_y_.push_back(0.0);

      std::string tmp = "Target " + std::to_string(update->states.at(i).first);
      labels_.push_back(tmp);
      index = cart_x_.size() - 1;
    }

    cart_x_.at(index).push_back(update->states.at(i).second(0));
    cart_y_.at(index).push_back(update->states.at(i).second(1));

    velo_x_.at(index) = update->states.at(i).second(2);
    velo_y_.at(index) = update->states.at(i).second(3);

    acc_x_.at(index) = update->states.at(i).second(4);
    acc_y_.at(index) = update->states.at(i).second(5);
  }
}

}  // namespace app

}  // namespace sensfus
