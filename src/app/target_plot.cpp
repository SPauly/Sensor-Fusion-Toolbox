#include "app/target_plot.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {
void TargetPlot::RunControllInterface() {
  // Display Target Positions and Velocities
  for (size_t i = 0; i < id_of_target_at_index_.size(); i++) {
    TargetIdType targ = id_of_target_at_index_.at(static_cast<TargetIdType>(i));

    ImGui::Text("Status Target %lli", targ);

    ImGui::Text("Cartesian Position: x: %lf  y: %lf", cart_x_.at(targ).back(),
                cart_y_.at(targ).back());

    ImGui::Text("Current Velocity: (%lf,%lf)  Current Acceleration: (%lf,%lf)",
                velo_x_.at(targ), velo_y_.at(targ), acc_x_.at(targ),
                acc_y_.at(targ));

    ImGui::Separator();
  }
}

void TargetPlot::RunPlotInterface() {
  ImDrawList* draw_list = ImPlot::GetPlotDrawList();

  for (size_t i = 0; i < id_of_target_at_index_.size(); i++) {
    const auto& x = cart_x_.at(i);
    const auto& y = cart_y_.at(i);

    // Set line style for thin continuous lines
    ImPlot::SetNextLineStyle(ImVec4(0, 0.5f, 1.0f, -1.0f),
                             1.0f);  // RGBA + thickness
    ImPlot::PlotLine(labels_.at(i).c_str(), x.data(), y.data(),
                     (int)(x.size()));

    // Optionally add small scatter points (visual clarity)
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.0f);
    ImPlot::PlotScatter((labels_.at(i) + " points").c_str(), x.data(), y.data(),
                        (int)(x.size()));

    // Draw velocity and acceleration vectors
    // Draw arrow vectors for the latest update
    if (!x.empty()) {
      const ImPlotPoint p0 = ImPlot::PlotToPixels(x.back(), y.back());

      // Get current index (assuming same order as tangents/normals)
      ObjectPosition2D t, n;
      t.setZero();
      n.setZero();

      for (const auto& [id, vec] : last_update_->tangentials) {
        if (id == i) t = vec;
      }
      for (const auto& [id, vec] : last_update_->normvecs) {
        if (id == i) n = vec;
      }

      // Get acceleration vector a = (ax, ay)
      Eigen::Vector2d a(acc_x_.at(i), acc_y_.at(i));

      // Tangential and normal components of acceleration
      double a_t = a.dot(t);
      double a_n = a.dot(n);

      double scale = 100.0;  // for visualization

      // Draw r_dot (velocity vector)
      Eigen::Vector2d r_dot(velo_x_.at(i), velo_y_.at(i));
      ImVec2 v_end = ImPlot::PlotToPixels(x.back() + scale / 10.0 * r_dot.x(),
                                          y.back() + scale / 10.0 * r_dot.y());
      draw_list->AddLine(ImVec2((float)p0.x, (float)p0.y), v_end,
                         IM_COL32(0, 255, 0, 255), 1.0f);  // Green for velocity

      // Draw r_ddot (acceleration vector)
      ImVec2 a_end = ImPlot::PlotToPixels(x.back() + scale * a.x(),
                                          y.back() + scale * a.y());
      draw_list->AddLine(ImVec2((float)p0.x, (float)p0.y), a_end,
                         IM_COL32(255, 0, 0, 255),
                         1.0f);  // Red for acceleration

      // Draw tangential component a_t * t
      Eigen::Vector2d a_t_vec = a_t * t;
      ImVec2 at_end = ImPlot::PlotToPixels(x.back() + scale * a_t_vec.x(),
                                           y.back() + scale * a_t_vec.y());
      draw_list->AddLine(ImVec2((float)p0.x, (float)p0.y), at_end,
                         IM_COL32(0, 255, 255, 255),
                         1.0f);  // Cyan for tangential

      // Draw normal component a_n * n
      Eigen::Vector2d a_n_vec = a_n * n;
      ImVec2 an_end = ImPlot::PlotToPixels(x.back() + scale * a_n_vec.x(),
                                           y.back() + scale * a_n_vec.y());
      draw_list->AddLine(ImVec2((float)p0.x, (float)p0.y), an_end,
                         IM_COL32(255, 0, 255, 255),
                         1.0f);  // Magenta for normal
    }
  }

  RunControllInterface();
}

void TargetPlot::AddTargetUpdate(
    const std::shared_ptr<const TrueTargetState2D> update) {
  // Store the last update for later use
  last_update_ = update;

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
