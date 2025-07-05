#include "kalman_sim.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {

KalmanSim::KalmanSim(size_t id, std::shared_ptr<sim::SensorSimulator> sim)
    : id_(id), sim_(std::move(sim)) {
  kalman_ = std::make_shared<kalman::GUIKalmanFilter<ObjectState2D, true>>();

  event_bus_ = sim_->GetEventBus();

  state_sub_ =
      event_bus_->Subscribe<kalman::KalmanState<ObjectState2D>>("KalmanState");
  metadata_sub_ =
      event_bus_->Subscribe<kalman::KalmanStateMetadata<ObjectState2D>>(
          "KalmanStateMetadata");
}

void KalmanSim::OnUIRender() {
  // Check for new data
  auto state = state_sub_->FetchLatest();
  if (state) {
    latest_prediction_ = *state;
    x_predicted_.push_back(state->x(0));
    y_predicted_.push_back(state->x(1));
  }

  auto metadata = metadata_sub_->FetchLatest();
  if (metadata) {
    latest_update_ = *metadata;
    x_updated_.push_back(metadata->xk.x(0));
    y_updated_.push_back(metadata->xk.x(1));
  }

  // --- Kalman Filter Control Window ---
  if (ImGui::Begin("Kalman Filter Control")) {
    // Show latest prediction and update in a table
    if (ImGui::BeginTable("PredictionUpdateTable", 3,
                          ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("Step");
      ImGui::TableSetupColumn("x");
      ImGui::TableSetupColumn("y");
      ImGui::TableHeadersRow();

      // Prediction row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Predicted");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_prediction_.x(0));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_prediction_.x(1));

      // Ground truth row (set to 0 for now)
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Ground Truth");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", 0.0);

      // Difference row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Difference");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_prediction_.x(0) - 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_prediction_.x(1) - 0.0);

      // Update row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Updated");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_update_.xk.x(0));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_update_.xk.x(1));

      // Update difference row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Update Diff");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_update_.xk.x(0) - 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_update_.xk.x(1) - 0.0);

      ImGui::EndTable();
    }

    // Show timestamps
    ImGui::Text(
        "Prediction Timestamp: %llu",
        static_cast<unsigned long long>(latest_prediction_.k_timestamp));
    ImGui::Text("Update Timestamp: %llu",
                static_cast<unsigned long long>(latest_update_.k_timestamp));

    // Euclidean distances
    float pred_dist = std::sqrt(std::pow(latest_prediction_.x(0), 2) +
                                std::pow(latest_prediction_.x(1), 2));
    float upd_dist = std::sqrt(std::pow(latest_update_.xk.x(0), 2) +
                               std::pow(latest_update_.xk.x(1), 2));
    ImGui::Text("Prediction to Target Distance: %.3f", pred_dist);
    ImGui::Text("Update to Target Distance: %.3f", upd_dist);

    // Settings
    static int update_interval = 4;  // In predictions between updates
    static bool enable_retrodiction = false;
    static int retrodiction_steps = 5;
    ImGui::Separator();
    ImGui::Text("Settings");
    ImGui::SliderInt("Update Interval (pred/update)", &update_interval, 0, 10);
    ImGui::Checkbox("Enable Retrodiction", &enable_retrodiction);
    if (enable_retrodiction) {
      ImGui::SliderInt("Retrodiction Steps", &retrodiction_steps, 1, 10);
    }
  }

  ImGui::End();

  // --- Kalman Filter Metadata Window ---
  if (ImGui::Begin("Kalman Filter Metadata")) {
    // Covariance matrix P
    if (ImGui::BeginTable("CovarianceP", 3, ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("P0");
      ImGui::TableSetupColumn("P1");
      ImGui::TableSetupColumn("P2");
      ImGui::TableHeadersRow();
      for (int i = 0; i < 3; ++i) {
        ImGui::TableNextRow();
        for (int j = 0; j < 3; ++j) {
          ImGui::TableSetColumnIndex(j);
          ImGui::Text("%.3f", latest_update_.xk.P(i, j));
        }
      }
      ImGui::EndTable();
    }

    // Show other matrices
    if (ImGui::CollapsingHeader("Other Matrices")) {
      // Innovation
      ImGui::Text("Innovation:");
      for (int i = 0; i < 2; ++i)
        ImGui::Text("  %.3f", latest_update_.innovation(i));

      // Innovation covariance
      ImGui::Text("Innovation Covariance:");
      for (int i = 0; i < 2; ++i)
        ImGui::Text("  %.3f  %.3f", latest_update_.inv_covariance(i, 0),
                    latest_update_.inv_covariance(i, 1));

      // Kalman gain
      ImGui::Text("Kalman Gain:");
      for (int i = 0; i < 6; ++i)
        ImGui::Text("  %.3f  %.3f", latest_update_.kalman_gain(i, 0),
                    latest_update_.kalman_gain(i, 1));
    }
  }
  ImGui::End();
}

void KalmanSim::RunPlotCallback() {
  // This function will be called to render the plot
  ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 2.0f);
  ImPlot::PlotScatter("Predicted", x_predicted_.data(), y_predicted_.data(),
                      (int)x_predicted_.size());

  ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 2.0f);
  ImPlot::PlotScatter("Updated", x_updated_.data(), y_updated_.data(),
                      (int)x_updated_.size());
}
}  // namespace app
}  // namespace sensfus
