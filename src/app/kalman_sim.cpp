#include "kalman_sim.h"

#include <imgui.h>
#include <implot.h>

namespace sensfus {
namespace app {

KalmanSim::KalmanSim(size_t id, std::shared_ptr<sim::SensorSimulator> sim)
    : id_(id), sim_(std::move(sim)) {
  event_bus_ = sim_->GetEventBus();

  state_sub_ =
      event_bus_->Subscribe<kalman::KalmanState<ObjectState2D>>("KalmanState");
  retro_sub_ = event_bus_->Subscribe<kalman::KalmanState<ObjectState2D>>(
      "KalmanRetrodictState");
  metadata_sub_ =
      event_bus_->Subscribe<kalman::KalmanStateMetadata<ObjectState2D>>(
          "KalmanStateMetadata");

  kalman_ =
      std::make_shared<kalman::KalmanFilterWithEventBus<ObjectState2D, true>>(
          event_bus_);
}

void KalmanSim::OnUIRender() {
  // Check for new data
  auto state = state_sub_->Fetch();
  if (state) {
    latest_prediction_ = *state;

    x_predicted_.push_back(latest_prediction_.x(0));
    y_predicted_.push_back(latest_prediction_.x(1));
  }

  auto metadata = metadata_sub_->Fetch();
  if (metadata) {
    latest_update_ = *metadata;

    x_updated_.push_back(latest_update_.xk.x(0));
    y_updated_.push_back(latest_update_.xk.x(1));
  }

  auto retro_state = retro_sub_->Fetch();
  if (retro_state) {
    x_retro_.push_back(retro_state->x(0));
    y_retro_.push_back(retro_state->x(1));
  }

  // --- Kalman Filter Control Window ---
  if (ImGui::Begin("Kalman Filter Control")) {
    // Show latest prediction and update in a table
    if (ImGui::BeginTable("PredictionUpdateTable", 7,
                          ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("Step");
      ImGui::TableSetupColumn("x");
      ImGui::TableSetupColumn("y");
      ImGui::TableSetupColumn("velo_x");
      ImGui::TableSetupColumn("velo_y");
      ImGui::TableSetupColumn("acc_x");
      ImGui::TableSetupColumn("acc_y");
      ImGui::TableHeadersRow();

      // Prediction row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Predicted");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_prediction_.x(0));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_prediction_.x(1));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3f", latest_prediction_.x(2));
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3f", latest_prediction_.x(3));
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3f", latest_prediction_.x(4));
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.3f", latest_prediction_.x(5));

      // Ground truth row (set to 0 for now)
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Ground Truth");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3f", 0.0);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.3f", 0.0);

      // Difference row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Difference");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_prediction_.x(0) - 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_prediction_.x(1) - 0.0);
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3f", latest_prediction_.x(2) - 0.0);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3f", latest_prediction_.x(3) - 0.0);
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3f", latest_prediction_.x(4) - 0.0);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.3f", latest_prediction_.x(5) - 0.0);

      // Update row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Updated");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_update_.xk.x(0));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_update_.xk.x(1));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3f", latest_update_.xk.x(2));
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3f", latest_update_.xk.x(3));
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3f", latest_update_.xk.x(4));
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.3f", latest_update_.xk.x(5));

      // Update difference row
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Update Diff");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", latest_update_.xk.x(0) - 0.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.3f", latest_update_.xk.x(1) - 0.0);
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3f", latest_update_.xk.x(2) - 0.0);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3f", latest_update_.xk.x(3) - 0.0);
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3f", latest_update_.xk.x(4) - 0.0);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.3f", latest_update_.xk.x(5) - 0.0);

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
    static int update_interval = 4,
               prev_update_interval = 0;  // In predictions between updates
    static bool enable_retrodiction = false;
    static int retrodiction_steps = 5;
    static float meas_noise = 10000.0, process_noise = 10000.0;
    ImGui::Separator();
    ImGui::Text("Settings");
    ImGui::SliderInt("Update Interval (pred/update)", &update_interval, 0, 10);
    if (update_interval != prev_update_interval) {
      // Update the Kalman filter with the new interval
      std::static_pointer_cast<
          kalman::KalmanFilterWithEventBus<ObjectState2D, true>>(kalman_)
          ->SetUpdateInterval(update_interval);
      prev_update_interval = update_interval;
    }

    static bool enable_dynamic_update = false;
    ImGui::Checkbox("Enable dynamic update interval", &enable_dynamic_update);

    ImGui::Checkbox("Enable Retrodiction", &enable_retrodiction);
    if (enable_retrodiction) {
      ImGui::SliderInt("Retrodiction Steps", &retrodiction_steps, 1, 10);
    }

    ImGui::InputFloat("Measurement Noise R", &meas_noise, 0.0f, 0.0f, "%.3f");
    ImGui::InputFloat("Process Noise", &process_noise, 0.0f, 0.0f, "%.3f");

    if (ImGui::Button("Apply Settings")) {
      // Apply the settings to the Kalman filter
      std::static_pointer_cast<
          kalman::KalmanFilterWithEventBus<ObjectState2D, true>>(kalman_)
          ->SetMeasurementNoise(meas_noise);
      std::static_pointer_cast<
          kalman::KalmanFilterWithEventBus<ObjectState2D, true>>(kalman_)
          ->SetProcessNoise(process_noise);
    }

    // Show F and D matrices if available
    if (ImGui::CollapsingHeader("F and D Matrices")) {
      // State transition matrix F
      ImGui::Text("State Transition Matrix F:");
      if (latest_update_.xk.F.rows() > 0 && latest_update_.xk.F.cols() > 0) {
        if (ImGui::BeginTable("FMatrix", latest_update_.xk.F.cols(),
                              ImGuiTableFlags_Borders)) {
          for (int i = 0; i < latest_update_.xk.F.rows(); ++i) {
            ImGui::TableNextRow();
            for (int j = 0; j < latest_update_.xk.F.cols(); ++j) {
              ImGui::TableSetColumnIndex(j);
              ImGui::Text("%.3f", latest_update_.xk.F(i, j));
            }
          }
          ImGui::EndTable();
        }
      } else {
        ImGui::Text("F matrix not available.");
      }

      // Evolution covariance matrix D
      ImGui::Text("Evolution Covariance Matrix D:");
      if (latest_update_.xk.D.rows() > 0 && latest_update_.xk.D.cols() > 0) {
        if (ImGui::BeginTable("DMatrix", latest_update_.xk.D.cols(),
                              ImGuiTableFlags_Borders)) {
          for (int i = 0; i < latest_update_.xk.D.rows(); ++i) {
            ImGui::TableNextRow();
            for (int j = 0; j < latest_update_.xk.D.cols(); ++j) {
              ImGui::TableSetColumnIndex(j);
              ImGui::Text("%.3f", latest_update_.xk.D(i, j));
            }
          }
          ImGui::EndTable();
        }
      } else {
        ImGui::Text("D matrix not available.");
      }
    }
  }

  ImGui::End();

  // --- Kalman Filter Metadata Window ---
  if (ImGui::Begin("Kalman Filter Metadata")) {
    // Covariance matrix P (6x6)
    if (ImGui::BeginTable("CovarianceP", 6, ImGuiTableFlags_Borders)) {
      ImGui::TableSetupColumn("P0");
      ImGui::TableSetupColumn("P1");
      ImGui::TableSetupColumn("P2");
      ImGui::TableSetupColumn("P3");
      ImGui::TableSetupColumn("P4");
      ImGui::TableSetupColumn("P5");
      ImGui::TableHeadersRow();
      for (int i = 0; i < 6; ++i) {
        ImGui::TableNextRow();
        for (int j = 0; j < 6; ++j) {
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
        ImGui::Text("  %.20f  %.20f", latest_update_.kalman_gain(i, 0),
                    latest_update_.kalman_gain(i, 1));
    }
  }
  ImGui::End();
}

void KalmanSim::RunPlotCallback() {
  // This function will be called to render the plot
  ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 3.0f);
  ImPlot::PushStyleColor(ImPlotCol_MarkerFill, IM_COL32(100, 200, 255, 255));
  ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, IM_COL32(50, 150, 255, 255));
  ImPlot::PlotScatter("Predicted", x_predicted_.data(), y_predicted_.data(),
                      (int)x_predicted_.size());
  ImPlot::PopStyleColor(2);  // Pop marker colors

  ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 3.0f);
  ImPlot::PushStyleColor(ImPlotCol_MarkerFill, IM_COL32(0, 255, 0, 255));
  ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, IM_COL32(0, 200, 0, 255));
  ImPlot::PlotScatter("Updated", x_updated_.data(), y_updated_.data(),
                      (int)x_updated_.size());

  ImPlot::PopStyleColor(2);  // Pop marker colors

  ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 3.0f);
  ImPlot::PushStyleColor(ImPlotCol_MarkerFill, IM_COL32(255, 0, 0, 255));
  ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, IM_COL32(200, 0, 0, 255));
  ImPlot::PlotScatter("Retrodicted", x_retro_.data(), y_retro_.data(),
                      (int)x_retro_.size());

  ImPlot::PopStyleColor(2);  // Pop marker colors

  ImPlot::PlotLine("Smoothed", x_retro_.data(), y_retro_.data(),
                   x_retro_.size());

  ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f));  // semi-transparent red
  ImPlot::PlotLine("Filtered", x_updated_.data(), y_updated_.data(),
                   (int)x_updated_.size());
}
}  // namespace app
}  // namespace sensfus
