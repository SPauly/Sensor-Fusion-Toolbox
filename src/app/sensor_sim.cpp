#include "app/sensor_sim.h"

#include <iostream>
#include <string>
#ifdef _WIN32
#include <Windows.h>
#endif

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>  // Will drag system OpenGL headers

#include "sensfus/types.h"
#include "sensfus/kalman/kalman_filter.h"

namespace sensfus {
namespace app {
SensorSim::SensorSim()
    : ApplicationBase(),
      sim_(std::make_shared<sim::SensorSimulator>()),
      target_plot_(TargetPlot(sim_)) {
  sensfus::kalman::KalmanFilter<sensfus::ObjectState2D> kf;
}

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

bool SensorSim::Run() {
  // Init the graphics application
  if (!Init()) {
    std::cerr << "Failed to initialize the application." << std::endl;
    return false;
  }

  // Main loop
  bool ret = true;
  while (ret = Render()) {
    // Handle events here
  }

  Shutdown();

  if (respawn_flag_)
    return true;
  else
    return false;
}

bool SensorSim::Init() {
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) return false;
#ifdef _WIN32
  // Make process DPI aware (Windows 10+)
  SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
#endif

// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  const char *glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#else
  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

#endif

  // Create window with graphics context
  window_ =
      glfwCreateWindow(display_w_, display_h_, "Sensor Simulator", NULL, NULL);
  if (window_ == NULL) return false;
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // Enable vsync

  // --- DPI scaling logic ---
  float xscale = 1.0f, yscale = 1.0f;
  glfwGetWindowContentScale(window_, &xscale, &yscale);
  xscale = (xscale > 1.5f) ? 1.5f : xscale;  // Limit scaling to 1.5x
  yscale = (yscale > 1.5f) ? 1.5f : yscale;  // Limit scaling to 1.5x
  float dpi_scale = (xscale > 1.0f) ? xscale : 1.0f;

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  io_ = &ImGui::GetIO();
  (void)io_;
  io_->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io_->ConfigFlags |= ImGuiConfigFlags_DockingEnable;  // Enable Docking
  io_->ConfigFlags |=
      ImGuiConfigFlags_ViewportsEnable;  // Enable Multi-Viewport / Platform
                                         // Windows
  io_->BackendFlags |=
      ImGuiBackendFlags_RendererHasVtxOffset;  // Enable more than 64k vertices
  // io_->ConfigViewportsNoAutoMerge = true;
  // io_->ConfigViewportsNoTaskBarIcon = true;

  // Set the style
  ConfigWindow();

  // Scale ImGui style
  ImGuiStyle &style = ImGui::GetStyle();
  style.ScaleAllSizes(dpi_scale);

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Scale fonts
  io_->Fonts->Clear();
  io_->Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf",
                                 16.0f * dpi_scale);
  io_->FontGlobalScale =
      1.0f;  // Don't use FontGlobalScale if you scale font size

  // End ImGui Window Init

  // Setup the ImPlot context
  ImPlot::CreateContext();

  // --- ImPlot DPI scaling ---
  ImPlotStyle &plotStyle = ImPlot::GetStyle();
  plotStyle.LineWeight *= dpi_scale;
  plotStyle.MarkerSize *= dpi_scale;
  plotStyle.MarkerWeight *= dpi_scale;
  plotStyle.FillAlpha *= 1.0f;  // usually not needed
  plotStyle.ErrorBarSize *= dpi_scale;
  plotStyle.ErrorBarWeight *= dpi_scale;
  plotStyle.DigitalBitHeight *= dpi_scale;
  plotStyle.DigitalBitGap *= dpi_scale;
  plotStyle.PlotBorderSize *= dpi_scale;
  plotStyle.MinorAlpha *= 1.0f;
  plotStyle.MajorTickLen.x *= dpi_scale;
  plotStyle.MajorTickLen.y *= dpi_scale;
  plotStyle.MinorTickLen.x *= dpi_scale;
  plotStyle.MinorTickLen.y *= dpi_scale;
  plotStyle.MajorTickSize.x *= dpi_scale;
  plotStyle.MajorTickSize.y *= dpi_scale;
  plotStyle.MinorTickSize.x *= dpi_scale;
  plotStyle.MinorTickSize.y *= dpi_scale;
  plotStyle.MajorGridSize.x *= dpi_scale;
  plotStyle.MajorGridSize.y *= dpi_scale;
  plotStyle.MinorGridSize.x *= dpi_scale;
  plotStyle.MinorGridSize.y *= dpi_scale;
  plotStyle.PlotPadding.x *= dpi_scale;
  plotStyle.PlotPadding.y *= dpi_scale;
  plotStyle.LabelPadding.x *= dpi_scale;
  plotStyle.LabelPadding.y *= dpi_scale;
  plotStyle.LegendPadding.x *= dpi_scale;
  plotStyle.LegendPadding.y *= dpi_scale;
  plotStyle.LegendInnerPadding.x *= dpi_scale;
  plotStyle.LegendInnerPadding.y *= dpi_scale;
  plotStyle.LegendSpacing.x *= dpi_scale;
  plotStyle.LegendSpacing.y *= dpi_scale;
  plotStyle.MousePosPadding.x *= dpi_scale;
  plotStyle.MousePosPadding.y *= dpi_scale;
  plotStyle.AnnotationPadding.x *= dpi_scale;
  plotStyle.AnnotationPadding.y *= dpi_scale;
  plotStyle.FitPadding.x *= dpi_scale;
  plotStyle.FitPadding.y *= dpi_scale;
  plotStyle.PlotDefaultSize.x *= dpi_scale;
  plotStyle.PlotDefaultSize.y *= dpi_scale;
  plotStyle.PlotMinSize.x *= dpi_scale;
  plotStyle.PlotMinSize.y *= dpi_scale;

  viewport_ = ImGui::GetMainViewport();

  // Init the necessary layers
  sim_ = std::make_shared<sim::SensorSimulator>();

  sensor_viewport_ = std::make_shared<SensorViewport>();
  sensor_viewport_->RegisterPlotCallback("Target Plot",
                                         target_plot_.GetCallback());

  layer_stack_.PushLayer(sensor_viewport_);
  layer_stack_.PushLayer(std::make_shared<TrajectoryPlaner>(sim_));

  // Setup the event bus communication
  event_bus_ = sim_->GetEventBus();
  target_sub_ = event_bus_->Subscribe<TrueTargetState2D>("TrueTargetState2D");
  radar_sub_ = event_bus_->Subscribe<RadarSensorInfo2D>("RadarSensorInfo2D");

  // Setup Kalman filter layer
  kalman_ = std::make_shared<KalmanSim>(0, sim_);
  layer_stack_.PushLayer(kalman_);  // ID is not used in this case
  sensor_viewport_->RegisterPlotCallback(
      "Kalman Filter Plot",
      std::bind(&KalmanSim::RunPlotCallback, kalman_.get()));

  // Setup default start state
  // sim_->StartSimulation();
  radar_sensors_.push_back(sim_->AddRadarSensor());
  radar_sensors_.back()->SetSensorPosition(ObjectPosition2D(0.0, 0.0));

  radar_plots_.push_back(std::make_shared<RadarPlot>(
      radar_sensors_.back()->GetId(), sim_, radar_sensors_.back()));

  // Register the necessary gui callbacks
  std::string tmp =
      "Radar Sensor " + std::to_string(radar_sensors_.back()->GetId());
  sensor_viewport_->RegisterPlotCallback(tmp,
                                         radar_plots_.back()->GetCallback());

  return true;
}

void SensorSim::Shutdown() {
  sim_->StartSimulation();

  // Shutdown layers
  layer_stack_.clear();
  kalman_.reset();
  sim_.reset();

  // Destroy the ImPlot context
  ImPlot::DestroyContext();

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window_);
  glfwTerminate();
}

bool SensorSim::Render() {
  if (glfwWindowShouldClose(window_)) return false;

  glfwPollEvents();
  if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != 0) {
    ImGui_ImplGlfw_Sleep(10);
    return true;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // We need a dockspace for our app layout
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  // Handle the simulation data before rendering
  HandleSimulationData();

  for (auto &layer : layer_stack_) {
    layer->OnUIRender();
  }

  ImGui::ShowDemoWindow();
  ImPlot::ShowDemoWindow();

  MenuBar();

  if (adding_sensor_) {
    AddSensor();
  }

  SensorControl();

  // Rendering
  ImGui::Render();
  glfwGetFramebufferSize(window_, &temp_display_w_, &temp_display_h_);
  glViewport(0, 0, temp_display_w_, temp_display_h_);
  glClearColor(clear_color_.x * clear_color_.w, clear_color_.y * clear_color_.w,
               clear_color_.z * clear_color_.w, clear_color_.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  if (io_->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    GLFWwindow *backup_current_context = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backup_current_context);
  }

  glfwSwapBuffers(window_);

  if (respawn_flag_) return false;

  return true;
}

void SensorSim::HandleSimulationData() {
  // Handle the simulation data
  auto target = target_sub_->Fetch();

  while (target) {
    target_plot_.AddTargetUpdate(target);
    target = target_sub_->Fetch();
  }

  auto radar = radar_sub_->Fetch();

  while (radar) {
    // Distribute the radar data to the specific radar plot
    radar_plots_.at(radar->id)->AddSensorUpdate(radar);
    radar = radar_sub_->Fetch();
  }
}

void SensorSim::MenuBar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Add Sensor", "STRG + P")) adding_sensor_ = true;
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Toggle Dark/Light mode [not impl]", "",
                          &use_dark_mode))
        SetStyle();
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("Contact")) {
#ifdef _WIN32
        ShellExecute(nullptr, "open",
                     "https:\\\\github.com\\SPauly\\Sensor-Fusion-Toolbox",
                     nullptr, nullptr, SW_SHOWNORMAL);
#endif  // _WIN32
      }

      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}

void SensorSim::SensorControl() {
  if (ImGui::Begin("Sensor Control")) {
    ImGui::Text("Sensor Control Window");
    static float update_time_ms = 100.0f;  // Default 100 ms
    static float update_time_prev;
    ImGui::SliderFloat("Update Time (ms)", &update_time_ms, 0.1f, 5000.0f,
                       "%.1f ms", ImGuiSliderFlags_Logarithmic);

    if (update_time_ms != update_time_prev) {
      sim_->SetUpdateRate(static_cast<uint64_t>(update_time_ms * 1e6));
      update_time_prev = update_time_ms;
    }
    HelpMarker(
        "Controls how often the simulation updates. Lower values mean "
        "faster updates.",
        "(?)");

    // Show start and stop simulation options
    if (ImGui::Button("Start Simulation")) {
      sim_->StartSimulation();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop Simulation")) {
      sim_->HaltSimulation();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Simulation")) {
      sim_->ResetSimulation();
      respawn_flag_ = true;
    }

    ImGui::Separator();

    for (auto &radar_plot : radar_plots_) {
      radar_plot->RunControllInterface();
    }

    ImGui::End();
  }
}

void SensorSim::AddSensor() {
  ImGui::OpenPopup("Adding Sensor");
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

  if (ImGui::BeginPopupModal("Adding Sensor", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Which Sensor do you want to add?");

    static float x, y, z;

    ImGui::PushItemWidth(100);
    ImGui::InputFloat("x", &x);
    ImGui::InputFloat("y", &y);
    ImGui::PopItemWidth();

    if (ImGui::Button("Create Sensor")) {
      radar_sensors_.push_back(sim_->AddRadarSensor());
      radar_sensors_.back()->SetSensorPosition(ObjectPosition2D(x, y));

      radar_plots_.push_back(std::make_shared<RadarPlot>(
          radar_sensors_.back()->GetId(), sim_, radar_sensors_.back()));

      // Register the necessary gui callbacks
      std::string tmp =
          "Radar Sensor " + std::to_string(radar_sensors_.back()->GetId());
      sensor_viewport_->RegisterPlotCallback(
          tmp, radar_plots_.back()->GetCallback());

      adding_sensor_ = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void SensorSim::HelpMarker(const char *description, const char *marker) {
  ImGui::TextDisabled((marker) ? marker : "(?)");
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(description);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void SensorSim::HyperLink(const char *link, const char *marker) {
  ImGui::TextDisabled((marker) ? marker : link);
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(link);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
  if (ImGui::IsItemClicked()) {
#ifdef _WIN32
    ShellExecute(nullptr, "open", link, nullptr, nullptr, SW_SHOWNORMAL);
#endif  // _WIN32
  }
}

void SensorSim::ConfigWindow() {
  style_ = &ImGui::GetStyle();
  colors_ = style_->Colors;

  style_->WindowTitleAlign = ImVec2(0.5f, 0.5f);
  style_->WindowMenuButtonPosition = ImGuiDir_None;
  style_->TabRounding = rounding_;
  style_->FrameRounding = rounding_;
  style_->WindowRounding = rounding_;

  SetStyle();
}

void SensorSim::SetStyle() {
  return (use_dark_mode) ? DarkMode() : LightMode();
}

void SensorSim::LightMode() {}

void SensorSim::DarkMode() {}

}  // namespace app
}  // namespace sensfus