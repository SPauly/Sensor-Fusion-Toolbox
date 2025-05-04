#include "app/sensor_sim.h"

#include <iostream>
#include <string>
#ifdef _WIN32
#include <Windows.h>
#endif

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <implot.h>

namespace sensfus {
namespace app {
SensorSim::SensorSim() : ApplicationBase() {}

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void SensorSim::Run() {
  // Init the graphics application
  if (!Init()) {
    std::cerr << "Failed to initialize the application." << std::endl;
    return;
  }

  // Main loop
  while (Render()) {
    // Handle events here
  }

  Shutdown();
}

bool SensorSim::Init() {
  // Begin: ImGui Window Init

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) return false;

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
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
#endif

  // Create window with graphics context
  window_ =
      glfwCreateWindow(display_w_, display_h_, "Sensor Simulator", NULL, NULL);
  if (window_ == NULL) return false;
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // Enable vsync

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
  // io_->ConfigViewportsNoAutoMerge = true;
  // io_->ConfigViewportsNoTaskBarIcon = true;

  // Set the style
  ConfigWindow();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // End ImGui Window Init

  // Setup the ImPlot context
  ImPlot::CreateContext();

  viewport_ = ImGui::GetMainViewport();

  // Init the necessary layers
  layer_stack_.PushLayer<RadarPlot>();
  radar_sim_ = std::make_shared<sim::RadarSim>();
  layer_stack_.PushLayer(std::make_shared<TrajectoryPlaner>(radar_sim_));

  return true;
}

void SensorSim::Shutdown() {
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

  for (auto &layer : layer_stack_) {
    layer->OnUIRender();
  }

  ImGui::ShowDemoWindow();
  ImPlot::ShowDemoWindow();

  MenuBar();

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

  return true;
}

void SensorSim::MenuBar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("Menu")) {
      ImGui::MenuItem("Add Project", "STRG + P");
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Toggle Dark/Light mode [not impl]", "",
                          &use_dark_mode))
        SetStyle();
      if (ImGui::MenuItem("Show Graph [beta]", "STRG + G", &show_graph_)) {
        if (show_graph_) {
          glfwSetWindowSize(window_, display_w_ + display_w_offset_graph_,
                            display_h_);
          // layer_stack_.ShowLayer(graph_);
        } else {
          glfwSetWindowSize(window_, display_w_, display_h_);
          // layer_stack_.HideLayer(graph_);
        }
      }

      if (ImGui::MenuItem("Enable open workspace [beta]", "STRG+O",
                          &use_open_workspace)) {
        // Set the open_workspace flags in the graph layer
      }
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