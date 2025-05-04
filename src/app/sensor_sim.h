#ifndef SENSOR_SIM_H
#define SENSOR_SIM_H

#include "app/application_base.h"

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

#include <GLFW/glfw3.h>  // Will drag system OpenGL headers
#include <imgui.h>

#include "app/radar_plot.h"
#include "app/utils/layerstack.h"

namespace sensfus {
namespace app {

class SensorSim : public ApplicationBase {
 public:
  explicit SensorSim();
  virtual ~SensorSim() {};

  virtual void Run() override;

 protected:
  virtual bool Init() override;
  virtual void Shutdown() override;
  virtual bool Render() override;
  virtual inline const float GetFramerate() override { return io_->Framerate; };

 private:
  void MenuBar();

  void ConfigWindow();
  void SetStyle();
  void DarkMode();
  void LightMode();
  void HelpMarker(const char *description, const char *marker = nullptr);
  void HyperLink(const char *link, const char *marker = nullptr);

 private:
  // config
  bool use_open_workspace = false;
  bool show_graph_ = false;
  bool use_dark_mode = false;

  // internal use
  GLFWwindow *window_;
  ImGuiViewport *viewport_;
  ImGuiIO *io_;
  bool submitting_feedback_ = false;

  // Appearence
  const int display_w_ = 445;
  const int display_h_ = 650;
  const int display_w_offset_graph_ = 445;
  int temp_display_w_, temp_display_h_;  // for temporary use
  const ImGuiWindowFlags closed_workspace_flags_ = ImGuiWindowFlags_NoCollapse |
                                                   ImGuiWindowFlags_NoMove |
                                                   ImGuiWindowFlags_NoResize;
  const ImGuiWindowFlags open_workspace_flags_ =
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
  const float rounding_ = 2.0;

  // Style
  ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  ImVec4 *colors_ = nullptr;
  ImGuiStyle *style_ = nullptr;

  utils::LayerStack layer_stack_;
};

}  // namespace app
}  // namespace sensfus

#endif  // SENSOR_SIM_H