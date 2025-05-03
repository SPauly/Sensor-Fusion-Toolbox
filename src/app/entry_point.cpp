#include "app/sensor_sim.h"

#ifndef _WIN32
int main(int argc, char **argv) {
  sensfus::app::SensorSim app;
  app.Run();
  return 0;
}
#elif _WIN32
#include <windows.h>

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {
  sensfus::app::SensorSim app;
  app.Run();
  return 0;
}

#endif  // _WIN32