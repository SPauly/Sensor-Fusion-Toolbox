cmake_minimum_required(VERSION 3.14)
project(Sensor-Fusion-Toolbox)


#---------------------------------#
# Setup environment
#---------------------------------#

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Enable FetchContent module
include(FetchContent)

# Download and make Eigen available
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0 
)

FetchContent_MakeAvailable(Eigen)

#----------------------------------#
# Create senssimlib library
#----------------------------------#

set(SENSSIMLIB_SOURCES
  src/sensor_simulator.cpp
  src/sensor_radar.cpp
  # Add other core simulation sources here if needed
)

add_library(senssimlib STATIC ${SENSSIMLIB_SOURCES})

target_include_directories(senssimlib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

target_link_libraries(senssimlib PUBLIC Eigen3::Eigen)

#----------------------------------#
# Build the executables
#----------------------------------#

# Add the gui dependencies
include(cmake/target_imgui_implot_glfw.cmake)


set(SENSOR_SIM_APP_TARGETS
  src/app/entry_point.cpp
  src/app/radar_plot.cpp
  src/app/sensor_sim.cpp
  src/app/sensor_viewport.cpp
  src/app/target_plot.cpp
  src/app/trajectory_planer.cpp
  src/app/utils/layerstack.cpp
)

if(WIN32)
  add_executable(SensorSimulator WIN32
    ${SENSOR_SIM_APP_TARGETS}
  )
else()
  add_executable(SensorSimulator
    ${SENSOR_SIM_APP_TARGETS}
  )
endif()

target_include_directories(SensorSimulator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

target_link_libraries(SensorSimulator PRIVATE
   senssimlib
   ${TARGET_LIBRARIES}
   Eigen3::Eigen
)

#---------------------------------#
# Setup Tests
#---------------------------------#

if(BUILD_TESTS)

# Fetch google test
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# Workaround for Windows:
# Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# GoogleTest soll nur lokal, in diesem Projekt verwendet werden.
option(INSTALL_GTEST "Enable installation of googletest." OFF)

# Include CMake configuration of GoogleTest
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Enable GoogleTest
enable_testing()


# List with all tests to be created
set(UTILS_TESTS
  rand_generator_test
  timers_test
  types_test
  trajectory_test
  eventbus_test
)

set(SINGLE_FILE_TESTS
  ${UTILS_TESTS}
)

# Erzeuge für jeden Test ein eigenes Target
foreach(target ${SINGLE_FILE_TESTS})
  add_executable(${target} ${CMAKE_CURRENT_SOURCE_DIR}/test/${target}.cpp)
  target_include_directories(${target} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  target_link_libraries(${target} GTest::gtest_main Eigen3::Eigen senssimlib) 
  gtest_discover_tests(${target})
endforeach()

endif() # BUILD_TESTS