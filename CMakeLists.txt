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
    GIT_TAG 3.4.0 # Replace with the desired Eigen version
)

FetchContent_MakeAvailable(Eigen)

#----------------------------------#
# Create libraries
#----------------------------------#


#---------------------------------#
# Setup Tests
#---------------------------------#

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
  target_link_libraries(${target} GTest::gtest_main Eigen3::Eigen) 
  gtest_discover_tests(${target})
endforeach()