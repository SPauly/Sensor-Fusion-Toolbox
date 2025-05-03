cmake_minimum_required(VERSION 3.5.1)

link_directories("${PROJECT_BINARY_DIR}/lib/frontend/imgui_glfw")    

#packages
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# Configure GLFW
set( GLFW_BUILD_DOCS OFF CACHE BOOL "")
set( GLFW_INSTALL OFF CACHE BOOL "")
set( GLFW_BUILD_TESTS OFF CACHE BOOL "")
set( GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw)

add_library(imgui
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_demo.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_draw.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_tables.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui_widgets.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_opengl3.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends/imgui_impl_glfw.cpp"
)

target_include_directories(imgui PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/include"
)

add_library( implot 
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot/implot.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot/implot_items.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot/implot_demo.cpp"
)

target_include_directories( implot PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot/"
)

# Add the dependencies
add_dependencies(imgui glfw)
add_dependencies(implot imgui)

target_link_libraries(implot imgui)

# set the libraries for linking
set(TARGET_LIBRARIES 
    implot    
    imgui
    glfw 
    ${GLFW_LIBRARIES}
    OpenGL::GL
)