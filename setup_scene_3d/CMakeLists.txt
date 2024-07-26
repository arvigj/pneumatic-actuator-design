cmake_minimum_required(VERSION 3.1)

project(Setup3D DESCRIPTION "Setup scene in 3D")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)
include(nanospline)
include(json)
include(cli11)

# Enable the target igl::glfw
igl_include(glfw)

# Add your project files
file(GLOB SRC_FILES *.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)

target_link_libraries(${PROJECT_NAME} PUBLIC nanospline)

target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)

target_link_libraries(${PROJECT_NAME} PUBLIC CLI11::CLI11)
