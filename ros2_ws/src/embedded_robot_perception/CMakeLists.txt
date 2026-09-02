cmake_minimum_required(VERSION 3.16)
project(embedded_robot_perception VERSION 1.0.0 LANGUAGES CXX)

option(ENABLE_CUDA "Build the bounded CUDA pose backend" OFF)
option(ENABLE_TENSORRT "Report and validate the TensorRT ABI from the CUDA backend" OFF)
option(ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)
if(ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-fno-omit-frame-pointer -fsanitize=address,undefined)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(embedded_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

set(core_sources
  src/backend_selector.cpp
  src/calibration_manager.cpp
  src/cpu_pose_backend.cpp
  src/frame_aligner.cpp
  src/grasp_validator.cpp
)
add_library(perception_core SHARED ${core_sources})
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(ENABLE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(perception_cuda_backend SHARED src/cuda/cuda_pose_backend.cu)
  target_include_directories(perception_cuda_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(perception_cuda_backend PUBLIC perception_core CUDA::cudart)
  target_compile_definitions(perception_core PUBLIC EMBEDDED_ROBOT_HAS_CUDA_BACKEND=1)
  target_compile_definitions(perception_cuda_backend PUBLIC EMBEDDED_ROBOT_HAS_CUDA_BACKEND=1)
  set_target_properties(perception_cuda_backend PROPERTIES CUDA_STANDARD 20 CUDA_STANDARD_REQUIRED ON)
  if(ENABLE_TENSORRT)
    find_path(TENSORRT_INCLUDE_DIR NvInferVersion.h REQUIRED)
    target_include_directories(perception_cuda_backend PRIVATE ${TENSORRT_INCLUDE_DIR})
    target_compile_definitions(perception_cuda_backend PRIVATE EMBEDDED_ROBOT_HAS_TENSORRT=1)
  endif()
endif()

add_library(perception_component SHARED src/perception_node.cpp)
target_include_directories(perception_component PRIVATE include)
target_link_libraries(perception_component perception_core)
if(ENABLE_CUDA)
  target_link_libraries(perception_component perception_cuda_backend)
endif()
ament_target_dependencies(perception_component
  diagnostic_msgs embedded_robot_interfaces geometry_msgs lifecycle_msgs rclcpp rclcpp_action rclcpp_components
  rclcpp_lifecycle sensor_msgs tf2 tf2_ros vision_msgs
)
rclcpp_components_register_nodes(perception_component "embedded_robot_perception::PerceptionNode")

add_executable(perception_core_benchmark bench/core_benchmark.cpp)
target_include_directories(perception_core_benchmark PRIVATE include)
target_link_libraries(perception_core_benchmark PRIVATE perception_core)

add_executable(mock_fixture_publisher src/mock_fixture_publisher.cpp)
ament_target_dependencies(mock_fixture_publisher
  embedded_robot_interfaces geometry_msgs rclcpp sensor_msgs tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(perception_core_test test/core_test.cpp)
  target_include_directories(perception_core_test PRIVATE include)
  target_link_libraries(perception_core_test PRIVATE perception_core)
  if(ENABLE_CUDA)
    target_link_libraries(perception_core_test PRIVATE perception_cuda_backend)
  endif()
  add_test(NAME perception_core_test COMMAND perception_core_test)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS perception_core perception_component perception_core_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS mock_fixture_publisher RUNTIME DESTINATION lib/${PROJECT_NAME})
if(ENABLE_CUDA)
  install(TARGETS perception_cuda_backend LIBRARY DESTINATION lib)
endif()
ament_export_include_directories(include)
ament_export_libraries(perception_core)
ament_package()
