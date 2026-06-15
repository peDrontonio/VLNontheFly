cmake_minimum_required(VERSION 3.8)
project(edgellm_vlm_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:-Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

option(EDGELLM_VLM_ENABLE_EDGELLM "Build the TensorRT Edge-LLM backend" OFF)
set(EDGELLM_SOURCE_DIR "" CACHE PATH "Path to TensorRT-Edge-LLM source tree")
set(EDGELLM_BUILD_DIR "" CACHE PATH "Path to TensorRT-Edge-LLM build tree")
set(TRT_PACKAGE_DIR "" CACHE PATH "Path to TensorRT package")

add_executable(edgellm_vlm_node
  src/main.cpp
  src/vlm_node.cpp
)

target_include_directories(edgellm_vlm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(edgellm_vlm_node PUBLIC cxx_std_17)

if(EDGELLM_VLM_ENABLE_EDGELLM)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  target_compile_definitions(edgellm_vlm_node PRIVATE EDGELLM_VLM_ENABLE_EDGELLM=1)
  target_sources(edgellm_vlm_node PRIVATE src/cuda_link_stub.cu)
  set_target_properties(edgellm_vlm_node PROPERTIES
    CUDA_ARCHITECTURES 87
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )
  if(NOT EDGELLM_SOURCE_DIR OR NOT EDGELLM_BUILD_DIR OR NOT TRT_PACKAGE_DIR)
    message(FATAL_ERROR "EDGELLM_SOURCE_DIR, EDGELLM_BUILD_DIR, and TRT_PACKAGE_DIR are required when EDGELLM_VLM_ENABLE_EDGELLM=ON")
  endif()
  find_library(NVINFER_LIBRARY nvinfer HINTS ${TRT_PACKAGE_DIR}/lib /usr/lib/aarch64-linux-gnu REQUIRED)
  find_library(NVONNXPARSER_LIBRARY nvonnxparser HINTS ${TRT_PACKAGE_DIR}/lib /usr/lib/aarch64-linux-gnu REQUIRED)
  find_library(CUDA_DRIVER_LIBRARY cuda HINTS /usr/local/cuda/targets/aarch64-linux/lib/stubs REQUIRED)
  find_library(CUDA_DEVRT_LIBRARY cudadevrt HINTS /usr/local/cuda/targets/aarch64-linux/lib REQUIRED)
  find_library(CUDA_RUNTIME_STATIC_LIBRARY cudart_static HINTS /usr/local/cuda/targets/aarch64-linux/lib REQUIRED)

  set(EDGELLM_CORE_LIBRARY ${EDGELLM_BUILD_DIR}/cpp/libedgellmCore.a)
  set(EDGELLM_CUTEDSL_SHIM_LIBRARY ${EDGELLM_BUILD_DIR}/cpp/libtrt_edgellm_cutedsl_cudart_shim.a)
  set(EDGELLM_CUTEDSL_LIBRARY ${EDGELLM_SOURCE_DIR}/cpp/kernels/cuteDSLArtifact/aarch64/sm_87/libcutedsl_aarch64.a)
  foreach(required_file IN ITEMS
      ${EDGELLM_CORE_LIBRARY}
      ${EDGELLM_CUTEDSL_SHIM_LIBRARY}
      ${EDGELLM_CUTEDSL_LIBRARY})
    if(NOT EXISTS ${required_file})
      message(FATAL_ERROR "Required Edge-LLM library not found: ${required_file}")
    endif()
  endforeach()

  target_include_directories(edgellm_vlm_node PRIVATE
    ${EDGELLM_SOURCE_DIR}/cpp
    ${EDGELLM_SOURCE_DIR}/examples/multimodal
    ${EDGELLM_SOURCE_DIR}/3rdParty/nlohmannJson/include
    ${EDGELLM_SOURCE_DIR}/3rdParty/stb
    ${TRT_PACKAGE_DIR}/include
  )
  target_link_directories(edgellm_vlm_node PRIVATE
    /usr/local/cuda/targets/aarch64-linux/lib
    /usr/local/cuda/targets/aarch64-linux/lib/stubs
  )
  target_link_options(edgellm_vlm_node PRIVATE
    -Wl,--wrap=_cudaLaunchKernelEx
    -Wl,--unresolved-symbols=ignore-in-shared-libs
  )
  target_link_libraries(edgellm_vlm_node
    ${EDGELLM_CORE_LIBRARY}
    ${EDGELLM_CUTEDSL_LIBRARY}
    ${EDGELLM_CUTEDSL_SHIM_LIBRARY}
    ${CUDA_DRIVER_LIBRARY}
    ${NVONNXPARSER_LIBRARY}
    ${NVINFER_LIBRARY}
    CUDA::cudart
    ${CUDA_DEVRT_LIBRARY}
    ${CUDA_RUNTIME_STATIC_LIBRARY}
    rt
    pthread
    dl
  )
endif()

ament_target_dependencies(
  edgellm_vlm_node
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(edgellm_vlm_node ${OpenCV_LIBS})

install(TARGETS edgellm_vlm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vlm_primitive_gate.py
  scripts/vlm_point_gate.py
  scripts/vlm_region_gate.py
  scripts/vlm_nav_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_vlm_primitive_gate
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vlm_primitive_gate.py
  )
  add_test(
    NAME test_vlm_point_gate
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vlm_point_gate.py
  )
  add_test(
    NAME test_vlm_region_gate
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vlm_region_gate.py
  )
  add_test(
    NAME test_vlm_nav_supervisor
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vlm_nav_supervisor.py
  )
endif()

ament_package()
