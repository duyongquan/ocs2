cmake_minimum_required(VERSION 3.8)
project(ocs2_hpipm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(ocs2_blasfeo REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# include(FetchContent)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Define directories
# set(HPIPM_DEVEL_PREFIX ${PROJECT_SOURCE_DIR} CACHE STRING "HPIPM install path")
# set(HPIPM_INCLUDE_DIR ${HPIPM_DEVEL_PREFIX}/include)
# set(HPIPM_LIB_DIR ${HPIPM_DEVEL_PREFIX}/lib)
# set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
# set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# # Create directories if they do not exist
# file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
# file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})
# file(MAKE_DIRECTORY ${HPIPM_DOWNLOAD_DIR})
# file(MAKE_DIRECTORY ${HPIPM_BUILD_DIR})

# message(STATUS "BLASFEO_PATH: " ${BLASFEO_PATH})

# # HPIPM Settings
# set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
# set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
# set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
# set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# # Download & build source
# FetchContent_Declare(hpipmDownload
#   GIT_REPOSITORY https://github.com/giaf/hpipm
#   GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
#   UPDATE_COMMAND ""
#   SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
#   BINARY_DIR ${HPIPM_BUILD_DIR}
#   BUILD_COMMAND $(MAKE)
#   INSTALL_COMMAND "$(MAKE) install"
#   )
# FetchContent_MakeAvailable(hpipmDownload)

# # Copy header to where catkin expects them
# file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")
# foreach(HEADER_FILE ${HEADERS})
#   message(STATUS "FOUND HEADER: " ${HEADER_FILE})
#   file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
# endforeach()

# # Install the library where catkin expects them
# set_target_properties(hpipm PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${HPIPM_LIB_DIR})

include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  ocs2_core
  ocs2_qp_solver
  ocs2_blasfeo
)

# Hpipm interface
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
