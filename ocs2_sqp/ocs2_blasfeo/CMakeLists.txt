cmake_minimum_required(VERSION 3.8)
project(ocs2_blasfeo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
include(FetchContent)


# Define directories
set(BLASFEO_DEVEL_PREFIX ${PROJECT_SOURCE_DIR} CACHE STRING "Blasfeo install path")
set(BLASFEO_INCLUDE_DIR ${BLASFEO_DEVEL_PREFIX}/include)
set(BLASFEO_LIB_DIR ${BLASFEO_DEVEL_PREFIX}/lib)
set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})
file(MAKE_DIRECTORY ${BLASFEO_LIB_DIR})
file(MAKE_DIRECTORY ${BLASFEO_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${BLASFEO_BUILD_DIR})

# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(blasfeoDownload
	GIT_REPOSITORY https://github.com/giaf/blasfeo
	GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
	UPDATE_COMMAND ""
	SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
	BINARY_DIR ${BLASFEO_BUILD_DIR}
	BUILD_COMMAND $(MAKE)
	INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(blasfeoDownload)

# Copy header to where catkin expects them
file(GLOB_RECURSE HEADERS "${BLASFEO_DOWNLOAD_DIR}/include/*.h")
foreach(HEADER_FILE ${HEADERS})
	message(STATUS "FOUND HEADER: " ${HEADER_FILE})
	file(COPY ${HEADER_FILE} DESTINATION ${BLASFEO_INCLUDE_DIR})
endforeach()

# Install the library where catkin expects them
set_target_properties(blasfeo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BLASFEO_LIB_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
