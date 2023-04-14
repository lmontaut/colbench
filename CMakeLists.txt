# Copyright (c) 2023 INRIA
# Author: Louis Montaut
cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME colbench)
set(PROJECT_DESCRIPTION "C++ benchmarks for collision detection")
set(PROJECT_URL "https://github.com/lmontaut/colbench")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
set(CMAKE_CXX_STANDARD 17)

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# cmake files
include(${CMAKE_CURRENT_LIST_DIR}/cmake/base.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/ide.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/apple.cmake)
include(CMakeDependentOption)
compute_project_args(PROJECT_ARG LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# set CXX standard
if(DEFINED CMAKE_CXX_STANDARD)
  check_minimal_cxx_standard(17 ENFORCE)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

# If needed, set CMake policy for APPLE systems
apply_default_apple_configuration()

# Python bindings options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
CMAKE_DEPENDENT_OPTION(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" OFF BUILD_PYTHON_INTERFACE OFF)

# Dependencies
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(hpp-fcl REQUIRED)
add_project_dependency(fmt REQUIRED)

SET_BOOST_DEFAULT_OPTIONS()
EXPORT_BOOST_DEFAULT_OPTIONS()
IF(WIN32)
  add_project_dependency(Boost REQUIRED COMPONENTS chrono thread date_time serialization program_options)
ELSE(WIN32)
  add_project_dependency(Boost REQUIRED chrono serialization program_options)
ENDIF(WIN32)

# Build main library
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS ${PROJECT_SOURCE_DIR}/include/colbench/*.hpp)
add_library(${PROJECT_NAME} INTERFACE)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  INTERFACE Eigen3::Eigen
  INTERFACE pinocchio::pinocchio
  INTERFACE hpp-fcl::hpp-fcl
  INTERFACE fmt::fmt
)

target_include_directories(
  ${PROJECT_NAME} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
                      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
set(EXPORTED_TARGETS_LIST ${PROJECT_NAME})
add_header_group(${PROJECT_NAME}_HEADERS)

# Config files to know where things are
# Used by both testing and benchmarks
config_files(benchmarks/config.h)

add_subdirectory(benchmarks)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(BUILD_PYTHON_INTERFACE)
  set(PYTHON_COMPONENTS Interpreter Development.Module)
  findpython()
  set(PY_NAME py${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})

  find_package(eigenpy 2.7.14 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)

  add_subdirectory(bindings)
endif()

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
