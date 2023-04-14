#
# Copyright (c) 2023 INRIA
# Author: Louis Montaut
#

macro(add_benchmark benchmark_name source)
  add_executable(${benchmark_name} ${source})
  target_link_libraries(${benchmark_name}
    PUBLIC
    ${PROJECT_NAME}
    Boost::program_options
  )
  IF(NOT WIN32)
    target_compile_options(${benchmark_name} PRIVATE "-Wno-c99-extensions")
  ENDIF(NOT WIN32)
endmacro(add_benchmark)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_subdirectory(ycb)
add_subdirectory(ellipsoids)
