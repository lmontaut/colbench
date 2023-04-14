#
# Copyright (c) 2023 INRIA
# Author: Louis Montaut
#

FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework filesystem chrono timer)
FIND_PACKAGE(proxsuite REQUIRED)

macro(add_colbench_test test_name source)
  ADD_UNIT_TEST(${test_name} ${source})
  target_link_libraries(${test_name}
    PUBLIC
    ${PROJECT_NAME}
    Boost::chrono
    Boost::filesystem
    Boost::timer
    proxsuite::proxsuite
  )
  add_custom_target(
    run_${test_name}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${test_name}
    VERBATIM
  )
  IF(NOT WIN32)
    target_compile_options(${test_name} PRIVATE "-Wno-c99-extensions")
  ENDIF(NOT WIN32)
endmacro(add_colbench_test)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_colbench_test(test_gjk test_gjk.cpp)
add_colbench_test(test_serialization test_serialization.cpp)
add_colbench_test(test_csv test_csv.cpp)
add_colbench_test(test_proxsuite test_proxsuite.cpp)
add_colbench_test(test_collision_problem test_collision_problem.cpp)
