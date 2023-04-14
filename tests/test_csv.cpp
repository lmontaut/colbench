//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#define BOOST_TEST_MODULE COLBENCH_CSV

#include <iostream>
#include <boost/test/included/unit_test.hpp>
#include "colbench/csv.hpp"
#include "benchmarks/config.h"

BOOST_AUTO_TEST_CASE(test_csv_read){
  io::CSVReader<3> file(std::string(COLBENCH_YCB_DIR) + "/data/ycb.csv");
  file.read_header(io::ignore_no_column, "shape_path", "num_vertices", "num_faces");
  std::string shape_path; int num_vertices, num_faces;
  int rows_read = 0;
  while(file.read_row(shape_path, num_vertices, num_faces)){
    // std::cout << "PATH: " << shape_path << std::endl;
    // std::cout << "    Num vertices: " << num_vertices << std::endl;
    // std::cout << "    Num faces: " << num_faces << std::endl;
    ++rows_read;
  }
  BOOST_CHECK(rows_read > 0);
}
