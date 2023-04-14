//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#define BOOST_TEST_MODULE COLBENCH_SERIALIZATION

#include <boost/test/included/unit_test.hpp>
#include <boost/archive/tmpdir.hpp>
#include "colbench/serialization.hpp"
#include "benchmarks/config.h"
#include "colbench/utility.hpp"

using colbench::CollisionProblem;

void check_same_col_prob(const CollisionProblem& prob1, const CollisionProblem& prob2){
  BOOST_CHECK(prob1.pair_id == prob2.pair_id);
  BOOST_CHECK(prob1.id_shape1 == prob2.id_shape1);
  BOOST_CHECK(prob1.id_shape2 == prob2.id_shape2);
  BOOST_CHECK(prob1.id_pose == prob2.id_pose);
  BOOST_CHECK(std::abs((prob1.M1.rotation() - prob2.M1.rotation()).maxCoeff()) < 1e-16);
  BOOST_CHECK(std::abs((prob1.M1.translation() - prob2.M1.translation()).maxCoeff()) < 1e-16);
  BOOST_CHECK(std::abs((prob1.M2.rotation() - prob2.M2.rotation()).maxCoeff()) < 1e-16);
  BOOST_CHECK(std::abs((prob1.M2.translation() - prob2.M2.translation()).maxCoeff()) < 1e-16);
  BOOST_CHECK((prob1.unscaled_translation - prob2.unscaled_translation).norm() < 1e-16);
  BOOST_CHECK((prob1.unscaled_separation_vector - prob2.unscaled_separation_vector).norm() < 1e-16);
  BOOST_CHECK((prob1.normalized_separation_vector - prob2.normalized_separation_vector).norm() < 1e-16);
  BOOST_CHECK(std::abs(prob1.unscaled_dist - prob2.unscaled_dist) < 1e-16);
}

BOOST_AUTO_TEST_CASE(test_serialize_collision_problem){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";
  CollisionProblem prob1;
  colbench::storeSerialize(prob1, filename);

  CollisionProblem prob2;
  colbench::loadSerialize(prob2, filename);
  check_same_col_prob(prob1, prob2);
}

BOOST_AUTO_TEST_CASE(test_serialize_collision_problem_vector){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";
  std::vector<colbench::CollisionProblem> vec_prob1;
  for (size_t i = 0; i < 100; i++) {
    colbench::CollisionProblem tmp_prob;
    tmp_prob.pair_id = i;
    tmp_prob.M1.setRandom();
    tmp_prob.M2.setRandom();
    tmp_prob.unscaled_separation_vector.setRandom();
    tmp_prob.normalized_separation_vector.setRandom();
    tmp_prob.unscaled_dist = 3.0 * double(i);
    vec_prob1.push_back(tmp_prob);
  }
  colbench::storeSerialize(vec_prob1, filename);
  std::vector<colbench::CollisionProblem> vec_prob2;
  colbench::loadSerialize(vec_prob2, filename);
  BOOST_CHECK(vec_prob1.size() == 100);
  BOOST_CHECK(vec_prob2.size() == vec_prob1.size());
  for (size_t i = 0; i < vec_prob2.size(); i++) {
    check_same_col_prob(vec_prob1[i], vec_prob2[i]);
  }
}

BOOST_AUTO_TEST_CASE(test_serialize_collision_problem_vector_vector){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";
  std::vector<std::vector<colbench::CollisionProblem>> vec_vec_prob1;
  for (size_t i = 0; i < 100; i++) {
    std::vector<colbench::CollisionProblem> vec_prob1;
    for (size_t j = 0; j < 50; j++) {
      colbench::CollisionProblem tmp_prob;
      tmp_prob.pair_id = i + j;
      tmp_prob.M1.setRandom();
      tmp_prob.M2.setRandom();
      tmp_prob.unscaled_separation_vector.setRandom();
      tmp_prob.normalized_separation_vector.setRandom();
      tmp_prob.unscaled_dist = 3.0 * double(i);
      vec_prob1.push_back(tmp_prob);
    }
    vec_vec_prob1.push_back(vec_prob1);
  }
  colbench::storeSerialize(vec_vec_prob1, filename);
  std::vector<std::vector<colbench::CollisionProblem>> vec_vec_prob2;
  colbench::loadSerialize(vec_vec_prob2, filename);
  BOOST_CHECK(vec_vec_prob1.size() == 100);
  BOOST_CHECK(vec_vec_prob2.size() == vec_vec_prob1.size());
  for (size_t i = 0; i < vec_vec_prob2.size(); i++) {
    BOOST_CHECK(vec_vec_prob1[i].size() == 50);
    BOOST_CHECK(vec_vec_prob2[i].size() == vec_vec_prob1[i].size());
    for (size_t j = 0; j < vec_vec_prob2[i].size(); j++) {
      check_same_col_prob(vec_vec_prob1[i][j], vec_vec_prob2[i][j]);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_SE3_serialization){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";

  // Serialize a SE3
  SE3 M1 = SE3::Random();
  colbench::storeSerialize(M1, filename);
  SE3 M2;
  colbench::loadSerialize(M2, filename);
  BOOST_CHECK((M1.translation() - M2.translation()).norm() < 1e-16);
  double max_norm_rot = std::abs((M1.rotation() - M2.rotation()).maxCoeff());
  BOOST_CHECK(max_norm_rot < 1e-16);

  // Serialize a vector of SE3
  std::vector<SE3> vec_M1;
  for (size_t i = 0; i < 1000; i++) {
    M1 = SE3::Random();
    vec_M1.push_back(M1);
  }
  colbench::storeSerialize(vec_M1, filename);
  std::vector<SE3> vec_M2;
  colbench::loadSerialize(vec_M2, filename);
  BOOST_CHECK(vec_M1.size() == 1000);
  BOOST_CHECK(vec_M2.size() == vec_M1.size());
  for (size_t i = 0; i < vec_M2.size(); i++) {
    BOOST_CHECK((vec_M1[i].translation() - vec_M2[i].translation()).norm() < 1e-16);
    double max_norm_rot = std::abs((vec_M1[i].rotation() - vec_M2[i].rotation()).maxCoeff());
    BOOST_CHECK(max_norm_rot < 1e-16);
  }
}

BOOST_AUTO_TEST_CASE(test_serialize_hpp_fcl){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";
  hpp::fcl::Box box1(1., 1., 1.);
  colbench::storeSerialize(box1, filename);
  hpp::fcl::Box box2(1., 1., 1.);
  colbench::loadSerialize(box2, filename);
}

BOOST_AUTO_TEST_CASE(test_pinocchio_se3){
  SE3 M1, M2;
  M1.setRandom();
  M2 = M1;
  M1.setRandom();
  BOOST_CHECK(std::abs((M1.rotation() - M2.rotation()).maxCoeff()) > 1e-1);
  BOOST_CHECK(std::abs((M1.translation() - M2.translation()).maxCoeff()) > 1e-1);
}

BOOST_AUTO_TEST_CASE(test_hpp_fcl_convex_serialization){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";

  // First check if YCB data exists
  std::filesystem::path shape_path(std::string(COLBENCH_YCB_DIR) + "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  bool exists = std::filesystem::exists(shape_path);
  if(!exists){
    std::cout << std::string(shape_path) + " does not exist.\
              \nMake sure you have run `python benchmarks/ycb/ycb_download.py` and `python benchmarks/ycb/ycb_to_csv.py` in the root repository directory."
    << std::endl;
  } else {
    std::shared_ptr<ConvexBase> shape = colbench::loadConvexMesh(shape_path);
    {
      colbench::storeConvexHull(shape, filename);
    }

    BVHModel<AABB> bvh2;
    colbench::loadSerialize(bvh2, filename);
    bvh2.buildConvexRepresentation(false);

    BOOST_CHECK(shape->num_points == bvh2.convex->num_points);
    Convex<Triangle>* convex1 = static_cast<Convex<Triangle>*>(shape.get());
    Convex<Triangle>* convex2 = static_cast<Convex<Triangle>*>(bvh2.convex.get());

    for (size_t i = 0; i < convex2->num_points; i++) {
      BOOST_CHECK((convex1->points[i] - convex2->points[i]).norm() < 1e-16);
    }

    for (size_t i = 0; i < convex2->num_polygons; i++) {
      BOOST_CHECK(convex1->polygons[0] == convex2->polygons[0]);
      BOOST_CHECK(convex1->polygons[1] == convex2->polygons[1]);
      BOOST_CHECK(convex1->polygons[2] == convex2->polygons[2]);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_hpp_fcl_convex_serialization_2){
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string filename = tmp_dir + "file.bin";

  // First check if YCB data exists
  std::filesystem::path shape_path1(std::string(COLBENCH_YCB_DIR) + "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  bool exists1 = std::filesystem::exists(shape_path1);
  std::filesystem::path shape_path2(std::string(COLBENCH_YCB_DIR) + "/data/ycb_data/002_master_chef_can/google_16k/nontextured.stl");
  bool exists2 = std::filesystem::exists(shape_path2);
  std::string message_no_ycb = " does not exist.\nMake sure you have run `python benchmarks/ycb/ycb_download.py` and `python benchmarks/ycb/ycb_to_csv.py` in the root repository directory.";
  if(!exists1){
    std::cout << std::string(shape_path1) + message_no_ycb << std::endl;
  } else if (!exists2)  {
    std::cout << std::string(shape_path2) + message_no_ycb << std::endl;
  } else {
    std::vector<std::shared_ptr<ConvexBase>> shapes;
    shapes.push_back(colbench::loadConvexMesh(shape_path1));
    shapes.push_back(colbench::loadConvexMesh(shape_path2));
    {
      colbench::storeConvexHull(shapes, filename);
    }

    std::vector<BVHModel<AABB>> bvhs;
    colbench::loadSerialize(bvhs, filename);
    for (size_t i = 0; i < bvhs.size(); i++) {
      bvhs[i].buildConvexRepresentation(false);
      BOOST_CHECK(shapes[i]->num_points == bvhs[i].convex->num_points);
      Convex<Triangle>* convex1 = static_cast<Convex<Triangle>*>(shapes[i].get());
      Convex<Triangle>* convex2 = static_cast<Convex<Triangle>*>(bvhs[i].convex.get());

      for (size_t i = 0; i < convex2->num_points; i++) {
        BOOST_CHECK((convex1->points[i] - convex2->points[i]).norm() < 1e-16);
      }

      for (size_t i = 0; i < convex2->num_polygons; i++) {
        BOOST_CHECK(convex1->polygons[0] == convex2->polygons[0]);
        BOOST_CHECK(convex1->polygons[1] == convex2->polygons[1]);
        BOOST_CHECK(convex1->polygons[2] == convex2->polygons[2]);
      }
    }
  }
}
