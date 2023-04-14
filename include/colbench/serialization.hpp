//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_SERIALIZATION_HPP
#define COLBENCH_SERIALIZATION_HPP

#include <iostream>
#include <fstream>
#include "colbench/collision_problem.hpp"
#include "colbench/fwd.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using colbench::CollisionProblem;
using pinocchio::SE3;

namespace colbench {

template<typename T>
void storeSerialize(const T& thing,
                    std::string store_path){
  std::cout << "Storing in " << store_path << std::endl;
  {
    std::ofstream ofs(store_path);
    boost::archive::text_oarchive oa(ofs);
    oa << thing;
  }
  std::cout << "... DONE.\n" << std::endl;
}

template<typename T>
void loadSerialize(T& thing,
                   std::string store_path){
  std::cout << "Loading from " << store_path << std::endl;
  {
    std::ifstream ifs(store_path);
    boost::archive::text_iarchive ia(ifs);
    ia >> thing;
  }
  std::cout << "... DONE.\n" << std::endl;
}


template<typename T>
void storeSerialize(const std::vector<T>& things,
                    std::string store_path){
  std::cout << "Storing in " << store_path << std::endl;
  {
    std::ofstream ofs(store_path);
    boost::archive::text_oarchive oa(ofs);
    oa << things;
  }
  std::cout << "... DONE.\n" << std::endl;
}


template<typename T>
void loadSerialize(std::vector<T>& things,
                   std::string store_path){
  std::cout << "Loading from " << store_path << std::endl;
  {
    std::ifstream ifs(store_path);
    boost::archive::text_iarchive ia(ifs);
    ia >> things;
  }
  std::cout << "... DONE.\n" << std::endl;
}

template<typename T>
void storeSerialize(const std::vector<std::vector<T>>& things,
                    std::string store_path){
  std::cout << "Storing in " << store_path << std::endl;
  {
    std::ofstream ofs(store_path);
    boost::archive::text_oarchive oa(ofs);
    oa << things;
  }
  std::cout << "... DONE.\n" << std::endl;
}

template<typename T>
void loadSerialize(std::vector<std::vector<T>>& things,
                   std::string store_path){
  std::cout << "Loading from " << store_path << std::endl;
  {
    std::ifstream ifs(store_path);
    boost::archive::text_iarchive ia(ifs);
    ia >> things;
  }
  std::cout << "... DONE.\n" << std::endl;
}

inline void fromConvexHullToBVH(const std::shared_ptr<ConvexBase>& shape, BVHModel<AABB>& bvh){
  // Creates a new BVH which will store the convex hull
  Convex<Triangle>* convex = static_cast<Convex<Triangle>*>(shape.get());

  std::vector<Vector3d> points(convex->num_points);
  for (size_t i = 0; i < convex->num_points; i++) {
    points[i] = convex->points[i];
  }

  std::vector<Triangle> triangles(convex->num_polygons);
  for (size_t i = 0; i < convex->num_polygons; i++) {
    triangles[i] = convex->polygons[i];
  }

  bvh.beginModel();
  bvh.addSubModel(points, triangles);
  bvh.endModel();
  bvh.buildConvexRepresentation(false);
}

inline void storeConvexHull(const std::shared_ptr<ConvexBase>& shape,
                     const std::string& filename){
  BVHModel<AABB> bvh;
  fromConvexHullToBVH(shape, bvh);
  colbench::storeSerialize(bvh, filename);
}

inline void storeConvexHull(const std::vector<std::shared_ptr<ConvexBase>>& shapes,
                     const std::string& filename){
  std::vector<BVHModel<AABB>> bvhs;
  for (size_t i = 0; i < shapes.size(); i++) {
    BVHModel<AABB> bvh;
    fromConvexHullToBVH(shapes[i], bvh);
    bvhs.push_back(bvh);
  }
  colbench::storeSerialize(bvhs, filename);
}

} // namespace colbench

#endif //#ifndef COLBENCH_SERIALIZATION_HPP
