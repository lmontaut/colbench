//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_RESULTS_HPP
#define COLBENCH_RESULTS_HPP

#include <fstream>
#include <string.h>

namespace colbench {

void initResultsFile(std::ofstream& file){
  file << "name,pair_id,id_shape1,num_vertices_shape1,num_faces_shape1,id_shape2,num_vertices_shape2,num_faces_shape2,pose_id,dist_shapes,gjk_dotprod_init_star,solver_time,gjk_numit,gjk_it_momentum_stopped,gjk_num_support_dotprods,is_collision\n";
}

struct BenchmarkResults {
  std::string solver_name;
  size_t pair_id;
  size_t id_shape1;
  size_t num_vertices_shape1;
  size_t num_faces_shape1;
  size_t id_shape2;
  size_t num_vertices_shape2;
  size_t num_faces_shape2;
  size_t id_pose;
  double dist_shapes;
  double gjk_dotprod_init_star;
  double solver_time;
  size_t gjk_numit;
  size_t gjk_it_momentum_stopped;
  size_t gjk_num_support_dotprods;
  size_t is_collision;

  BenchmarkResults()
      : solver_name("NO_NAME"),
        pair_id(0),
        id_shape1(0),
        num_vertices_shape1(0),
        num_faces_shape1(0),
        id_shape2(0),
        num_vertices_shape2(0),
        num_faces_shape2(0),
        id_pose(0),
        dist_shapes(0),
        gjk_dotprod_init_star(0),
        solver_time(0),
        gjk_numit(0),
        gjk_it_momentum_stopped(0),
        gjk_num_support_dotprods(0),
        is_collision(0) {}

  BenchmarkResults(std::string name_, size_t pair_id_,
                   size_t id_shape1_, size_t num_vertices_shape1_, size_t num_faces_shape1_,
                   size_t id_shape2_, size_t num_vertices_shape2_, size_t num_faces_shape2_,
                   size_t id_pose_,
                   double dist_shapes_, double gjk_solve_time_,
                   double gjk_dotprod_init_star_, size_t gjk_numit_,
                   size_t gjk_it_momentum_stopped_, size_t num_support_dotprods_,
                   size_t is_collision_)
      : solver_name(name_),
        pair_id(pair_id_),
        id_shape1(id_shape1_),
        num_vertices_shape1(num_vertices_shape1_),
        num_faces_shape1(num_faces_shape1_),
        id_shape2(id_shape2_),
        num_vertices_shape2(num_vertices_shape2_),
        num_faces_shape2(num_faces_shape2_),
        id_pose(id_pose_),
        dist_shapes(dist_shapes_),
        gjk_dotprod_init_star(gjk_dotprod_init_star_),
        solver_time(gjk_solve_time_),
        gjk_numit(gjk_numit_),
        gjk_it_momentum_stopped(gjk_it_momentum_stopped_),
        gjk_num_support_dotprods(num_support_dotprods_),
        is_collision(is_collision_) {}

  inline void clear() {
    pair_id = 0;
    id_shape1 = 0;
    num_vertices_shape1 = 0;
    num_faces_shape1 = 0;
    id_shape2 = 0;
    num_vertices_shape2 = 0;
    num_faces_shape2 = 0;
    id_pose = 0;
    dist_shapes = 0;
    gjk_dotprod_init_star = 0;
    solver_time = 0;
    gjk_numit = 0;
    gjk_it_momentum_stopped = 0;
    gjk_num_support_dotprods = 0;
    is_collision = 0;
  }

  inline void storeInFile(std::ofstream& file){
  file << solver_name + "," +
          std::to_string(pair_id) + "," +
          std::to_string(id_shape1) + "," +
          std::to_string(num_vertices_shape1) + "," +
          std::to_string(num_faces_shape1) + "," +
          std::to_string(id_shape2) + "," +
          std::to_string(num_vertices_shape2) + "," +
          std::to_string(num_faces_shape2) + "," +
          std::to_string(id_pose) + "," +
          std::to_string(dist_shapes) + "," +
          std::to_string(gjk_dotprod_init_star) + "," +
          std::to_string(solver_time) + "," +
          std::to_string(gjk_numit) + "," +
          std::to_string(gjk_it_momentum_stopped) + "," +
          std::to_string(gjk_num_support_dotprods) + "," +
          std::to_string(is_collision)
          + "\n";
  }
};

} // namespace colbench

#endif
