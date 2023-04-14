//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_PROBLEM_HPP
#define COLBENCH_PROBLEM_HPP

#include <string.h>
#include "colbench/fwd.hpp"

namespace colbench {

struct CollisionProblem {
  // We generate a bunch of collision problems by doing the following:
  // - We select pairs of objects
  // - For each pair, we generate N poses, for which we compute the separation vector
  // - The separation vector can be used to control the distance btw the shapes
  size_t pair_id;
  size_t id_shape1;
  size_t id_shape2;
  size_t id_pose;
  SE3 M1; // pose of shape2 -- pose of shape1 is always the identity
  SE3 M2; // pose of shape2 -- pose of shape1 is always the identity
  Vector3d p1, p2; // Nearest points after GJK converged
  Vector3d p1_early, p2_early; // Nearest points after early stopping
  Vector3d unscaled_translation;
  // Distance computation
  Vector3d unscaled_separation_vector;
  Vector3d normalized_separation_vector;
  double unscaled_dist; // dist btw shape1 and shape2 for pose T
  // Boolean collision check
  Vector3d unscaled_separation_vector_early_stop;
  Vector3d normalized_separation_vector_early_stop;
  double unscaled_dist_early_stop; // lower bound on dist btw shape1 and shape2 for pose T

  void scaleToDist(const double& desired_dist){
    // t = M.translation()
    // t <- t + dt
    M2.translation(// t
                  unscaled_translation +
                  // dt
                  unscaled_separation_vector +
                  (unscaled_dist > 0? -1: 1) *
                  desired_dist *
                  normalized_separation_vector);
  }

  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & pair_id;
      ar & id_shape1;
      ar & id_shape2;
      ar & id_pose;
      ar & M1;
      ar & M2;
      ar & p1;
      ar & p2;
      ar & p1_early;
      ar & p2_early;
      ar & unscaled_translation;
      ar & unscaled_separation_vector;
      ar & normalized_separation_vector;
      ar & unscaled_dist;
      ar & unscaled_separation_vector_early_stop;
      ar & normalized_separation_vector_early_stop;
      ar & unscaled_dist_early_stop;
  }

  CollisionProblem(){
    pair_id = 0;
    id_shape1 = 0;
    id_shape2 = 0;
    id_pose = 0;
    M1.setIdentity();
    M2.setIdentity();
    p1.setZero(); p2.setZero();
    p1_early.setZero(); p2_early.setZero();
    unscaled_translation.setZero();
    unscaled_separation_vector.setZero();
    normalized_separation_vector.setZero();
    unscaled_dist = -1.;
    unscaled_separation_vector_early_stop.setZero();
    normalized_separation_vector_early_stop.setZero();
    unscaled_dist_early_stop = -1.;
  }

  CollisionProblem* clone() const { return new CollisionProblem(*this); };

  inline bool operator==(const CollisionProblem& other) const {
    return pair_id == other.pair_id &&
           id_shape1 == other.id_shape1 &&
           id_shape2 == other.id_shape2 &&
           id_pose == other.id_pose &&
           M1 == other.M1 &&
           M2 == other.M2 &&
           p1 == other.p1 && p2 == other.p2 &&
           p1_early == other.p1_early && p2_early == other.p2_early &&
           unscaled_translation == other.unscaled_translation &&
           unscaled_separation_vector == other.unscaled_separation_vector &&
           normalized_separation_vector == other.normalized_separation_vector &&
           unscaled_dist == other.unscaled_dist &&
           unscaled_separation_vector_early_stop == other.unscaled_separation_vector_early_stop &&
           normalized_separation_vector_early_stop == other.normalized_separation_vector_early_stop &&
           unscaled_dist_early_stop == other.unscaled_dist_early_stop;
  }

}; // struct CollisionProblem

}
#endif
