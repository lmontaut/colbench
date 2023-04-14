//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#include <boost/python/def.hpp>
#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "colbench.hh"
#include "colbench/collision_problem.hpp"
#include "colbench/serialization.hpp"
#include "pickle.hh"

using colbench::CollisionProblem;

#define DEF_RW_CLASS_ATTRIB(CLASS, ATTRIB) \
  def_readwrite(#ATTRIB, &CLASS::ATTRIB)

BOOST_PYTHON_MODULE(pycolbench){
  PyImport_ImportModule("warnings");
  expose_colbench();
}

struct CollisionProblemWrapper {
  static void saveToBinary(const CollisionProblem& problem,
                           const std::string& path) {
    colbench::storeSerialize(problem, path);
  }

  static void loadFromBinary(CollisionProblem& problem,
                             const std::string& path) {
    colbench::loadSerialize(problem, path);
  }
};

struct CollisionProblemVectorWrapper {
  static void saveToBinary(const std::vector<CollisionProblem>& vec_problem,
                           const std::string& path) {
    colbench::storeSerialize(vec_problem, path);
  }

  static void loadFromBinary(std::vector<CollisionProblem>& vec_problem,
                             const std::string& path) {
    colbench::loadSerialize(vec_problem, path);
  }
};

struct CollisionProblemVectorVectorWrapper {
  static void saveToBinary(const std::vector<std::vector<CollisionProblem>>& vec_vec_problem,
                           const std::string& path) {
    colbench::storeSerialize(vec_vec_problem, path);
  }

  static void loadFromBinary(std::vector<std::vector<CollisionProblem>>& vec_vec_problem,
                             const std::string& path) {
    colbench::loadSerialize(vec_vec_problem, path);
  }
};

void expose_colbench(){
  if (!eigenpy::register_symbolic_link_to_registered_type<CollisionProblem>()) {
    boost::python::class_<CollisionProblem> (
      "CollisionProblem", "Data structure to store a collision problem", boost::python::no_init)
    .def(boost::python::init<>(boost::python::arg("self"),"Default constructor"))
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, pair_id)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, id_shape1)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, id_shape2)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, id_pose)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, M1)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, M2)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, p1)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, p2)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, p1_early)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, p2_early)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, unscaled_translation)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, unscaled_separation_vector)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, normalized_separation_vector)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, unscaled_dist)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, unscaled_separation_vector_early_stop)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, normalized_separation_vector_early_stop)
    .DEF_RW_CLASS_ATTRIB(CollisionProblem, unscaled_dist_early_stop)
    .def("scaleToDist", &CollisionProblem::scaleToDist)
    .def("saveToBinary", &CollisionProblemWrapper::saveToBinary)
    .def("loadFromBinary", &CollisionProblemWrapper::loadFromBinary)
    .def("clone", &CollisionProblem::clone, "Clone method.", return_value_policy<manage_new_object>())
    .def(self == self)
    .def_pickle(PickleObject<CollisionProblem>())
    ;
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<std::vector<CollisionProblem>>()) {
    boost::python::class_<std::vector<CollisionProblem>> ("StdVec_CollisionProblem")
      .def("saveToBinary", &CollisionProblemVectorWrapper::saveToBinary)
      .def("loadFromBinary", &CollisionProblemVectorWrapper::loadFromBinary)
      .def(vector_indexing_suite<std::vector<CollisionProblem> >())
      .def_pickle(PickleObject<std::vector<CollisionProblem>>())
    ;
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<std::vector<std::vector<CollisionProblem>>>()) {
    boost::python::class_<std::vector<std::vector<CollisionProblem>>> ("StdVec_StdVec_CollisionProblem")
      .def("saveToBinary", &CollisionProblemVectorVectorWrapper::saveToBinary)
      .def("loadFromBinary", &CollisionProblemVectorVectorWrapper::loadFromBinary)
      .def(vector_indexing_suite<std::vector<std::vector<CollisionProblem>>>())
      .def_pickle(PickleObject<std::vector<std::vector<CollisionProblem>>>())
    ;
  }
}
