//
// Copyright (c) 2023 INRIA
// Author: Louis Montaut
//

#ifndef COLBENCH_UTILITY_HPP
#define COLBENCH_UTILITY_HPP

#include "colbench/benchmark_setup.hpp"
#include "colbench/collision_problem.hpp"
#include "hpp/fcl/timings.h"
#include "colbench/benchmark_results.hpp"
#include "proxsuite/proxqp/dense/dense.hpp"
#include "colbench/fwd.hpp"
#include "colbench/csv.hpp"
#include "benchmarks/config.h"

namespace colbench{

inline void eulerToMatrix(double a, double b, double c, Matrix3d& R) {
  double c1 = cos(a);
  double c2 = cos(b);
  double c3 = cos(c);
  double s1 = sin(a);
  double s2 = sin(b);
  double s3 = sin(c);

  R << c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3,
      -c2 * s3, s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

inline double rand_interval(double rmin, double rmax) {
  double t = rand() / ((double)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

inline void generateRandomTransforms(double extents[6],
                              std::vector<Transform3f>& transforms,
                              std::size_t n) {
  transforms.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    double x = rand_interval(extents[0], extents[3]);
    double y = rand_interval(extents[1], extents[4]);
    double z = rand_interval(extents[2], extents[5]);

    const double pi = 3.1415926;
    double a = rand_interval(0, 2 * pi);
    double b = rand_interval(0, 2 * pi);
    double c = rand_interval(0, 2 * pi);

    {
      Matrix3d R;
      eulerToMatrix(a, b, c, R);
      Vector3d T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}

inline std::shared_ptr<ConvexBase> loadConvexMesh(std::string path){
  NODE_TYPE bv_type = BV_AABB;
  MeshLoader loader(bv_type);
  BVHModelPtr_t bvh1 = loader.load(path);
  bvh1->buildConvexHull(true, "Qt");
  return bvh1->convex;
}

inline void bringShapesToDist(const ShapeBase* shape1, const Transform3f& T1,
                       const ShapeBase* shape2, Transform3f& T2, double desired_dist){
  DistanceRequest req;
  DistanceResult res;
  double dist = hpp::fcl::distance(shape1, T1, shape2, T2, req, res);

  Vector3d separation_vector = res.nearest_points[0] - res.nearest_points[1];
  Vector3d n = separation_vector.normalized();
  Vector3d newt = T2.getTranslation();
  newt += separation_vector + (dist > 0? -1: 1) * desired_dist * n;
  T2.setTranslation(newt);
}

inline void runAndTimeGJK(GJK& gjk, const MinkowskiDiff& mink_diff,
                  const Vector3d& init_guess, const support_func_guess_t& init_support_guess,
                  const std::string& name, colbench::BenchmarkResults& res){
  double time = 0.;
  Timer timer;
  int N = 1000;
  timer.stop(); timer.start();
  for (int i = 0; i < N; i++) {
    gjk.evaluate(mink_diff, init_guess, init_support_guess);
  }
  timer.stop();
  time = timer.elapsed().user;
  time /= double(N);

  // Store results
  res.solver_name = name;
  res.dist_shapes = gjk.ray.norm();
  res.gjk_dotprod_init_star = init_guess.dot(gjk.ray);
  res.solver_time = time;
  res.gjk_numit = gjk.getIterations() + 1;
  res.gjk_it_momentum_stopped = gjk.getIterationsMomentumStopped();
  res.gjk_num_support_dotprods = gjk.getCumulativeSupportDotprods();
}

using proxsuite::proxqp::dense::QP;
inline QP<double> setupProxQPSolver(const Convex<Triangle>* shape1, const Transform3f& T1,
                             const Convex<Triangle>* shape2, const Transform3f& T2) {
  proxsuite::proxqp::isize dim = 6;
  proxsuite::proxqp::isize neq = 0;
  proxsuite::proxqp::isize nin = shape1->num_normals + shape2->num_normals;
  proxsuite::proxqp::dense::QP<double> qp_solver(dim, neq, nin);

  Eigen::Matrix<double, 3, 3> I3; I3.setIdentity();
  Eigen::Matrix<double, 6, 6> H;
  H.block<3, 3>(0, 0) = I3;
  H.block<3, 3>(3, 3) = I3;
  H.block<3, 3>(0, 3) = -I3;
  H.block<3, 3>(3, 0) = -I3;
  // H *= 0.5;

  Eigen::Vector<double, 6> g; g.setZero();
  auto A = proxsuite::nullopt;
  auto b = proxsuite::nullopt;

  // double description
  Eigen::VectorXd l = Eigen::VectorXd(nin); l.setOnes(); l *= -1e20;
  auto R1 = T1.getRotation(); auto t1 = T1.getTranslation();
  auto R2 = T2.getRotation(); auto t2 = T2.getTranslation();
  Eigen::VectorXd u = Eigen::VectorXd(nin); u.setZero();
  Eigen::MatrixXd C = Eigen::MatrixXd(nin, dim); C.setZero();
  for (size_t i = 0; i < shape1->num_normals; i++) {
    auto n = R1 * shape1->normals[i];
    C.block<1, 3>(static_cast<int>(i), 0) = n;
    u(static_cast<int>(i)) = n.dot(t1) - shape1->offsets[i];
  }
  for (size_t i = 0; i < shape2->num_normals; i++) {
    auto n = R2 * shape2->normals[i];
    C.block<1, 3>(static_cast<int>(shape1->num_normals + i), 3) = n;
    u(static_cast<int>(shape1->num_normals + i)) = n.dot(t2) - shape2->offsets[i];
  }
  qp_solver.init(H, g, A, b, C, l, u);
  return qp_solver;
}

inline void createDir(const std::string& path){
  if (!std::filesystem::exists(path)){
    std::filesystem::create_directories(path);
  }
}

inline void read_ycb_csv(std::vector<std::string>& paths) {
  std::string filename(std::string(COLBENCH_YCB_DIR) + "/data/ycb.csv");
  bool exists = std::filesystem::exists(filename);
  if (!exists){
    throw std::logic_error(filename + " does not exist.\
              Make sure you have run `python benchmarks/ycb/ycb_download.py` and\
              `python benchmarks/ycb/ycb_to_csv.py` in the root repository directory.");
  }
  io::CSVReader<3> file(filename);
  file.read_header(io::ignore_no_column, "shape_path", "num_vertices", "num_faces");
  std::string shape_path; int num_vertices, num_faces;
  while(file.read_row(shape_path, num_vertices, num_faces)){
    paths.push_back(shape_path);
  }
}

// Create cube
inline Convex<Triangle>* create_cube(double l, double w, double d){
  Vector3d* pts = new Vector3d[8];
  pts[0] = Vector3d(l, w, d);
  pts[1] = Vector3d(l, w, -d);
  pts[2] = Vector3d(l, -w, d);
  pts[3] = Vector3d(l, -w, -d);
  pts[4] = Vector3d(-l, w, d);
  pts[5] = Vector3d(-l, w, -d);
  pts[6] = Vector3d(-l, -w, d);
  pts[7] = Vector3d(-l, -w, -d);

  Triangle* polygons = new Triangle[12];
  polygons[0].set(0, 3, 1);
  polygons[1].set(0, 2, 3);
  polygons[2].set(0, 1, 5);
  polygons[3].set(0, 5, 4);
  polygons[4].set(3, 5, 1);
  polygons[5].set(3, 7, 5);
  polygons[6].set(2, 7, 3);
  polygons[7].set(2, 6, 7);
  polygons[8].set(6, 5, 7);
  polygons[9].set(6, 4, 5);
  polygons[10].set(4, 6, 2);
  polygons[11].set(4, 2, 0);
  Convex<Triangle>* cube_ptr = new Convex<Triangle>(true,
                                                    pts,  // points
                                                    8,    // num points
                                                    polygons,
                                                    12  // number of polygons
    );

  return cube_ptr;
}

using hpp::fcl::Vec3f;
using hpp::fcl::FCL_REAL;
inline Convex<Triangle>* create_cube_proxqp(double l, double w, double d){
  Convex<Triangle>* cube_ptr = create_cube(l, w, d);
  cube_ptr->num_normals = 6;
  cube_ptr->normals = new Vec3f[6];
  cube_ptr->offsets = new FCL_REAL[6];
  cube_ptr->normals[0] = Vec3f(1, 0, 0);
  cube_ptr->offsets[0] = -l;
  cube_ptr->normals[1] = Vec3f(-1, 0, 0);
  cube_ptr->offsets[1] = -l;

  cube_ptr->normals[2] = Vec3f(0, 1, 0);
  cube_ptr->offsets[2] = -w;
  cube_ptr->normals[3] = Vec3f(0, -1, 0);
  cube_ptr->offsets[3] = -w;

  cube_ptr->normals[4] = Vec3f(0, 0, 1);
  cube_ptr->offsets[4] = -d;
  cube_ptr->normals[5] = Vec3f(0, 0, -1);
  cube_ptr->offsets[5] = -d;
  return cube_ptr;
}

inline void updateBenchResultsBase(BenchmarkResults& bench_results, const BenchmarkSetup& bench_setup, const CollisionProblem& problem)
{
  bench_results.solver_name = bench_setup.solver_type;
  bench_results.pair_id = problem.pair_id;
  bench_results.id_shape1 = problem.id_shape1;
  bench_results.id_shape2 = problem.id_shape2;
  bench_results.id_pose = problem.id_pose;
}

inline void updateBenchResults(BenchmarkResults& bench_results, const BenchmarkSetup& bench_setup, const CollisionProblem& problem,
                               const Convex<Triangle>* shape1, const Convex<Triangle>* shape2)
{
  updateBenchResultsBase(bench_results, bench_setup, problem);
  bench_results.num_vertices_shape1 = shape1->num_points;
  bench_results.num_faces_shape1 = shape1->num_polygons;
  bench_results.num_vertices_shape2 = shape2->num_points;
  bench_results.num_faces_shape2 = shape2->num_polygons;
}

using hpp::fcl::Ellipsoid;
inline void updateBenchResults(BenchmarkResults& bench_results, const BenchmarkSetup& bench_setup, const CollisionProblem& problem,
                               const Ellipsoid*, const Ellipsoid*)
{
  updateBenchResultsBase(bench_results, bench_setup, problem);
  bench_results.num_vertices_shape1 = 0;
  bench_results.num_faces_shape1 = 0;
  bench_results.num_vertices_shape2 = 0;
  bench_results.num_faces_shape2 = 0;
}

template<class S>
inline void appendBenchResultsToFileGJK(BenchmarkResults& bench_results, std::ofstream& file_results,
                                        const BenchmarkSetup& bench_setup, const CollisionProblem& problem,
                                        const S* shape1, const S* shape2,
                                        const GJK& gjk, const Vector3d& init_guess, const double& timings)
{
  bench_results.clear();
  updateBenchResults(bench_results, bench_setup, problem, shape1, shape2);
  bench_results.dist_shapes = (gjk.ray).norm();
  bench_results.gjk_dotprod_init_star = (gjk.ray.normalized()).dot(init_guess.normalized());
  bench_results.solver_time = timings;
  bench_results.gjk_numit = gjk.getIterations() + 1;
  bench_results.gjk_it_momentum_stopped = gjk.getIterationsMomentumStopped() + 1;
  bench_results.gjk_num_support_dotprods = gjk.getCumulativeSupportDotprods();
  bench_results.is_collision = (bench_results.dist_shapes * bench_results.dist_shapes <= bench_setup.tol ? 1: 0);
  bench_results.storeInFile(file_results);
}

template<class S>
inline void appendBenchResultsToFileProxQP(BenchmarkResults& bench_results, std::ofstream& file_results,
                                           const BenchmarkSetup& bench_setup, const CollisionProblem& problem,
                                           const S* shape1, const S* shape2,
                                           const QP<double>& qp, const double& timings)
{
  bench_results.clear();
  updateBenchResults(bench_results, bench_setup, problem, shape1, shape2);
  bench_results.dist_shapes = (qp.results.x.block<3, 1>(0, 0) - qp.results.x.block<3, 1>(3, 0)).norm();
  bench_results.gjk_dotprod_init_star = 0;
  bench_results.solver_time = timings;
  bench_results.gjk_numit = 0;
  bench_results.gjk_it_momentum_stopped = 0;
  bench_results.gjk_num_support_dotprods = 0;
  bench_results.is_collision = (bench_results.dist_shapes * bench_results.dist_shapes <= bench_setup.tol ? 1: 0);
  bench_results.storeInFile(file_results);
}


} // namespace colbench
#endif
