from utils_trajectories import Args, visualize_random_trajectory, generate_random_trajectory_2shapes, render_trajectory_timestep, create_visualizer
import pycolbench
import numpy as np
import pinocchio as pin
import hppfcl
import random
from tqdm import tqdm
import time

if __name__ == "__main__":
    args = Args().parse_args()
    if args.seed >= 0:
        np.random.seed(args.seed)
        pin.seed(args.seed)
        random.seed(args.seed)

    args.dt = float(1 / args.fps)
    args.T = int(args.sim_time / args.dt)

    if args.single_trajectory:
        visualize_random_trajectory(args)
    else:
        # Create data structure to save the data
        trajectories_data = pycolbench.StdVec_StdVec_CollisionProblem()
        dit_req = hppfcl.DistanceRequest()
        dist_res = hppfcl.DistanceResult()
        col_req = hppfcl.CollisionRequest()
        col_req.security_margin = 0.
        col_req.distance_upper_bound = 0.
        col_res = hppfcl.CollisionResult()
        print("Generating trajectories...")
        for i in tqdm(range(args.numtraj)):
            trajectory = generate_random_trajectory_2shapes(args)

            traj_data = pycolbench.StdVec_CollisionProblem()
            shape1 = trajectory.rgeom_model.geometryObjects[0].geometry
            shape2 = trajectory.rgeom_model.geometryObjects[1].geometry
            for t in range(len(trajectory.xs)):
                q = trajectory.xs[t][:trajectory.rmodel.nq]
                pin.updateGeometryPlacements(trajectory.rmodel, trajectory.rdata,
                                             trajectory.rgeom_model, trajectory.rgeom_data, q)
                prob = pycolbench.CollisionProblem()
                prob.pair_id = i
                prob.id_shape1 = trajectory.id_shape1
                prob.id_shape2 = trajectory.id_shape2
                prob.id_pose = t
                M1 = trajectory.rgeom_data.oMg[0]
                M2 = trajectory.rgeom_data.oMg[1]
                prob.M1 = M1
                prob.M2 = M2
                prob.unscaled_translation = M2.translation
                dist_res.clear()
                dist = hppfcl.distance(shape1, M1, shape2, M2, dit_req, dist_res)
                col_res.clear()
                col = hppfcl.collide(shape1, M1, shape2, M2, col_req, col_res)
                prob.p1 = dist_res.getNearestPoint1()
                prob.p2 = dist_res.getNearestPoint2()
                prob.unscaled_separation_vector = prob.p1 - prob.p2
                prob.normalized_separation_vector = prob.unscaled_separation_vector / np.linalg.norm(prob.unscaled_separation_vector)
                prob.unscaled_dist = dist
                if col:
                    contact: hppfcl.Contact = col_res.getContacts()[0]
                    prob.p1_early = contact.getNearestPoint1()
                    prob.p2_early = contact.getNearestPoint2()
                    prob.unscaled_separation_vector_early_stop = prob.p1_early - prob.p2_early
                    prob.unscaled_dist_early_stop = -np.linalg.norm(prob.unscaled_separation_vector_early_stop)
                else:
                    prob.p1_early = col_res.getNearestPoint1()
                    prob.p2_early = col_res.getNearestPoint2()
                    prob.unscaled_dist_early_stop = np.linalg.norm(prob.unscaled_separation_vector_early_stop)
                    prob.unscaled_separation_vector_early_stop = prob.p1_early - prob.p2_early
                prob.normalized_separation_vector_early_stop = prob.unscaled_separation_vector_early_stop / np.linalg.norm(prob.unscaled_separation_vector_early_stop)
                traj_data.append(prob)
            trajectories_data.append(traj_data)
        save_path = "./benchmarks/ycb/data/ycb_trajectories.bin"
        trajectories_data.saveToBinary(save_path)
