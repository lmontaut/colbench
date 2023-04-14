import hppfcl
import numpy as np
import pinocchio as pin
import pandas as pd
import random
from pycolbench.utils_render import renderPoint, renderLine, renderArrowLine, meshcat_material, loadmesh, get_transform
from pycontact.simulators import (
        NCPPGSSimulator,
        RaisimSimulator,
        NCPStagProjSimulator,
        CCPADMMSimulator)
from typing import List, Tuple
from tqdm import tqdm
from tap import Tap

class Args(Tap):
    single_trajectory: bool = False
    numtraj: int = 100
    seed: int = 123
    fps: int = 120
    mu: float = 0.4
    el: float = 0.2
    dt: float = -1.
    sim_time: float = 2.
    T: int = -1
    K: float = 2.
    mass1: float = 1.
    mass2: float = 1.
    scale: float = 1.
    scale_inertia: float = 1.
    scale_contact: float = 1
    add_walls: bool = False
    shape_type: str = "mesh"
    debug: bool = False
    debug_contact: bool = False
    vis_contact_normals: bool = False
    vis_contact_axis: bool = False
    vis_witness_points: bool = False
    grid: bool = False
    axes: bool = False
    opacity: float = 1.0

class Trajectory:
    rmodel: pin.Model
    rdata: pin.Data
    rgeom_model: pin.GeometryModel
    rgeom_data: pin.GeometryData
    rvisual_model: pin.GeometryModel
    rvisual_data: pin.GeometryData
    actuation: np.ndarray
    q0: np.ndarray
    xs: List = []
    lams: List[np.ndarray] = []
    Js: List[np.ndarray] = []
    Rs: List[np.ndarray] = []
    es: List[np.ndarray] = []
    Gs: List[np.ndarray] = []
    gs: List[np.ndarray] = []
    mus: List[np.ndarray] = []
    contact_points: List = []
    T: int = 0
    mu:float = 0.
    el: float = 0.
    dt: float = 0.
    sim_time: float = 0.
    K: float = 0.
    id_shape1: int = -1
    id_shape2: int = -1
    n_iter_: List
    stop_: List

    def __init__(self, rmodel: pin.Model, rgeom_model: pin.GeometryModel,
                 rvisual_model: pin.GeometryModel, actuation: np.ndarray) -> None:
        self.rmodel = rmodel
        self.rgeom_model = rgeom_model
        self.rvisual_model = rvisual_model
        self.actuation = actuation
        self.q0 = self.rmodel.qref
        self.xs = []
        self.n_iter_ = []
        self.stop_ = []

    def create_data(self) -> None:
        self.rdata = self.rmodel.createData()
        self.rgeom_data = self.rgeom_model.createData()
        self.rvisual_data = self.rvisual_model.createData()

def load_mesh(path: str, scale: float=1) -> hppfcl.Convex:
    loader = hppfcl.MeshLoader()
    mesh = loader.load(path, scale * np.ones(3))
    mesh.buildConvexHull(True, "Qt")
    return mesh.convex

def create_trajectory_model_2shapes(args: Args, path1: str, path2: str) -> Trajectory:
    """
    Docstring for Pinocchio pin.Inertia:
    This class represenses a sparse version of a Spatial Inertia and its is defined by its mass, its center o
    f mass location and the rotational inertia expressed around this center of mass.
    """
    rmodel = pin.Model()
    rgeom_model = pin.GeometryModel()
    rgeom_model.frictions = []
    rgeom_model.elasticities = []

    # SHAPE 1
    l = 0.1
    freeflyer = pin.JointModelFreeFlyer()
    M1 = pin.SE3.Identity()
    jointObj1 = rmodel.addJoint(0, freeflyer, M1, "jointObj1")
    if args.shape_type == "mesh":
        shape1 = load_mesh(path1, args.scale)
        v1 = shape1.computeCOM()
        I1_ = args.mass1 * shape1.computeMomentofInertiaRelatedToCOM() * args.scale_inertia / shape1.computeVolume()
        I1 = pin.Inertia(args.mass1, v1, I1_)
        if args.debug or args.debug_contact:
            print(f"Shape1 mesh file: {path1}")
    elif args.shape_type == "box":
        shape1 = hppfcl.Box(args.scale * l, args.scale * l, args.scale * l)
        I1 = pin.Inertia.FromBox(args.mass1, args.scale_inertia * args.scale * l, args.scale_inertia * args.scale * l, args.scale_inertia * args.scale * l)
    elif args.shape_type == "sphere":
        shape1 = hppfcl.Sphere(args.scale * l)
        I1 = pin.Inertia.FromSphere(args.mass1, args.scale_inertia * args.scale * l)
    else:
        raise NotImplementedError
    if args.debug or args.debug_contact:
        shape1.computeLocalAABB()
        aabb1: hppfcl.AABB = shape1.aabb_local
        w, l, h = aabb1.width(), aabb1.depth(), aabb1.height()
        print(f"Shape1 AABB dimensions: {round(w,2)} x {round(l,2)} x {round(h,2)}")
        print(f"Shape1 volume: {shape1.computeVolume()}")
        print(f"Inertia matrix shape1: {I1}\n")
    rmodel.appendBodyToJoint(jointObj1, I1, M1)
    geom1 = pin.GeometryObject("obj1", jointObj1, jointObj1, M1, shape1)

    geom1.meshColor = np.array([1.0, 0.2, 0.2, args.opacity])
    geom1_id = rgeom_model.addGeometryObject(geom1)

    # SHAPE 2
    l = 0.1
    freeflyer = pin.JointModelFreeFlyer()
    M2 = pin.SE3.Identity()
    jointObj2 = rmodel.addJoint(0, freeflyer, M2, "jointObj2")
    if args.shape_type == "mesh":
        shape2 = load_mesh(path2, args.scale)
        v2 = shape2.computeCOM()
        I2_ = args.mass2 * shape2.computeMomentofInertiaRelatedToCOM() * args.scale_inertia / shape2.computeVolume()
        I2 = pin.Inertia(args.mass2, v2, I2_)
        if args.debug or args.debug_contact:
            print(f"Shape2 mesh file: {path2}")
    elif args.shape_type == "box":
        shape2 = hppfcl.Box(args.scale * l, args.scale * l, args.scale * l)
        I2 = pin.Inertia.FromBox(args.mass2, args.scale_inertia * args.scale * l, args.scale_inertia * args.scale * l, args.scale_inertia * args.scale * l)
    elif args.shape_type == "sphere":
        shape2 = hppfcl.Sphere(args.scale * l)
        I2 = pin.Inertia.FromSphere(args.mass2, args.scale_inertia * args.scale * l)
    else:
        raise NotImplementedError

    if args.debug or args.debug_contact:
        shape2.computeLocalAABB()
        aabb2: hppfcl.AABB = shape2.aabb_local
        w, l, h = aabb2.width(), aabb2.depth(), aabb2.height()
        print(f"Shape2 AABB dimensions: {round(w,2)} x {round(l,2)} x {round(h,2)}")
        print(f"Shape2 volume: {shape2.computeVolume()}")
        print(f"Inertia matrix shape2: {I2}\n")
    rmodel.appendBodyToJoint(jointObj2, I2, M2)
    geom2 = pin.GeometryObject("obj2", jointObj2, jointObj2, M2, shape2)

    geom2.meshColor = np.array([0.2, 1.0, 0.2, args.opacity])
    geom2_id = rgeom_model.addGeometryObject(geom2)

    col_pair1 = pin.CollisionPair(geom1_id, geom2_id)
    rgeom_model.addCollisionPair(col_pair1)
    rgeom_model.frictions += [args.mu]
    rgeom_model.elasticities += [args.el]

    # PLANE
    n = np.array([0.0, 0.0, 1])
    floor_shape = hppfcl.Halfspace(n, 0)
    T = pin.SE3.Identity()
    floor = pin.GeometryObject("floor", 0, 0, T, floor_shape)
    floor.meshColor = np.array([1., 1., 1., 1.])
    floor_id = rgeom_model.addGeometryObject(floor)

    col_pair2 = pin.CollisionPair(floor_id, geom1_id)
    rgeom_model.addCollisionPair(col_pair2)
    rgeom_model.frictions += [args.mu]
    rgeom_model.elasticities += [args.el]

    col_pair3 = pin.CollisionPair(floor_id, geom2_id)
    rgeom_model.addCollisionPair(col_pair3)
    rgeom_model.frictions += [args.mu]
    rgeom_model.elasticities += [args.el]

    # WALLS
    if args.add_walls:
        nz = np.zeros(3)
        nz[2] = 1.
        pts = [np.array([0, -0.5, 0.]),
               np.array([-0.5, 0., 0.]),
               np.array([0., 0.5, 0.]),
               np.array([0.5, 0., 0.])]
        for i in range(4):
            angle = 30.
            theta = (angle / 180.) * 3.14
            if i < 2:
                theta *= -1.
            R = np.eye(3)
            c, s = np.cos(theta), np.sin(theta)
            if i%2 == 0:
                # Rotation along the x-axis
                R[1, 1] = c
                R[2, 2] = c
                R[1, 2] = -s
                R[2, 1] = s
            else:
                # Rotation along y-axis
                R[0, 0] = c
                R[2, 2] = c
                R[0, 2] = -s
                R[2, 0] = s
            wall_shape = hppfcl.Halfspace(nz, 0)
            T = pin.SE3(R, 0.1 * args.scale * pts[i])
            wall = pin.GeometryObject("wall_" + str(i), 0, 0, T, wall_shape)
            wall.meshColor = np.array([0.5, 0.5, 0.5, 1.])
            if i == 3:
                wall.meshColor[3] = 0.
            wall_id = rgeom_model.addGeometryObject(wall)
            col_pair_wall1 = pin.CollisionPair(wall_id, geom1_id)
            rgeom_model.addCollisionPair(col_pair_wall1)
            rgeom_model.frictions += [args.mu]
            rgeom_model.elasticities += [args.el]

            col_pair_wall2 = pin.CollisionPair(wall_id, geom2_id)
            rgeom_model.addCollisionPair(col_pair_wall2)
            rgeom_model.frictions += [args.mu]
            rgeom_model.elasticities += [args.el]

    rmodel.qref = pin.neutral(rmodel)

    actuation = np.eye(rmodel.nv)
    rvisual_model = rgeom_model.copy()
    trajectory = Trajectory(rmodel, rgeom_model, rvisual_model, actuation)
    trajectory.create_data()
    for req in trajectory.rgeom_data.collisionRequests:
        req.security_margin = 1e-3
    trajectory.path1 = path1
    trajectory.path2 = path2
    return trajectory

def get_random_ycb_path() -> Tuple[int, str]:
    df = pd.read_csv("./benchmarks/ycb/data/ycb.csv")
    ycb_paths = df["shape_path"].tolist()
    n_ycb = len(ycb_paths)
    i = random.randint(0, n_ycb - 1)
    path = ycb_paths[i]
    return i, path

def random_configuration(model: pin.Model, data: pin.Data,
                         geom_model: pin.GeometryModel, geom_data: pin.GeometryData):
    valid_conf = False
    lower_limit = np.zeros(model.nq)
    upper_limit = np.ones(model.nq)
    for _ in range(100):
        q = pin.randomConfiguration(model, lower_limit, upper_limit * 2.0)
        valid_conf = True
        pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
        pin.computeCollisions(geom_model, geom_data, True)
        for res in geom_data.collisionResults:
            if res.isCollision():
                valid_conf = False
        if valid_conf:
            return q
    return model.qinit

def generate_trajectory(trajectory: Trajectory, scale: float = 1.):
    rmodel, rdata = trajectory.rmodel, trajectory.rdata
    rgeom_model, rgeom_data = trajectory.rgeom_model, trajectory.rgeom_data
    actuation = trajectory.actuation

    # Create simulator
    simulator = NCPPGSSimulator(warm_start=True)
    # simulator = RaisimSimulator(warm_start=True)
    # simulator = NCPStagProjSimulator(warm_start=True)
    # simulator = CCPADMMSimulator(warm_start=True)

    # Init config
    a = 0.2
    q0 = rmodel.qref
    q0[0] = np.random.rand(1)[0] * 0.
    q0[7] = 0.
    q0[1] = -0.1 * scale
    q0[2] = (3 * a / 2) * scale
    q0[8] = 0.1 * scale
    q0[9] = (3. * a / 2) * scale
    trajectory.q0 = q0
    v0 = np.random.rand(rmodel.nv)
    # v0[0] = v0[1] = v0[2] = v0[6] = v0[7] = v0[8] = 0.
    q, v = q0.copy(), v0.copy()

    # Run simulator for T steps
    trajectory.xs = []
    trajectory.lams = []
    trajectory.Js = []
    trajectory.Rs = []
    trajectory.Gs = []
    trajectory.gs = []
    trajectory.mus = []
    trajectory.contact_points = []
    trajectory.n_iter_ = []
    trajectory.stop_ = []
    for t in range(trajectory.T):
        trajectory.xs += [np.concatenate((q, v))]
        tau_act = np.zeros(actuation.shape[1])
        tau = actuation @ tau_act
        fext = [pin.Force(np.zeros(6)) for _ in range(rmodel.njoints)]
        q, v = simulator.step(
            rmodel, rdata, rgeom_model, rgeom_data, q, v, tau,
            fext, trajectory.dt, trajectory.K, 1000, 1e-8#,
            #rel_th_stop=1e-16
        )
        trajectory.lams += [simulator.lam]
        trajectory.Js += [simulator.J]
        trajectory.Rs += [simulator.R]
        trajectory.Gs += [simulator.G]
        trajectory.gs += [simulator.g]
        trajectory.mus += [simulator.mus]
        trajectory.contact_points += [simulator.contact_points]
        trajectory.n_iter_ += [simulator.solver.n_iter_]
        trajectory.stop_ += [simulator.solver.stop_]

def generate_random_trajectory_2shapes(args: Args, path1: str = None, path2: str = None) -> Trajectory:
    i, j = 0, 0
    if path1 is None and args.shape_type == "mesh":
        i, path1 = get_random_ycb_path()
    if path2 is None and args.shape_type == "mesh":
        j, path2 = get_random_ycb_path()
    trajectory = create_trajectory_model_2shapes(args, path1, path2)
    trajectory.id_shape1 = i
    trajectory.id_shape2 = j
    trajectory.T = args.T
    trajectory.dt = args.dt
    trajectory.sim_time = args.sim_time
    trajectory.mu = args.mu
    trajectory.el = args.el
    trajectory.K = args.K
    generate_trajectory(trajectory, args.scale)
    return trajectory

Viewer = pin.visualize.MeshcatVisualizer
def create_visualizer(trajectory: Trajectory, args: Args, q0: np.ndarray = None) -> Viewer:
    import meshcat
    vis = Viewer(trajectory.rmodel, trajectory.rgeom_model, trajectory.rvisual_model)
    vis.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    vis.viewer.delete()
    vis.loadViewerModel()
    if not args.grid:
        vis.viewer["/Grid"].set_property("visible", False)
    else:
        vis.viewer["/Grid"].set_property("visible", True)
    if not args.axes:
        vis.viewer["/Axes"].set_property("visible", False)
    else:
        vis.viewer["/Axes"].set_property("visible", True)

    # OBB rendering
    # if args.shape_type == "mesh":
    #     def render_mesh_and_aabb(path: str, shape_id: int):
    #         loader = hppfcl.MeshLoader()
    #         mesh: hppfcl.BVHModelOBB = loader.load(path)
    #         mesh_vis = loadmesh(mesh)
    #         if shape_id == 1:
    #             color_mesh = meshcat_material(1., 0., 0., 1.)
    #         if shape_id == 2:
    #             color_mesh = meshcat_material(0., 1., 0., 1.)
    #         vis.viewer[f"pinocchio/visuals/obj{shape_id}/full_mesh"].set_object(mesh_vis, color_mesh)
    #         mesh.buildConvexHull(True, "Qt")
    #         cvx: hppfcl.Convex = mesh.convex
    #         cvx.computeLocalAABB()
    #         aabb: hppfcl.AABB = cvx.aabb_local
    #         l = aabb.max_ - aabb.min_
    #         aabb_vis = meshcat.geometry.Box(l)
    #         if shape_id == 1:
    #             color_aabb = meshcat_material(1., 0., 0., 0.2)
    #         if shape_id == 2:
    #             color_aabb = meshcat_material(0., 1., 0., 0.2)
    #         vis.viewer[f"pinocchio/visuals/obj{shape_id}/aabb"].set_object(aabb_vis, color_aabb)
    #         center = (aabb.max_ + aabb.min_) * 0.5
    #         T_ = hppfcl.Transform3f(np.eye(3), center)
    #         T = get_transform(T_)
    #         vis.viewer[f"pinocchio/visuals/obj{shape_id}/aabb"].set_transform(T)
    #
    #         if shape_id == 1:
    #             color_line = np.array([1., 0., 0., 1.])
    #         if shape_id == 2:
    #             color_line = np.array([0., 1., 0., 1.])
    #         points = []
    #         ll = l / 2
    #         corner = center + ll
    #         points.append(corner)
    #         points.append(corner - np.array([l[0], 0, 0]))
    #         points.append(corner - np.array([0, l[1], 0]))
    #         points.append(corner - np.array([0, 0, l[2]]))
    #         corner = center - ll
    #         points.append(corner)
    #         points.append(corner + np.array([l[0], 0, 0]))
    #         points.append(corner + np.array([0, l[1], 0]))
    #         points.append(corner + np.array([0, 0, l[2]]))
    #
    #         lines = []
    #         lines.append([0, 1])
    #         lines.append([0, 2])
    #         lines.append([0, 3])
    #         lines.append([3, 5])
    #         lines.append([3, 6])
    #         lines.append([4, 5])
    #         lines.append([4, 6])
    #         lines.append([4, 7])
    #         lines.append([1, 7])
    #         lines.append([2, 7])
    #         lines.append([1, 6])
    #         lines.append([2, 5])
    #         for i in range(len(lines)):
    #             pos1 = points[lines[i][0]]
    #             pos2 = points[lines[i][1]]
    #             renderLine(vis.viewer, pos1, pos2, f"pinocchio/visuals/obj{shape_id}/aabb_edges/edge{i}", 0.002, color_line)
    #
    #     render_mesh_and_aabb(trajectory.path1, 1)
    #     render_mesh_and_aabb(trajectory.path2, 2)

    q = q0 if q0 is not None else trajectory.rmodel.qref
    pin.updateGeometryPlacements(trajectory.rmodel, trajectory.rdata,
                                 trajectory.rgeom_model, trajectory.rgeom_data, q)

    plane_width = 1e-3
    geom_id = trajectory.rgeom_model.getGeometryId("floor")
    geom_obj = trajectory.rgeom_model.geometryObjects[geom_id]
    vis.viewer["floor"].set_object(meshcat.geometry.Box(np.array([100 * args.scale, 100 * args.scale, plane_width])),
                                   meshcat_material(*geom_obj.meshColor))
    Mplus = np.eye(4)
    Mplus[:3, 3] = np.array([0, 0, -plane_width / 2])
    geom_id = trajectory.rgeom_model.getGeometryId("floor")
    M_ = trajectory.rgeom_data.oMg[geom_id]
    M = np.eye(4)
    M[:3, 3] = M_.translation
    M[:3, :3] = M_.rotation
    M = np.dot(M, Mplus)
    vis.viewer["floor"].set_transform(M)

    if args.add_walls:
        for i in range(4):
            geom_id = trajectory.rgeom_model.getGeometryId("wall_" + str(i))
            geom_obj = trajectory.rgeom_model.geometryObjects[geom_id]
            if (geom_obj.meshColor[3] > 0.):
                vis.viewer["wall_" + str(i)].set_object(meshcat.geometry.Box(np.array([args.scale * 1, args.scale * 1, plane_width])),
                                                        meshcat_material(*geom_obj.meshColor))
                M_ = trajectory.rgeom_data.oMg[geom_id]
                M = np.eye(4)
                M[:3, 3] = M_.translation
                M[:3, :3] = M_.rotation
                M = np.dot(M, Mplus)
                vis.viewer["wall_" + str(i)].set_transform(M)
    vis.display(q0)
    return vis

def render_normals_and_contact_points(vis: Viewer, trajectory: Trajectory, timestep: int, args: Args) -> None:
    # Delete previous timestep's normals and cps
    vis.viewer["normals"].delete()
    vis.viewer["contact_points"].delete()

    if args.vis_witness_points:
        model, data = trajectory.rmodel, trajectory.rdata
        geom_model, geom_data = trajectory.rgeom_model, trajectory.rgeom_data
        q = trajectory.xs[timestep][:model.nq]
        pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)

        # Compute distance
        shape1_id = geom_model.getGeometryId("obj1")
        shape1 = geom_model.geometryObjects[shape1_id].geometry
        M1 = geom_data.oMg[shape1_id]
        shape2_id = geom_model.getGeometryId("obj2")
        M2 = geom_data.oMg[shape2_id]
        shape2 = geom_model.geometryObjects[shape2_id].geometry
        req = hppfcl.DistanceRequest()
        res = hppfcl.DistanceResult()
        dist = hppfcl.distance(shape1, M1, shape2, M2, req, res)

        cp1 = res.getNearestPoint1()
        cp2 = res.getNearestPoint2()
        color_line = np.array([0., 0., 1., 1.])
        if dist > 0:
            renderLine(vis.viewer, cp1, cp2, "cp1cp2", 0.002, color_line)

    if len(trajectory.contact_points[timestep]) > 0:
        if args.debug_contact:
            print(f"-- SIMULATION STEP: {timestep}")
        R: np.ndarray = trajectory.Rs[timestep] # size 3 x 3nc
        contact_points: List = trajectory.contact_points[timestep]
        nc = len(contact_points)
        lam: np.ndarray = trajectory.lams[timestep] # size 3nc x 1
        # 1 contact point == 3 forces, 3 axis
        assert(int(lam.shape[0] / 3) == len(contact_points) == int(R.shape[1] / 3) == nc)

        nrender = 0
        for i in range(nc):
            pos = contact_points[i]
            normal = R[:,3*i + 2]
            strid = f"_{str(timestep)}_{str(i)}"
            normal_force = lam[3*i+2]
            if normal_force < 0 or normal_force > 1e3:
                print("\n -- INVALID NORMAL FORCE -- ")
                from pycontact import NCPPGSSolver, ContactProblem
                solver = NCPPGSSolver()
                G = trajectory.Gs[timestep]
                g = trajectory.gs[timestep]
                mus = trajectory.mus[timestep]
                prob = ContactProblem(G, g, mus)
                lam0 = np.zeros((3 * nc, 1))
                solver.setProblem(prob)
                solver.solve(prob, lam0, 1000, 1e-6, statistics=True)
                lam_solver = np.expand_dims(solver.getSolution(), axis=-1)
                print(f"G:\n {G}")
                print(f"g:\n {g}")
                print(f"mus:\n {mus}")
                print(f"Trajectory lams: {lam}")
                print(f"Solver lams: {lam_solver}")
                input()
            if args.vis_contact_axis:
                ex = R[:,3*i]
                ey = R[:,3*i + 1]
                res1 = renderArrowLine(vis.viewer, pos, pos + args.scale_contact * args.scale * normal / 100, "normals/normal" + strid, 0.001 * args.scale, 0.01 * args.scale, np.array([0., 0., 1., 1.]))
                res2 = renderArrowLine(vis.viewer, pos, pos + args.scale_contact * args.scale * ex / 100, "normals/ex" + strid, 0.001 * args.scale, 0.01 * args.scale, np.array([1., 0., 0., 1.]))
                res3 = renderArrowLine(vis.viewer, pos, pos + args.scale_contact * args.scale * ey / 100, "normals/ey" + strid, 0.001 * args.scale, 0.01 * args.scale, np.array([0., 1., 0., 1.]))
                nrender += res1 + res2 + res3
            else:
                if args.debug_contact:
                    print(f"Contact {i} - normal force = {normal_force[0]}")
                res = renderArrowLine(vis.viewer, pos, pos + args.scale_contact * args.scale * normal * normal_force / 10, "normals/normal" + strid, 0.001 * args.scale, 0.01 * args.scale, np.array([0., 0., 1., 1.]))
                nrender += res

        if args.debug_contact and nrender > 0:
            # print(f"Numit solver: {trajectory.n_iter_[timestep]}
            input()

def render_scene(vis: Viewer, q: np.ndarray) -> None:
    vis.display(q)

def render_trajectory_timestep(vis: Viewer, trajectory: Trajectory, timestep: int, args: Args) -> None:
    q = trajectory.xs[timestep][:trajectory.rmodel.nq]
    vis.display(q)

    # Render contact points and normals
    if args.debug or args.vis_contact_normals or args.debug_contact or args.vis_witness_points:
        render_normals_and_contact_points(vis, trajectory, timestep, args)

def visualize_random_trajectory(args: Args) -> None:
    trajectory = generate_random_trajectory_2shapes(args)

    # Visualize trajectory
    import time
    vis = create_visualizer(trajectory, args, trajectory.q0)
    while True:
        userinput: str = input("Press [enter] to launch the simulation. Press [q + enter] to quit")
        if userinput == "q":
            break
        for i in tqdm(range(trajectory.T)):
            render_trajectory_timestep(vis, trajectory, i, args)
            if args.debug:
                print(trajectory.n_iter_[i], trajectory.stop_[i])
                input()
            else:
                time.sleep(args.dt)
