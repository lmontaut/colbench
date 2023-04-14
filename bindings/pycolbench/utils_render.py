import numpy as np
import hppfcl
import pinocchio as pin
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

def create_visualizer(grid=False, axes=False):
    vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    vis.delete()
    if not grid:
        vis["/Grid"].set_property("visible", False)
    else:
        vis["/Grid"].set_property("visible", True)
    if not axes:
        vis["/Axes"].set_property("visible", False)
    else:
        vis["/Axes"].set_property("visible", True)
    return vis

def get_transform(T_: hppfcl.Transform3f):
    T = np.eye(4)
    if isinstance(T_, hppfcl.Transform3f):
        T[:3, :3] = T_.getRotation()
        T[:3, 3] = T_.getTranslation()
    elif isinstance(T_, pin.SE3):
        T[:3, :3] = T_.rotation
        T[:3, 3] = T_.translation
    else:
        raise NotADirectoryError
    return T

def rgbToHex(color):
    if len(color) == 4:
        c = color[:3]
        opacity = color[3]
    else:
        c = color
        opacity = 1.
    hex_color = '0x%02x%02x%02x' % (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
    return hex_color, opacity

def meshcat_material(r, g, b, a):
    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(r * 255) * 256 ** 2 + int(g * 255) * 256 + \
        int(b * 255)
    material.opacity = a
    return material

def renderPoint(vis: meshcat.Visualizer, point: np.ndarray, point_name: str,
                color=np.ones(4), radius_point=0.001):
    hex_color, opacity = rgbToHex(color)
    vis[point_name].set_object(g.Sphere(radius_point), g.MeshLambertMaterial(color=hex_color, opacity=opacity))
    vis[point_name].set_transform(tf.translation_matrix(point))

def renderCylinder(vis: meshcat.Visualizer, height: float,
                   radius: float, name: str, T_: hppfcl.Transform3f, color = np.array([0., 0., 0., 1.])):
    R = np.array([[1.,  0.,  0.,  0.],
                  [0.,  0., -1.,  0.],
                  [0.,  1.,  0.,  0.],
                  [0.,  0.,  0.,  1.]])
    RotatedCylinder = type("RotatedCylinder", (g.Cylinder,), {"intrinsic_transform": lambda self: R })
    vis[name].set_object(RotatedCylinder(height, radius), meshcat_material(*color))
    T = get_transform(T_)
    vis[name].set_transform(T)

TWOPI = 2 * np.pi
def create_arrow_head(scale_: float = 1., n: int=10) -> hppfcl.Convex:
    scale = scale_ / 2
    pts = hppfcl.StdVec_Vec3f()
    assert(n > 3)
    center = np.zeros(3)
    for i in range(n):
        pt = scale * np.array([np.cos(TWOPI * i / n), np.sin(TWOPI * i / n), 0.])
        center += pt
        pts.append(pt)
    pts.append(center)
    pts.append(scale * np.array([0., 0., 2.]))

    tris = hppfcl.StdVec_Triangle()
    for i in range(n):
        # Base triangle
        tris.append(hppfcl.Triangle(i, n, (i+1)%n))
        # Side triangle
        tris.append(hppfcl.Triangle(i, (i+1)%n, n+1))

    cvx = hppfcl.Convex(pts, tris)
    return cvx

def renderArrowHead(vis: meshcat.Visualizer, scale: float,
                    name: str, T_: hppfcl.Transform3f, color=np.array([0., 0., 0., 1.])):
    arrow = create_arrow_head(scale)
    renderConvex(vis, arrow, name, T_, color)

def renderLine(vis: meshcat.Visualizer, pt1: np.ndarray, pt2: np.ndarray,
               name: str, linewidth: float=1, color: np.ndarray=np.array([0., 0., 0., 1.]), thresh: float=1e-6) -> bool:
    height = np.linalg.norm(pt2 - pt1)
    if height > thresh:
        axis_ref = np.array([0., 0., 1.])
        axis = (pt2 - pt1) / height # - np.array([0., 0., 1.])
        num = np.outer(axis_ref + axis, axis_ref + axis)
        den = np.dot(axis_ref + axis, axis_ref + axis)
        if den > thresh:
            R = 2 * num / den - np.eye(3)
        else:
            R = np.array([[1, 0, 0],
                          [0, -1, 0],
                          [0, 0, -1]])
        t = pt1
        translation = hppfcl.Transform3f(np.eye(3), np.array([0., 0., height / 2]))
        T = hppfcl.Transform3f(R, t)
        Ttot = T * translation
        renderCylinder(vis, height, linewidth, name, Ttot, color)
        return True
    else:
        vis[name].delete()
        return False

def renderArrowLine(vis: meshcat.Visualizer, pt1: np.ndarray, pt2: np.ndarray,
                    name: str, linewidth: float=1, scale_arrow: float=1, color=np.array([0., 0., 0., 1.]), thresh: float=1e-6):
    height = np.linalg.norm(pt2 - pt1)
    if height > thresh:
        renderLine(vis, pt1, pt2, name, linewidth, color)
        T = hppfcl.Transform3f(np.eye(3), np.array([0., 0., height / 2]))
        renderArrowHead(vis, scale_arrow, name + "/arrow", T, color)
        return True
    else:
        vis[name].delete()
        return False

def renderEllipsoid(vis: meshcat.Visualizer, ellipsoid: hppfcl.Ellipsoid,
                    e_name: str, T_: hppfcl.Transform3f,
                    color=np.array([1., 1., 1., 1.])):
    vis[e_name].set_object(g.Ellipsoid(ellipsoid.radii), meshcat_material(*color))
    T = get_transform(T_)
    vis[e_name].set_transform(T)

def renderSphere(vis: meshcat.Visualizer, sphere: hppfcl.Sphere,
                 e_name: str, T_: hppfcl.Transform3f,
                 color=np.array([1., 1., 1., 1.])):
    vis[e_name].set_object(g.Sphere(sphere.radius), meshcat_material(*color))
    T = get_transform(T_)
    vis[e_name].set_transform(T)

def renderConvex(vis: meshcat.Visualizer, cvx: hppfcl.Convex,
                 cvx_name: str, T_: hppfcl.Transform3f,
                 color=np.array([0.2, 0.5, 0.7, 1.]), render_faces=False):
    """
    Given a meshcat visualiser and a hppfcl ConvexBase base, renders a hppfcl convex: set of points, for each point k neighbors.
    """
    mesh = loadmesh(cvx)
    vis[cvx_name].set_object(mesh, meshcat_material(*color))
    # vis[cvx_name].set_object(mesh)
    T = get_transform(T_)
    vis[cvx_name].set_transform(T)
    if render_faces:
        pairs = []
        idx_segment = 0
        def render_segment(w, w_swapped, idx_segment):
            if w not in pairs and w_swapped not in pairs:
                pairs.append(w)
                p1, p2 = cvx.point(w[0]), cvx.point(w[1])
                renderLine(vis, p1, p2, cvx_name + "/" + str(idx_segment), linewidth=10)
                idx_segment += 1
            return idx_segment
        for t in range(cvx.num_polygons):
            tri = cvx.polygons(t)
            w = [tri[0], tri[1]]
            w_swapped = [tri[1], tri[0]]
            idx_segment = render_segment(w, w_swapped, idx_segment)

            w = [tri[1], tri[2]]
            w_swapped = [tri[2], tri[1]]
            idx_segment = render_segment(w, w_swapped, idx_segment)

            w = [tri[2], tri[0]]
            w_swapped = [tri[0], tri[2]]
            idx_segment = render_segment(w, w_swapped, idx_segment)

def loadmesh(mesh: hppfcl.Convex):
    '''
    Taken from pinocchio: https://github.com/stack-of-tasks/pinocchio
    '''
    import meshcat.geometry as mg

    if isinstance(mesh, hppfcl.BVHModelBase):
        num_vertices = mesh.num_vertices
        num_tris = mesh.num_tris

        call_triangles = mesh.tri_indices
        call_vertices = mesh.vertices

    elif isinstance(mesh, hppfcl.Convex):
        num_vertices = mesh.num_points
        num_tris = mesh.num_polygons

        call_triangles = mesh.polygons
        call_vertices = mesh.points

    vertices = np.empty((num_vertices, 3))
    faces = np.empty((num_tris, 3), dtype=int)

    for k in range(num_tris):
        tri = call_triangles(k)
        faces[k] = [tri[i] for i in range(3)]

    for k in range(num_vertices):
        vert = call_vertices(k)
        vertices[k] = vert

    vertices = vertices.astype(np.float32)
    if num_tris > 0:
        mesh = mg.TriangularMeshGeometry(vertices, faces)
    else:
        mesh = mg.Points(mg.PointsGeometry(vertices.T,
            color=np.repeat(np.ones((3, 1)), num_vertices, axis=1)),
            mg.PointsMaterial(size=0.002))
    return mesh
