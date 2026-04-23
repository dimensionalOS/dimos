#!/usr/bin/env python3
"""Parse M20 URDF and compute a body-frame AABB for lidar self-filtering.

Strategy:
  - Walk the link/joint tree in base_link frame at neutral (zero-joint) pose
  - Transform each link's collision geometry to base_link
  - Compute per-link AABB + overall AABB
  - Report recommended crop box with safety margin

Joints at zero angle give the nominal standing pose. Leg swings during
gait/rotation are accounted for by inflating the Y and Z axes of the
output box.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy_, sy_ = math.cos(y), math.sin(y)
    Rx = [[1,0,0],[0,cr,-sr],[0,sr,cr]]
    Ry = [[cp,0,sp],[0,1,0],[-sp,0,cp]]
    Rz = [[cy_,-sy_,0],[sy_,cy_,0],[0,0,1]]
    return matmul(Rz, matmul(Ry, Rx))


def matmul(A, B):
    n = len(A); m = len(B[0]); k = len(B)
    return [[sum(A[i][x]*B[x][j] for x in range(k)) for j in range(m)] for i in range(n)]


def matvec(R, v):
    return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
            R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
            R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]


def compose(T1, T2):
    """Compose two (R, t) transforms: T1 then T2 → result in parent frame."""
    R1, t1 = T1
    R2, t2 = T2
    R = matmul(R1, R2)
    t = [t1[0]+matvec(R1,t2)[0], t1[1]+matvec(R1,t2)[1], t1[2]+matvec(R1,t2)[2]]
    return (R, t)


def identity():
    return ([[1,0,0],[0,1,0],[0,0,1]], [0,0,0])


def parse_origin(elem):
    """Parse an <origin> element, return (R, t)."""
    if elem is None or elem.find("origin") is None:
        return identity()
    o = elem.find("origin")
    xyz = [float(x) for x in (o.get("xyz","0 0 0").split())]
    rpy = [float(x) for x in (o.get("rpy","0 0 0").split())]
    return (rpy_to_R(rpy), xyz)


def box_corners_in_parent(size, T_shape):
    """Given a box with local size (sx,sy,sz) and local transform T_shape in
    the parent link's frame, return the 8 corner coords in parent frame."""
    sx, sy, sz = [s/2 for s in size]
    corners_local = [
        [x, y, z]
        for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)
    ]
    R, t = T_shape
    return [[
        R[0][0]*c[0]+R[0][1]*c[1]+R[0][2]*c[2]+t[0],
        R[1][0]*c[0]+R[1][1]*c[1]+R[1][2]*c[2]+t[1],
        R[2][0]*c[0]+R[2][1]*c[1]+R[2][2]*c[2]+t[2],
    ] for c in corners_local]


def cylinder_bounding_points(length, radius, T_shape, n_axial=2, n_radial=16):
    """Approximate cylinder's convex hull via points on its surface. Default
    cylinder aligned along local Z; if the shape's R rotates it elsewhere,
    those axes transform accordingly."""
    pts = []
    for z in (-length/2, length/2):
        for th in range(n_radial):
            ang = 2 * math.pi * th / n_radial
            pts.append([radius * math.cos(ang), radius * math.sin(ang), z])
    R, t = T_shape
    return [[
        R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2]+t[0],
        R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2]+t[1],
        R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2]+t[2],
    ] for p in pts]


def main():
    urdf = Path("/Users/afik_cohen/gt/dimos/crew/ace/plans/m20-rosnav-migration/M20_high_res.urdf")
    tree = ET.parse(urdf)
    root = tree.getroot()

    # Index links + joints
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")

    # Build parent-child map + joint transforms
    parent_of = {}   # child_link → (parent_link, joint_origin_transform)
    for j in joints:
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        T = parse_origin(j)
        parent_of[child] = (parent, T)

    # Compute each link's transform in base_link frame (at zero joint angles)
    link_T = {"base_link": identity()}
    def resolve(name):
        if name in link_T:
            return link_T[name]
        parent_name, T_joint = parent_of[name]
        T_parent = resolve(parent_name)
        T = compose(T_parent, T_joint)
        link_T[name] = T
        return T
    for lname in links:
        resolve(lname)

    # Walk all collision shapes, collect world-frame bounding points
    all_pts = []
    per_link = {}
    for lname, link in links.items():
        T_link = link_T[lname]
        link_pts = []
        for col in link.findall("collision"):
            T_shape_local = parse_origin(col)
            T_shape = compose(T_link, T_shape_local)
            geom = col.find("geometry")
            if geom is None:
                continue
            b = geom.find("box")
            c = geom.find("cylinder")
            s = geom.find("sphere")
            if b is not None:
                size = [float(x) for x in b.get("size").split()]
                pts = box_corners_in_parent(size, T_shape)
                link_pts.extend(pts)
            elif c is not None:
                L = float(c.get("length"))
                R = float(c.get("radius"))
                pts = cylinder_bounding_points(L, R, T_shape)
                link_pts.extend(pts)
            elif s is not None:
                R = float(s.get("radius"))
                R_tot, t = T_shape
                for dx in (-R, R):
                    for dy in (-R, R):
                        for dz in (-R, R):
                            link_pts.append([t[0]+dx, t[1]+dy, t[2]+dz])
        per_link[lname] = link_pts
        all_pts.extend(link_pts)

    print(f"=== Per-link AABBs (base_link frame, zero-joint pose) ===")
    for lname, pts in per_link.items():
        if not pts: continue
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        print(f"  {lname:12s}: "
              f"X[{min(xs):+.3f}..{max(xs):+.3f}] "
              f"Y[{min(ys):+.3f}..{max(ys):+.3f}] "
              f"Z[{min(zs):+.3f}..{max(zs):+.3f}]")

    if not all_pts:
        print("no collision pts"); return
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    zs = [p[2] for p in all_pts]

    print(f"\n=== Overall AABB (zero-joint pose) ===")
    print(f"  X: [{min(xs):+.3f}, {max(xs):+.3f}]  width = {max(xs)-min(xs):.3f} m")
    print(f"  Y: [{min(ys):+.3f}, {max(ys):+.3f}]  width = {max(ys)-min(ys):.3f} m")
    print(f"  Z: [{min(zs):+.3f}, {max(zs):+.3f}]  width = {max(zs)-min(zs):.3f} m")

    # Recommended crop: inflate to account for leg swings during gait.
    # Legs can kick out laterally (Y) and up/down (Z). X dimension is
    # dominated by hip positions — static.
    x_pad = 0.05
    y_pad = 0.10   # leg swing envelope
    z_pad = 0.15   # wheel drop + leg raise
    print(f"\n=== Recommended body-crop AABB (with padding {x_pad}/{y_pad}/{z_pad}) ===")
    print(f"  X: [{min(xs)-x_pad:+.3f}, {max(xs)+x_pad:+.3f}]")
    print(f"  Y: [{min(ys)-y_pad:+.3f}, {max(ys)+y_pad:+.3f}]")
    print(f"  Z: [{min(zs)-z_pad:+.3f}, {max(zs)+z_pad:+.3f}]")

    # Also: what does blind=1.0 (sphere from front lidar at +0.320) cover?
    # Compare.
    print(f"\n=== Comparison vs current blind=1.0 (sphere around lidar @ (+0.320,0,-0.013)) ===")
    lidar_pos = (0.320, 0, -0.013)
    blind = 1.0
    outside_sphere = sum(1 for p in all_pts if
        math.sqrt((p[0]-lidar_pos[0])**2+(p[1]-lidar_pos[1])**2+(p[2]-lidar_pos[2])**2) > blind)
    total = len(all_pts)
    print(f"  body collision pts outside blind={blind}m sphere: {outside_sphere}/{total}")
    print(f"  (pts outside = body points that SLIP through the spherical blind)")

    # Which link points slip through?
    print(f"\n=== Body points outside blind=1.0 sphere (per link) ===")
    for lname, pts in per_link.items():
        if not pts: continue
        out = sum(1 for p in pts if math.sqrt((p[0]-lidar_pos[0])**2+(p[1]-lidar_pos[1])**2+(p[2]-lidar_pos[2])**2) > blind)
        if out > 0:
            print(f"  {lname:12s}: {out:3d} pts slip through")


if __name__ == "__main__":
    main()
