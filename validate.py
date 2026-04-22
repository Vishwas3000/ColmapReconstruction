"""
validate.py

Validates bounding box alignment against the sparse point cloud and crops
to only the points inside the bounding volume.

Usage:
    python validate.py --workspace ./scenes/ar_capture

Outputs:
    sparse/sparse.ply           — full sparse cloud (for reference)
    sparse/sparse_cropped.ply   — only points inside the bounding box
    sparse/validation.png       — top/front/side view with bounding box drawn
"""

import argparse
import json
import subprocess
import os
import struct
from pathlib import Path

import numpy as np
import open3d as o3d

COLMAP = r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe"
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = \
    r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"


# ─── COLMAP binary reader ─────────────────────────────────────────────────────

def read_points3D_binary(path: Path) -> np.ndarray:
    """Returns (N,3) float32 XYZ array and (N,3) uint8 RGB array."""
    xyzs, rgbs = [], []
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            f.read(8)                              # point3D_id
            xyz  = struct.unpack("<ddd", f.read(24))
            rgb  = struct.unpack("<BBB", f.read(3))
            f.read(8)                              # error
            tlen = struct.unpack("<Q", f.read(8))[0]
            f.read(8 * tlen)                       # track
            xyzs.append(xyz)
            rgbs.append(rgb)
    if not xyzs:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)
    return np.array(xyzs, dtype=np.float32), np.array(rgbs, dtype=np.uint8)


# ─── Bounding box helpers ─────────────────────────────────────────────────────

def load_bounds(bounds_path: Path) -> dict:
    with open(bounds_path) as f:
        b = json.load(f)

    center  = np.array(b["center"],  dtype=np.float64)
    extents = np.array(b["extents"], dtype=np.float64)
    R       = np.array(b["rotation"], dtype=np.float64)   # 3×3, box-local→world

    # No coordinate flip needed:
    # inject_poses.py uses `c2w @ flip` to change camera axes convention,
    # but the world frame stays ARKit world space throughout.
    # The triangulated 3D points ARE in ARKit world space, same as bounds.json.

    return {
        "center":  center,
        "extents": extents,
        "R":       R,
        "raw":     b,
    }


def points_inside_box(xyzs: np.ndarray, bounds: dict) -> np.ndarray:
    """
    Returns boolean mask of points inside the oriented bounding box.
    Works by transforming points into box-local space and doing AABB check.
    """
    center  = bounds["center"]
    extents = bounds["extents"]
    R       = bounds["R"]

    # Translate then rotate into box-local frame.
    # R columns = box axes in world space.
    # For row vectors: world→local = v @ R  (equiv to R.T @ v for column vectors)
    local = (xyzs - center) @ R   # (N,3)

    half = extents / 2.0
    mask = (
        (np.abs(local[:, 0]) <= half[0]) &
        (np.abs(local[:, 1]) <= half[1]) &
        (np.abs(local[:, 2]) <= half[2])
    )
    return mask


def make_box_lineset(bounds: dict) -> o3d.geometry.LineSet:
    """Create an Open3D LineSet showing the bounding box wireframe."""
    c = bounds["center"]
    e = bounds["extents"] / 2.0
    R = bounds["R"]

    # 8 corners in box-local space
    corners_local = np.array([
        [-e[0], -e[1], -e[2]],
        [ e[0], -e[1], -e[2]],
        [-e[0],  e[1], -e[2]],
        [ e[0],  e[1], -e[2]],
        [-e[0], -e[1],  e[2]],
        [ e[0], -e[1],  e[2]],
        [-e[0],  e[1],  e[2]],
        [ e[0],  e[1],  e[2]],
    ])

    # Back to world space
    corners_world = corners_local @ R.T + c

    edges = [
        [0,1],[2,3],[4,5],[6,7],   # along X
        [0,2],[1,3],[4,6],[5,7],   # along Y
        [0,4],[1,5],[2,6],[3,7],   # along Z
    ]

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(corners_world)
    ls.lines  = o3d.utility.Vector2iVector(edges)
    ls.colors = o3d.utility.Vector3dVector([[1, 1, 0]] * len(edges))   # yellow
    return ls


# ─── Camera frustum helpers ───────────────────────────────────────────────────

def read_images_binary(path: Path) -> list[np.ndarray]:
    """Returns list of 4×4 camera-to-world matrices."""
    c2ws = []
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            f.read(4)                              # image_id
            qw, qx, qy, qz = struct.unpack("<dddd", f.read(32))
            tx, ty, tz      = struct.unpack("<ddd",  f.read(24))
            f.read(4)                              # camera_id
            # Read name (null-terminated)
            name = b""
            while True:
                ch = f.read(1)
                if ch == b"\x00": break
                name += ch
            n2d = struct.unpack("<Q", f.read(8))[0]
            f.read(24 * n2d)                       # 2D points

            # W2C → C2W
            R = quaternion_to_matrix(qw, qx, qy, qz)
            t = np.array([tx, ty, tz])
            c2w      = np.eye(4)
            c2w[:3,:3] = R.T
            c2w[:3, 3] = -R.T @ t
            c2ws.append(c2w)
    return c2ws


def quaternion_to_matrix(qw, qx, qy, qz) -> np.ndarray:
    R = np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])
    return R


def make_camera_frustrums(c2ws: list, scale: float = 0.05) -> o3d.geometry.LineSet:
    """Tiny frustum triangles showing camera positions and orientations."""
    points, lines, colors = [], [], []
    idx = 0
    for c2w in c2ws:
        origin = c2w[:3, 3]
        z_tip  = origin + c2w[:3, 2] * scale    # forward direction
        x_tip  = origin + c2w[:3, 0] * scale
        y_tip  = origin + c2w[:3, 1] * scale

        base = idx
        points += [origin, z_tip, x_tip, y_tip]
        lines  += [[base, base+1], [base, base+2], [base, base+3]]
        colors += [[0.4, 0.4, 1.0], [1, 0, 0], [0, 1, 0]]
        idx += 4

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.array(points))
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--source", choices=["sparse", "dense"], default="sparse",
                        help="Which point cloud to validate against")
    args = parser.parse_args()

    workspace   = Path(args.workspace)
    bounds_path = workspace / "bounds.json"
    sparse_dir  = workspace / "sparse" / "0"

    if not bounds_path.exists():
        raise FileNotFoundError(f"bounds.json not found in {workspace}")

    # ── 1. Export sparse PLY if not already done ──────────────────────────────
    sparse_ply = workspace / "sparse" / "sparse.ply"
    if not sparse_ply.exists():
        print("Exporting sparse PLY...")
        subprocess.run([
            COLMAP, "model_converter",
            "--input_path",  str(sparse_dir),
            "--output_path", str(sparse_ply),
            "--output_type", "PLY",
        ], check=True)

    # ── 2. Load point cloud ───────────────────────────────────────────────────
    if args.source == "dense":
        ply_path = workspace / "dense" / "fused.ply"
        if not ply_path.exists():
            raise FileNotFoundError("Dense cloud not found — run --dense-only first")
    else:
        ply_path = sparse_ply

    print(f"Loading {ply_path.name}...")
    xyzs, rgbs = read_points3D_binary(sparse_dir / "points3D.bin")
    print(f"Total points: {len(xyzs):,}")

    if len(xyzs) == 0:
        print("\nERROR: No 3D points in sparse model.")
        print("The point_triangulator needs feature matches — re-run inject_poses.py")
        print("(feature matching step was missing in earlier runs)")
        return

    # ── 3. Load bounds ────────────────────────────────────────────────────────
    bounds = load_bounds(bounds_path)

    # Always print diagnostics so you can verify alignment by eye
    print(f"\nPoint cloud (ARKit world space):")
    print(f"  X: {xyzs[:,0].min():.4f}  to  {xyzs[:,0].max():.4f}  (mean {xyzs[:,0].mean():.4f})")
    print(f"  Y: {xyzs[:,1].min():.4f}  to  {xyzs[:,1].max():.4f}  (mean {xyzs[:,1].mean():.4f})")
    print(f"  Z: {xyzs[:,2].min():.4f}  to  {xyzs[:,2].max():.4f}  (mean {xyzs[:,2].mean():.4f})")
    c = bounds["center"]
    e = bounds["extents"]
    print(f"\nBounding box (ARKit world space):")
    print(f"  Center:  [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]")
    print(f"  Extents: [{e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f}]")
    print(f"  X range: {c[0]-e[0]/2:.4f}  to  {c[0]+e[0]/2:.4f}")
    print(f"  Y range: {c[1]-e[1]/2:.4f}  to  {c[1]+e[1]/2:.4f}")
    print(f"  Z range: {c[2]-e[2]/2:.4f}  to  {c[2]+e[2]/2:.4f}")

    # ── 4. Crop ───────────────────────────────────────────────────────────────
    mask    = points_inside_box(xyzs, bounds)
    n_in    = mask.sum()
    n_total = len(xyzs)
    pct     = 100 * n_in / max(n_total, 1)

    print(f"\nPoints inside bounding box: {n_in:,} / {n_total:,}  ({pct:.1f}%)")

    if n_in == 0:
        print("\nWARNING: 0 points inside box — check if bbox ranges overlap point cloud ranges above.")
    else:
        cropped_xyzs = xyzs[mask]
        cropped_rgbs = rgbs[mask]

        # Save cropped PLY
        cropped_path = workspace / "sparse" / "sparse_cropped.ply"
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cropped_xyzs.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(cropped_rgbs.astype(np.float64) / 255.0)
        o3d.io.write_point_cloud(str(cropped_path), pcd)
        print(f"\nCropped cloud saved → {cropped_path}")

    # ── 5. Visualize ─────────────────────────────────────────────────────────
    print("\nOpening visualizer...")
    print("  Yellow box = bounding box")
    print("  Blue lines = camera forward direction")
    print("  White/grey = all points  |  Green = points inside box")

    # Full cloud (grey)
    full_pcd = o3d.geometry.PointCloud()
    full_pcd.points = o3d.utility.Vector3dVector(xyzs.astype(np.float64))
    full_pcd.colors = o3d.utility.Vector3dVector(
        np.tile([0.6, 0.6, 0.6], (len(xyzs), 1))
    )

    # Cropped cloud (green)
    inside_pcd = o3d.geometry.PointCloud()
    inside_pts = xyzs[mask]
    inside_pcd.points = o3d.utility.Vector3dVector(inside_pts.astype(np.float64))
    inside_pcd.colors = o3d.utility.Vector3dVector(
        np.tile([0.0, 1.0, 0.2], (len(inside_pts), 1))
    )

    # Bounding box wireframe
    box_ls = make_box_lineset(bounds)

    # Camera positions
    c2ws   = read_images_binary(sparse_dir / "images.bin")
    cam_ls = make_camera_frustrums(c2ws, scale=0.04)

    # Origin axes
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    o3d.visualization.draw_geometries(
        [full_pcd, inside_pcd, box_ls, cam_ls, axes],
        window_name="Validation — yellow=bbox  green=inside  blue=cameras",
        width=1280,
        height=720,
    )


if __name__ == "__main__":
    main()
