import argparse
import numpy as np
import open3d as o3d
from pathlib import Path


def load_point_cloud(ply_path):
    pcd = o3d.io.read_point_cloud(str(ply_path))
    if len(pcd.points) == 0:
        raise ValueError(f"Point cloud is empty or could not be read: {ply_path}")
    print(f"Loaded {len(pcd.points):,} points from {ply_path}")
    return pcd


def downsample(pcd, voxel_size=0.02):
    down = pcd.voxel_down_sample(voxel_size)
    print(f"Downsampled to {len(down.points):,} points (voxel size={voxel_size})")
    return down


def ransac_plane(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000):
    """Fit the dominant plane via RANSAC. Returns plane normal and inlier cloud."""
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )
    a, b, c, d = plane_model
    normal = np.array([a, b, c], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    print(f"RANSAC plane: [{a:.3f}, {b:.3f}, {c:.3f}, {d:.3f}]  inliers={len(inliers)}")
    return normal, inliers


def pca_axes(pcd):
    """Return eigenvectors sorted by descending eigenvalue (largest variance first)."""
    pts = np.asarray(pcd.points)
    centered = pts - pts.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # eigh returns ascending order — reverse to get descending
    idx = np.argsort(eigenvalues)[::-1]
    return eigenvectors[:, idx]   # columns are eigenvectors


def build_transform(up, pcd):
    """
    Build a 4x4 rigid transform that:
      - aligns `up` → world Z
      - aligns the longest in-plane axis → world X
      - centers the cloud at the origin
    """
    up = up / np.linalg.norm(up)

    # Ensure Z points upward (not downward)
    if up[1] < 0:
        up = -up

    # Project points onto the plane perpendicular to `up` and run PCA
    pts = np.asarray(pcd.points)
    proj = pts - np.outer(pts @ up, up)
    proj_pcd = o3d.geometry.PointCloud()
    proj_pcd.points = o3d.utility.Vector3dVector(proj)
    axes = pca_axes(proj_pcd)

    x_axis = axes[:, 0]   # longest in-plane direction
    y_axis = np.cross(up, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, up)
    x_axis /= np.linalg.norm(x_axis)

    # Rotation: columns are the new basis vectors expressed in world coords
    R = np.column_stack([x_axis, y_axis, up])   # shape (3, 3)

    center = pts.mean(axis=0)

    T = np.eye(4)
    T[:3, :3] = R.T
    T[:3, 3] = -R.T @ center

    return T, R


def apply_and_save(pcd, T, output_path):
    aligned = o3d.geometry.PointCloud(pcd)
    aligned.transform(T)
    o3d.io.write_point_cloud(str(output_path), aligned)
    print(f"Saved aligned cloud → {output_path}")

    pts = np.asarray(aligned.points)
    print(f"Aligned bounds:")
    print(f"  X: {pts[:,0].min():.3f}  to  {pts[:,0].max():.3f}")
    print(f"  Y: {pts[:,1].min():.3f}  to  {pts[:,1].max():.3f}")
    print(f"  Z: {pts[:,2].min():.3f}  to  {pts[:,2].max():.3f}")
    return aligned


def save_transform(T, path):
    np.savetxt(str(path), T, fmt="%.8f")
    print(f"Saved transform matrix → {path}")


def main():
    parser = argparse.ArgumentParser(description="Align a point cloud to world axes using RANSAC plane + PCA")
    parser.add_argument("--workspace",  required=True, help="Scene workspace folder")
    parser.add_argument("--source",     choices=["dense", "sparse"], default="dense",
                        help="Which PLY to align: dense=fused.ply, sparse=sparse.ply")
    parser.add_argument("--method",     choices=["ransac", "pca"], default="ransac",
                        help="ransac: use dominant plane as Up (best for scenes with floor/table). "
                             "pca: use smallest-variance axis as Up (best for compact objects)")
    parser.add_argument("--voxel-size", type=float, default=0.02,
                        help="Voxel size for downsampling before plane fitting (default 0.02). "
                             "Increase for large scenes, decrease for small objects.")
    parser.add_argument("--ransac-threshold", type=float, default=0.02,
                        help="Max distance from plane to count as inlier (default 0.02)")
    args = parser.parse_args()

    workspace = Path(args.workspace)

    if args.source == "dense":
        ply_path = workspace / "dense" / "fused.ply"
    else:
        ply_path = workspace / "sparse" / "sparse.ply"

    if not ply_path.exists():
        raise FileNotFoundError(f"PLY not found: {ply_path}\n"
                                f"Run the pipeline first or use --export-sparse-ply for sparse.")

    output_path   = ply_path.parent / f"{ply_path.stem}_aligned.ply"
    transform_path = ply_path.parent / "transform.txt"

    pcd = load_point_cloud(ply_path)
    down = downsample(pcd, voxel_size=args.voxel_size)

    if args.method == "ransac":
        print("\nFitting dominant plane with RANSAC...")
        up, _ = ransac_plane(down, distance_threshold=args.ransac_threshold)
    else:
        print("\nComputing PCA axes...")
        axes = pca_axes(down)
        up = axes[:, 2]   # smallest eigenvalue = least variance = plane normal

    print(f"\nUp vector: {up}")
    T, R = build_transform(up, pcd)

    apply_and_save(pcd, T, output_path)
    save_transform(T, transform_path)

    print(f"\nDone. To view the result:")
    print(f"  .\\view_ply.ps1 -workspace {args.workspace} -type {args.source}_aligned")


if __name__ == "__main__":
    main()
