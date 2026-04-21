"""
crop.py

Crops the COLMAP sparse model to only points within the bounding box
defined in bounds.json, writes a new binary model, then runs dense
reconstruction on that cropped volume.

Usage:
    # Crop sparse model + run full dense pipeline
    python crop.py --workspace ./scenes/ar_capture

    # Skip re-cropping — just re-run dense on existing sparse/cropped/
    python crop.py --workspace ./scenes/ar_capture --dense-only
"""

import argparse
import json
import os
import struct
import subprocess
from pathlib import Path

import numpy as np

COLMAP = r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe"
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = \
    r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"

# Number of intrinsic params per COLMAP camera model ID
_MODEL_PARAMS = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 12, 7: 5, 8: 3, 9: 4, 10: 12}


# ─── Binary readers ───────────────────────────────────────────────────────────

def read_cameras_bin(path: Path) -> dict:
    cams = {}
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            cid      = struct.unpack("<I", f.read(4))[0]
            model_id = struct.unpack("<i", f.read(4))[0]
            w        = struct.unpack("<Q", f.read(8))[0]
            h        = struct.unpack("<Q", f.read(8))[0]
            np_      = _MODEL_PARAMS[model_id]
            params   = struct.unpack(f"<{np_}d", f.read(8 * np_))
            cams[cid] = {"id": cid, "model_id": model_id,
                         "width": w, "height": h, "params": list(params)}
    return cams


def read_images_bin(path: Path) -> dict:
    imgs = {}
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            iid    = struct.unpack("<I", f.read(4))[0]
            qvec   = struct.unpack("<4d", f.read(32))
            tvec   = struct.unpack("<3d", f.read(24))
            cid    = struct.unpack("<I", f.read(4))[0]
            name   = b""
            while True:
                ch = f.read(1)
                if ch == b"\x00": break
                name += ch
            n2d = struct.unpack("<Q", f.read(8))[0]
            xys, p3d_ids = [], []
            for _ in range(n2d):
                x, y   = struct.unpack("<2d", f.read(16))
                p3d_id = struct.unpack("<q",  f.read(8))[0]
                xys.append((x, y))
                p3d_ids.append(p3d_id)
            imgs[iid] = {"id": iid, "qvec": qvec, "tvec": tvec,
                         "camera_id": cid, "name": name.decode(),
                         "xys": xys, "point3D_ids": p3d_ids}
    return imgs


def read_points3D_bin(path: Path) -> dict:
    pts = {}
    with open(path, "rb") as f:
        n = struct.unpack("<Q", f.read(8))[0]
        for _ in range(n):
            pid   = struct.unpack("<Q", f.read(8))[0]
            xyz   = struct.unpack("<3d", f.read(24))
            rgb   = struct.unpack("<3B", f.read(3))
            err   = struct.unpack("<d",  f.read(8))[0]
            tlen  = struct.unpack("<Q",  f.read(8))[0]
            track = [struct.unpack("<2I", f.read(8)) for _ in range(tlen)]
            pts[pid] = {"id": pid, "xyz": xyz, "rgb": rgb,
                        "error": err, "track": track}
    return pts


# ─── Binary writers ───────────────────────────────────────────────────────────

def write_cameras_bin(path: Path, cameras: dict):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(cameras)))
        for cam in cameras.values():
            np_ = len(cam["params"])
            f.write(struct.pack("<I",       cam["id"]))
            f.write(struct.pack("<i",       cam["model_id"]))
            f.write(struct.pack("<Q",       cam["width"]))
            f.write(struct.pack("<Q",       cam["height"]))
            f.write(struct.pack(f"<{np_}d", *cam["params"]))


def write_images_bin(path: Path, images: dict):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(images)))
        for img in images.values():
            f.write(struct.pack("<I",  img["id"]))
            f.write(struct.pack("<4d", *img["qvec"]))
            f.write(struct.pack("<3d", *img["tvec"]))
            f.write(struct.pack("<I",  img["camera_id"]))
            f.write(img["name"].encode() + b"\x00")
            f.write(struct.pack("<Q", len(img["xys"])))
            for (x, y), p3d_id in zip(img["xys"], img["point3D_ids"]):
                f.write(struct.pack("<2d", x, y))
                f.write(struct.pack("<q",  p3d_id))


def write_points3D_bin(path: Path, points: dict):
    with open(path, "wb") as f:
        f.write(struct.pack("<Q", len(points)))
        for pt in points.values():
            f.write(struct.pack("<Q",  pt["id"]))
            f.write(struct.pack("<3d", *pt["xyz"]))
            f.write(struct.pack("<3B", *pt["rgb"]))
            f.write(struct.pack("<d",  pt["error"]))
            f.write(struct.pack("<Q",  len(pt["track"])))
            for img_id, p2d_idx in pt["track"]:
                f.write(struct.pack("<2I", img_id, p2d_idx))


# ─── Bounding box ─────────────────────────────────────────────────────────────

def load_bounds(path: Path) -> dict:
    with open(path) as f:
        b = json.load(f)
    return {
        "center":  np.array(b["center"],   dtype=np.float64),
        "extents": np.array(b["extents"],  dtype=np.float64),
        "R":       np.array(b["rotation"], dtype=np.float64),
    }


def inside_box(xyz_array: np.ndarray, bounds: dict) -> np.ndarray:
    c, e, R = bounds["center"], bounds["extents"], bounds["R"]
    local = (xyz_array - c) @ R.T
    half  = e / 2.0
    return (
        (np.abs(local[:, 0]) <= half[0]) &
        (np.abs(local[:, 1]) <= half[1]) &
        (np.abs(local[:, 2]) <= half[2])
    )


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace",  required=True)
    parser.add_argument("--sparse-only", action="store_true",
                        help="Crop sparse model only — do not run dense reconstruction")
    parser.add_argument("--dense-only", action="store_true",
                        help="Skip cropping — re-run dense on existing sparse/cropped/")
    parser.add_argument("--min-obs",    type=int, default=2,
                        help="Min inside-box point observations to keep an image (default 2)")
    args = parser.parse_args()

    workspace  = Path(args.workspace)
    sparse_dir = workspace / "sparse" / "0"
    crop_dir   = workspace / "sparse" / "cropped"
    dense_dir  = workspace / "dense_cropped"
    images_dir = workspace / "images"

    # ── 1. Crop sparse model ──────────────────────────────────────────────────
    if not args.dense_only:
        bounds = load_bounds(workspace / "bounds.json")

        print("Reading sparse model...")
        cameras = read_cameras_bin(sparse_dir / "cameras.bin")
        images  = read_images_bin( sparse_dir / "images.bin")
        points  = read_points3D_bin(sparse_dir / "points3D.bin")
        print(f"  {len(cameras)} cameras  |  {len(images)} images  |  {len(points):,} points")

        # Filter points to bbox
        xyz_arr  = np.array([p["xyz"] for p in points.values()])
        id_arr   = np.array([p["id"]  for p in points.values()])
        mask     = inside_box(xyz_arr, bounds)
        inside_ids  = set(id_arr[mask].tolist())
        kept_points = {pid: points[pid] for pid in inside_ids}

        print(f"\nPoints inside bbox: {len(kept_points):,} / {len(points):,}  "
              f"({100 * len(kept_points) / max(len(points), 1):.1f}%)")

        if len(kept_points) == 0:
            print("ERROR: 0 points inside bbox — run validate.py first to check alignment.")
            return

        # Update images: null out point3D references to outside-box points,
        # drop images with fewer than --min-obs surviving observations
        kept_images = {}
        for img in images.values():
            new_p3d_ids = [
                pid if pid in inside_ids else -1
                for pid in img["point3D_ids"]
            ]
            n_obs = sum(1 for pid in new_p3d_ids if pid != -1)
            if n_obs >= args.min_obs:
                kept_images[img["id"]] = {**img, "point3D_ids": new_p3d_ids}

        print(f"Images with >= {args.min_obs} inside-box observations: "
              f"{len(kept_images)} / {len(images)}")

        kept_cameras = {
            img["camera_id"]: cameras[img["camera_id"]]
            for img in kept_images.values()
        }

        # Write cropped binary model
        crop_dir.mkdir(parents=True, exist_ok=True)
        write_cameras_bin( crop_dir / "cameras.bin",  kept_cameras)
        write_images_bin(  crop_dir / "images.bin",   kept_images)
        write_points3D_bin(crop_dir / "points3D.bin", kept_points)

        print(f"\nCropped model → {crop_dir}")
        print(f"  {len(kept_cameras)} cameras | {len(kept_images)} images | {len(kept_points):,} points")

        if args.sparse_only:
            print("\nSparse crop done. To run dense later:")
            print(f"  python crop.py --workspace {workspace} --dense-only")
            return

    # ── 2. Dense reconstruction on cropped model ──────────────────────────────
    print("\n--- Undistorting images for cropped model ---")
    dense_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        COLMAP, "image_undistorter",
        "--image_path",  str(images_dir),
        "--input_path",  str(crop_dir),
        "--output_path", str(dense_dir),
        "--output_type", "COLMAP",
    ], check=True)

    print("\n--- PatchMatch stereo (GPU) ---")
    subprocess.run([
        COLMAP, "patch_match_stereo",
        "--workspace_path",   str(dense_dir),
        "--workspace_format", "COLMAP",
        "--PatchMatchStereo.geom_consistency", "true",
    ], check=True)

    print("\n--- Stereo fusion ---")
    subprocess.run([
        COLMAP, "stereo_fusion",
        "--workspace_path",   str(dense_dir),
        "--workspace_format", "COLMAP",
        "--input_type",       "geometric",
        "--output_path",      str(dense_dir / "fused.ply"),
    ], check=True)

    # ── 3. Clip fused.ply to bounding box ────────────────────────────────────
    # Depth maps cover the full image frame, so fused.ply still contains points
    # outside the bbox. Filter them out here.
    print("\n--- Clipping fused.ply to bounding box ---")
    import open3d as o3d

    bounds   = load_bounds(workspace / "bounds.json")
    pcd      = o3d.io.read_point_cloud(str(dense_dir / "fused.ply"))
    xyz      = np.asarray(pcd.points)
    mask     = inside_box(xyz, bounds)
    n_before = len(xyz)
    n_after  = mask.sum()

    clipped = pcd.select_by_index(np.where(mask)[0])
    o3d.io.write_point_cloud(str(dense_dir / "fused.ply"), clipped)

    print(f"  {n_before:,} → {n_after:,} points  ({100*n_after/max(n_before,1):.1f}% kept)")
    print(f"\nDone — dense cloud → {dense_dir / 'fused.ply'}")
    print(f"Open: .\\view_ply.ps1 -workspace {workspace} -type dense_cropped")


if __name__ == "__main__":
    main()
