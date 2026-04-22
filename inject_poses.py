"""
inject_poses.py

Converts an objectCapture-iOS session export into a COLMAP workspace.
Instead of running feature_matching + mapper, it injects the AR-recorded
camera poses directly, then uses point_triangulator to build the sparse model.

Usage:
    python inject_poses.py --session ./session_1234 --workspace ./scenes/myscan

Output workspace structure:
    scenes/myscan/
      images/             ← copied from session frames/
      database.db         ← created by COLMAP feature_extractor
      sparse/known/       ← cameras.txt + images.txt from AR poses
      sparse/0/           ← triangulated points3D (output of point_triangulator)
"""

import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

COLMAP = r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe"


# ─── Coordinate conversion ────────────────────────────────────────────────────

def arkit_to_colmap(c2w: np.ndarray) -> np.ndarray:
    """
    Convert ARKit Camera-to-World (Y-up, -Z forward) to
    COLMAP World-to-Camera (Y-down, +Z forward).

    Steps:
      1. Flip Y and Z axes (ARKit -> OpenCV/COLMAP sensor convention)
      2. Invert to get World-to-Camera
    """
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    c2w_colmap = c2w @ flip
    return np.linalg.inv(c2w_colmap)   # World-to-Camera (W2C)


# ─── COLMAP text format writers ───────────────────────────────────────────────

def write_cameras_txt(path: Path, intrinsics_list: list[dict], image_size: tuple[int, int]):
    """Write COLMAP cameras.txt — one camera per frame (PINHOLE model)."""
    with open(path, "w") as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        for cam in intrinsics_list:
            cid   = cam["camera_id"]
            fx    = cam["fx"]
            fy    = cam["fy"]
            cx    = cam["cx"]
            cy    = cam["cy"]
            w, h  = image_size
            f.write(f"{cid} PINHOLE {w} {h} {fx} {fy} {cx} {cy}\n")


def write_images_txt(path: Path, images_list: list[dict]):
    """
    Write COLMAP images.txt — one entry per image with W2C pose.
    Each entry is two lines: pose line + empty keypoints line.
    """
    with open(path, "w") as f:
        f.write("# Image list with two lines of data per image:\n")
        f.write("# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
        f.write("# POINTS2D[] as (X, Y, POINT3D_ID)\n")

        for img in images_list:
            iid    = img["image_id"]
            name   = img["name"]
            cid    = img["camera_id"]
            w2c    = img["w2c"]           # 4×4 numpy array

            R = w2c[:3, :3]
            t = w2c[:3,  3]
            q = rotation_matrix_to_quaternion(R)   # qw, qx, qy, qz

            f.write(f"{iid} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} "
                    f"{t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {cid} {name}\n")
            f.write("\n")   # empty keypoints line


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3×3 rotation matrix to quaternion [qw, qx, qy, qz]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s  = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s  = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return np.array([qw, qx, qy, qz])


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session",   required=True, help="Path to exported iOS session folder")
    parser.add_argument("--workspace", required=True, help="Target COLMAP workspace folder")
    args = parser.parse_args()

    session   = Path(args.session)
    workspace = Path(args.workspace)

    frames_dir   = session / "frames"
    metadata_dir = session / "metadata"
    bounds_file  = session / "bounds.json"

    if not frames_dir.exists():
        raise FileNotFoundError(f"frames/ not found in session: {session}")

    # ── 1. Copy images to workspace ──────────────────────────────────────────
    images_dir = workspace / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    frame_files = sorted(frames_dir.glob("frame_*.jpg"))
    print(f"Found {len(frame_files)} frames")
    for f in frame_files:
        shutil.copy2(f, images_dir / f.name)

    # ── 2. Parse metadata ────────────────────────────────────────────────────
    cameras_list = []
    images_list  = []
    image_size   = None
    shared_intr  = None

    for i, frame_path in enumerate(frame_files):
        stem      = frame_path.stem.replace("frame_", "")
        meta_path = metadata_dir / f"metadata_{stem}.json"

        if not meta_path.exists():
            print(f"  Warning: no metadata for {frame_path.name}, skipping")
            continue

        with open(meta_path) as f:
            meta = json.load(f)

        if image_size is None:
            # Use raw sensor dimensions — COLMAP reads pixel data directly, ignores EXIF rotation
            image_size = (meta["image_width"], meta["image_height"])

        if shared_intr is None:
            intr = meta["intrinsics"]
            # Use intrinsics as-is — they are already in sensor (landscape) orientation
            shared_intr = {
                "fx": intr["fx"], "fy": intr["fy"],
                "cx": intr["cx"], "cy": intr["cy"],
            }

        c2w = np.array(meta["camera_pose_c2w"], dtype=np.float64)
        w2c = arkit_to_colmap(c2w)

        iid = i + 1   # COLMAP IDs are 1-indexed
        images_list.append({
            "image_id":  iid,
            "camera_id": 1,       # all frames share one camera
            "name":      frame_path.name,
            "w2c":       w2c,
        })

    # One shared PINHOLE camera for all frames
    cameras_list = [{"camera_id": 1, **shared_intr}]

    print(f"Parsed {len(images_list)} poses  |  fx={shared_intr['fx']:.1f}  fy={shared_intr['fy']:.1f}")

    # ── 3. Write COLMAP text files ────────────────────────────────────────────
    known_dir = workspace / "sparse" / "known"
    known_dir.mkdir(parents=True, exist_ok=True)

    write_cameras_txt(known_dir / "cameras.txt",  cameras_list, image_size)
    write_images_txt( known_dir / "images.txt",   images_list)

    # Empty points3D.txt required by COLMAP
    (known_dir / "points3D.txt").write_text(
        "# 3D point list — empty, will be filled by point_triangulator\n"
    )

    print(f"Wrote cameras.txt + images.txt to {known_dir}")

    # ── 4. COLMAP feature extraction ─────────────────────────────────────────
    db_path = workspace / "database.db"
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = \
        r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"

    # Pass our known intrinsics so COLMAP creates PINHOLE cameras — must match cameras.txt
    cam_params = f"{shared_intr['fx']},{shared_intr['fy']},{shared_intr['cx']},{shared_intr['cy']}"
    print(f"\nRunning feature extraction (PINHOLE  params={cam_params})...")
    subprocess.run([
        COLMAP, "feature_extractor",
        "--database_path",                str(db_path),
        "--image_path",                   str(images_dir),
        "--ImageReader.camera_model",     "PINHOLE",
        "--ImageReader.camera_params",    cam_params,
        "--ImageReader.single_camera",    "1",
    ], check=True)

    # ── 5. Import known poses into database ──────────────────────────────────
    print("\nImporting known poses into database...")
    out_dir = workspace / "sparse" / "0"
    out_dir.mkdir(parents=True, exist_ok=True)

    subprocess.run([
        COLMAP, "model_converter",
        "--input_path",  str(known_dir),
        "--output_path", str(known_dir),
        "--output_type", "BIN",
    ], check=True)

    # ── 5b. Feature matching (sequential — AR video sequence) ────────────────
    print("\nRunning sequential feature matching...")
    subprocess.run([
        COLMAP, "sequential_matcher",
        "--database_path", str(db_path),
    ], check=True)

    # ── 6. Triangulate points ─────────────────────────────────────────────────
    print("\nTriangulating 3D points...")
    subprocess.run([
        COLMAP, "point_triangulator",
        "--database_path", str(db_path),
        "--image_path",    str(images_dir),
        "--input_path",    str(known_dir),
        "--output_path",   str(out_dir),
        # Minimal BA — 1 iteration won't drift injected ARKit poses meaningfully.
        "--Mapper.ba_global_max_num_iterations", "1",
        "--Mapper.ba_local_max_num_iterations",  "1",
    ], check=True)

    print(f"\nDone. Sparse model with AR poses -> {out_dir}")
    print(f"Run dense reconstruction:")
    print(f"  python main.py --workspace {workspace} --dense-only")

    # ── 7. Copy bounds.json ───────────────────────────────────────────────────
    if bounds_file.exists():
        shutil.copy2(bounds_file, workspace / "bounds.json")
        print(f"Copied bounds.json -> {workspace / 'bounds.json'}")


if __name__ == "__main__":
    main()
