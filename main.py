import argparse
import os
import subprocess
from pathlib import Path

# ==============================================================================
# COLMAP Pipeline Scaffolding
# ==============================================================================
# As your mentor, I've set up this structural outline for you.
# Your task is to use the `subprocess` module to call the COLMAP executable
# for each of the steps below. Let's start with Feature Extraction!
# ==============================================================================

# Directory configurations
BASE_DIR = Path(__file__).parent.resolve()
WORKSPACE_DIR = BASE_DIR / "workspace"
IMAGES_DIR = WORKSPACE_DIR / "images"
SPARSE_DIR = WORKSPACE_DIR / "sparse"
DENSE_DIR = WORKSPACE_DIR / "dense"
DATABASE_PATH = WORKSPACE_DIR / "database.db"

# Ensure colmap CLI tool is accessible (Update this if colmap is not in your PATH)
COLMAP_EXECUTABLE = r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe"
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = r"C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"

def run_colmap_command(args):
    """
    Helper function to run a colmap command safely.
    You will need to implement this using `subprocess.run`.
    Make sure to handle errors appropriately!
    """
    print(f"Running command: {' '.join(args)}")
    subprocess.run(args, check=True)


def feature_extraction():
    """
    Step 1: Extract features from images.
    Command equivalent: colmap feature_extractor --database_path <path> --image_path <path>
    """
    print("--- Starting Feature Extraction ---")
    run_colmap_command([
        COLMAP_EXECUTABLE, "feature_extractor",
        "--database_path", str(DATABASE_PATH),
        "--image_path",    str(IMAGES_DIR),
    ])


def feature_matching(sequential=False):
    """
    Step 2: Match features across images.
    sequential=True  → sequential_matcher (best for video, matches only nearby frames)
    sequential=False → exhaustive_matcher (best for unordered photo sets, all vs all)
    """
    if sequential:
        print("--- Starting Feature Matching (sequential) ---")
        run_colmap_command([
            COLMAP_EXECUTABLE, "sequential_matcher",
            "--database_path", str(DATABASE_PATH),
        ])
    else:
        print("--- Starting Feature Matching (exhaustive) ---")
        run_colmap_command([
            COLMAP_EXECUTABLE, "exhaustive_matcher",
            "--database_path", str(DATABASE_PATH),
        ])


def mapper():
    """
    Step 3: Sparse 3D reconstruction (Structure from Motion).
    Command equivalent: colmap mapper --database_path <path> --image_path <path> --output_path <path>
    """
    print("--- Starting Mapper (SfM) ---")
    SPARSE_DIR.mkdir(parents=True, exist_ok=True)
    run_colmap_command([
        COLMAP_EXECUTABLE, "mapper",
        "--database_path", str(DATABASE_PATH),
        "--image_path",    str(IMAGES_DIR),
        "--output_path",   str(SPARSE_DIR),
    ])


def export_sparse_ply():
    """
    Export sparse point cloud to sparse/sparse.ply for viewing in MeshLab.
    """
    print("--- Exporting Sparse PLY ---")
    sparse_model = sorted(SPARSE_DIR.iterdir())[0]
    run_colmap_command([
        COLMAP_EXECUTABLE, "model_converter",
        "--input_path",  str(sparse_model),
        "--output_path", str(SPARSE_DIR / "sparse.ply"),
        "--output_type", "PLY",
    ])


def export_cropped_ply():
    """
    Export bbox-cropped sparse model (sparse/cropped/) to sparse/sparse_cropped.ply.
    Requires crop.py --sparse-only to have been run first.
    """
    print("--- Exporting Cropped Sparse PLY ---")
    crop_dir = SPARSE_DIR / "cropped"
    if not crop_dir.exists():
        raise FileNotFoundError(
            f"sparse/cropped/ not found — run: python crop.py --workspace {WORKSPACE_DIR} --sparse-only"
        )
    run_colmap_command([
        COLMAP_EXECUTABLE, "model_converter",
        "--input_path",  str(crop_dir),
        "--output_path", str(SPARSE_DIR / "sparse_cropped.ply"),
        "--output_type", "PLY",
    ])


def image_undistorter():
    """
    Step 4: Undistort images using the sparse model.
    Required before dense reconstruction.
    """
    print("--- Starting Image Undistortion ---")
    DENSE_DIR.mkdir(parents=True, exist_ok=True)
    run_colmap_command([
        COLMAP_EXECUTABLE, "image_undistorter",
        "--image_path",   str(IMAGES_DIR),
        "--input_path",   str(sorted(SPARSE_DIR.iterdir())[0]),
        "--output_path",  str(DENSE_DIR),
        "--output_type",  "COLMAP",
    ])


def patch_match_stereo():
    """
    Step 5: Compute depth maps for each image using PatchMatch (GPU).
    """
    print("--- Starting PatchMatch Stereo ---")
    run_colmap_command([
        COLMAP_EXECUTABLE, "patch_match_stereo",
        "--workspace_path", str(DENSE_DIR),
        "--workspace_format", "COLMAP",
        "--PatchMatchStereo.geom_consistency", "true",
    ])


def stereo_fusion():
    """
    Step 6: Fuse depth maps into a single dense point cloud (fused.ply).
    """
    print("--- Starting Stereo Fusion ---")
    run_colmap_command([
        COLMAP_EXECUTABLE, "stereo_fusion",
        "--workspace_path",   str(DENSE_DIR),
        "--workspace_format", "COLMAP",
        "--input_type",       "geometric",
        "--output_path",      str(DENSE_DIR / "fused.ply"),
    ])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace",   default=str(BASE_DIR / "workspace"), help="Path to workspace folder (must contain an images/ subfolder)")
    parser.add_argument("--sparse-only",  action="store_true", help="Run only sparse reconstruction (steps 1-3)")
    parser.add_argument("--dense-only",   action="store_true", help="Run only dense reconstruction (steps 4-6), assumes sparse already exists")
    parser.add_argument("--sequential",        action="store_true", help="Use sequential matcher instead of exhaustive (recommended for video)")
    parser.add_argument("--export-sparse-ply",  action="store_true", help="Export sparse cloud to sparse/sparse.ply")
    parser.add_argument("--export-cropped-ply", action="store_true", help="Export bbox-cropped sparse cloud to sparse/sparse_cropped.ply")
    args = parser.parse_args()

    # Override path constants with the chosen workspace
    import builtins
    WORKSPACE_DIR  = Path(args.workspace)
    IMAGES_DIR     = WORKSPACE_DIR / "images"
    SPARSE_DIR     = WORKSPACE_DIR / "sparse"
    DENSE_DIR      = WORKSPACE_DIR / "dense"
    DATABASE_PATH  = WORKSPACE_DIR / "database.db"

    import sys
    current_module = sys.modules[__name__]
    current_module.WORKSPACE_DIR = WORKSPACE_DIR
    current_module.IMAGES_DIR    = IMAGES_DIR
    current_module.SPARSE_DIR    = SPARSE_DIR
    current_module.DENSE_DIR     = DENSE_DIR
    current_module.DATABASE_PATH = DATABASE_PATH

    if not IMAGES_DIR.exists():
        raise FileNotFoundError(f"images/ folder not found in workspace: {IMAGES_DIR}")

    export_only = args.export_sparse_ply and not args.sparse_only and not args.dense_only
    run_sparse  = not args.dense_only and not export_only
    run_dense   = not args.sparse_only and not export_only

    print(f"COLMAP Pipeline Started — workspace: {WORKSPACE_DIR}")

    if run_sparse:
        feature_extraction()
        feature_matching(sequential=args.sequential)
        mapper()

    if args.export_sparse_ply:
        export_sparse_ply()

    if args.export_cropped_ply:
        export_cropped_ply()

    if run_dense:
        image_undistorter()
        patch_match_stereo()
        stereo_fusion()

    print("Pipeline Finished")
