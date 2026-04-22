"""
run.py — unified entry point for the COLMAP photogrammetry pipeline.

Usage:
    python run.py <command> [args]

Commands:
    ar       <session> <workspace>   Full AR pipeline: inject + crop + dense
    sparse   <workspace>             Sparse reconstruction (regular video/photos)
    dense    <workspace>             Dense reconstruction from existing sparse
    crop     <workspace>             Crop + dense reconstruction (AR scenes)
    view     <workspace> [type]      Open result in MeshLab (auto-detects best)
    validate <workspace>             Validate bounding box alignment (AR scenes)
"""

import argparse
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).parent.resolve()


def run(cmd: list):
    subprocess.run([str(c) for c in cmd], check=True)


# ─── Commands ─────────────────────────────────────────────────────────────────

def cmd_ar(args):
    session   = Path(args.session).resolve()
    workspace = Path(args.workspace).resolve()

    # ── Step 1: inject AR poses (always, unless already done) ──────────────────
    sparse_done = (workspace / "sparse" / "0" / "cameras.bin").exists()
    if sparse_done and not args.force:
        print("[skip] Sparse model already exists — use --force to re-inject")
    else:
        if args.force:
            import shutil
            for p in ["database.db", "sparse"]:
                target = workspace / p
                if target.is_dir():  shutil.rmtree(target)
                elif target.exists(): target.unlink()
        run([sys.executable, BASE / "inject_poses.py",
             "--session",   session,
             "--workspace", workspace])

    # ── Optional validation ────────────────────────────────────────────────────
    if args.validate:
        run([sys.executable, BASE / "validate.py",
             "--workspace", workspace])

    # ── Step 2: what to build after sparse ────────────────────────────────────
    if args.sparse:
        print("[done] Sparse only — stopping here.")
    elif args.dense:
        run([sys.executable, BASE / "main.py",
             "--workspace", workspace,
             "--dense-only"])
    elif args.crop_sparse:
        run([sys.executable, BASE / "crop.py",
             "--workspace", workspace,
             "--sparse-only"])
    else:
        # Default and --crop: crop to bounding box + dense_cropped
        run([sys.executable, BASE / "crop.py",
             "--workspace", workspace])


def cmd_sparse(args):
    workspace = Path(args.workspace).resolve()
    extra = ["--sequential"] if args.sequential else []
    run([sys.executable, BASE / "main.py",
         "--workspace", workspace,
         "--sparse-only"] + extra)


def cmd_dense(args):
    workspace = Path(args.workspace).resolve()
    run([sys.executable, BASE / "main.py",
         "--workspace", workspace,
         "--dense-only"])


def cmd_crop(args):
    workspace = Path(args.workspace).resolve()
    run([sys.executable, BASE / "crop.py",
         "--workspace", workspace])


def cmd_view(args):
    workspace  = Path(args.workspace).resolve()
    view_type  = args.type or _best_type(workspace)
    _ensure_ply(workspace, view_type)
    print(f"Opening {view_type} ...")
    subprocess.run([
        "powershell", "-ExecutionPolicy", "Bypass",
        "-File", str(BASE / "view_ply.ps1"),
        "-workspace", str(workspace),
        "-type", view_type,
    ])


def _ensure_ply(workspace: Path, view_type: str):
    """Auto-export sparse PLY files if the binary model exists but the PLY doesn't."""
    if view_type == "sparse":
        ply = workspace / "sparse" / "sparse.ply"
        model = workspace / "sparse" / "0"
        if not ply.exists() and model.exists():
            print("Exporting sparse PLY...")
            run([sys.executable, BASE / "main.py",
                 "--workspace", workspace,
                 "--export-sparse-ply"])

    elif view_type == "sparse_cropped":
        ply = workspace / "sparse" / "sparse_cropped.ply"
        model = workspace / "sparse" / "cropped"
        if not ply.exists() and model.exists():
            print("Exporting sparse_cropped PLY...")
            run([sys.executable, BASE / "main.py",
                 "--workspace", workspace,
                 "--export-cropped-ply"])


def cmd_validate(args):
    workspace = Path(args.workspace).resolve()
    run([sys.executable, BASE / "validate.py",
         "--workspace", workspace])


def _best_type(workspace: Path) -> str:
    candidates = [
        ("dense_cropped",  "dense_cropped/fused.ply"),
        ("dense",          "dense/fused.ply"),
        ("sparse_cropped", "sparse/sparse_cropped.ply"),
        ("sparse",         "sparse/sparse.ply"),
    ]
    # Check PLY files first
    for type_name, rel in candidates:
        if (workspace / rel).exists():
            return type_name
    # Fall back to models that exist but need export
    if (workspace / "sparse" / "cropped").exists():
        return "sparse_cropped"
    if (workspace / "sparse" / "0").exists():
        return "sparse"
    return "sparse"


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        prog="run.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # ar
    p = sub.add_parser("ar", help="AR pipeline: inject poses then sparse / dense / crop")
    p.add_argument("session",   help="Path to iOS session folder (inner folder containing frames/)")
    p.add_argument("workspace", help="Target workspace folder")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--sparse",       action="store_true", help="Inject only - stop after sparse")
    g.add_argument("--dense",        action="store_true", help="Inject + dense on full scene (no crop)")
    g.add_argument("--crop-sparse",  action="store_true", help="Inject + crop sparse only (no dense)")
    g.add_argument("--crop",         action="store_true", help="Inject + crop to bbox + dense_cropped (default)")
    p.add_argument("--validate", action="store_true", help="Run validate.py after injection")
    p.add_argument("--force",    action="store_true", help="Re-run injection even if sparse exists")

    # sparse
    p = sub.add_parser("sparse", help="Sparse reconstruction")
    p.add_argument("workspace")
    p.add_argument("--sequential", action="store_true", help="Sequential matcher (use for video)")

    # dense
    p = sub.add_parser("dense", help="Dense reconstruction from existing sparse")
    p.add_argument("workspace")

    # crop
    p = sub.add_parser("crop", help="Crop to bounding box + dense reconstruction")
    p.add_argument("workspace")

    # view
    p = sub.add_parser("view", help="Open result in MeshLab")
    p.add_argument("workspace")
    p.add_argument("type", nargs="?",
                   choices=["sparse", "dense", "sparse_cropped", "dense_cropped",
                            "sparse_aligned", "dense_aligned"],
                   help="Which result to view (auto-detected if omitted)")

    # validate
    p = sub.add_parser("validate", help="Validate bounding box alignment")
    p.add_argument("workspace")

    args = parser.parse_args()
    {
        "ar":       cmd_ar,
        "sparse":   cmd_sparse,
        "dense":    cmd_dense,
        "crop":     cmd_crop,
        "view":     cmd_view,
        "validate": cmd_validate,
    }[args.command](args)


if __name__ == "__main__":
    main()
