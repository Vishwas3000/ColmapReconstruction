# Setup & AR Capture Workflow

## Prerequisites

### 1. Python
Python 3.10 or later. Download from [python.org](https://python.org).

### 2. COLMAP (with CUDA)
Download `colmap-x64-windows-cuda.zip` from the [COLMAP releases page](https://github.com/colmap/colmap/releases/latest) and extract to:
```
C:\Users\<you>\OneDrive\Documents\GitHub\colmap-bin\
```
The folder should contain `bin\colmap.exe` and `plugins\platforms\`.

> If you don't have an NVIDIA GPU, use the `nocuda` build instead — dense reconstruction will be CPU-only and much slower.

### 3. MeshLab
Download from [meshlab.net](https://www.meshlab.net/#download) and extract to:
```
C:\Users\<you>\OneDrive\Documents\GitHub\meshlab\
```

### 4. ffmpeg (optional — for video input only)
Install via `winget install ffmpeg` or download from [ffmpeg.org](https://ffmpeg.org/download.html).

---

## Python Environment

```powershell
# Create and activate virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt
```

Dependencies: `numpy`, `open3d`, `tqdm`

---

## Project Structure

```
ColmapLearning/
├── run.ps1               ← single entry point (activates venv, calls run.py)
├── run.py                ← unified CLI dispatcher
├── main.py               ← COLMAP pipeline (sparse + dense)
├── inject_poses.py       ← AR session → COLMAP workspace
├── crop.py               ← bbox crop + dense reconstruction
├── validate.py           ← visualise bbox alignment
├── align.py              ← align point cloud to world axes
├── view_ply.ps1          ← open .ply in MeshLab
├── requirements.txt
├── COMMANDS.md           ← full command reference
├── app/
│   └── objectCapture-iOS/   ← iPhone capture app (Xcode project)
└── scenes/
    └── <scene>/
        ├── images/        ← input frames
        ├── bounds.json    ← bounding box (AR sessions only)
        ├── database.db    ← COLMAP feature database
        ├── sparse/        ← sparse reconstruction
        └── dense/
            └── fused.ply  ← dense point cloud
```

---

## iOS App Setup (AR Capture)

1. Open `app/objectCapture-iOS/objectCapture.xcodeproj` in Xcode.
2. Set your **Team** under Signing & Capabilities.
3. Connect your iPhone and select it as the build target.
4. Build & Run (`Cmd+R`).

> Requires iOS 16+ and an iPhone with LiDAR (iPhone 12 Pro or later) for best results. Works without LiDAR but AR tracking will be less stable.

---

## AR Capture Workflow

### Step 1 — Capture on iPhone

1. Launch the app.
2. Point the camera at a flat surface — wait for the plane to be detected (white grid appears).
3. **Tap** the surface to place the bounding box.
4. Resize the box around your object:
   - **Tap a coloured handle** to select a face (red=X, green=Y, blue=Z).
   - **Drag** to push/pull that face.
   - **Two-finger rotate** to spin the box around Y.
5. Tap **Start Capturing** and walk slowly around the object — keep the object fully in frame.
6. Tap **Stop & Export** when you have covered all angles.
7. A `.zip` is saved to the Files app. AirDrop or cable-transfer it to your PC and unzip it.

The unzipped folder has this structure:
```
session_1234567890/
└── session_1234567890/       ← inner folder (use this path)
    ├── frames/               ← captured JPEG frames
    ├── metadata/             ← per-frame JSON (pose + intrinsics)
    └── bounds.json           ← bounding box (center, extents, rotation)
```

### Step 2 — Run the pipeline on PC

```powershell
# Full AR pipeline: inject poses → crop to bbox → dense reconstruction
.\run.ps1 ar .\session_1234567890\session_1234567890 .\scenes\myscan
```

This runs three stages automatically:

| Stage | What happens |
|---|---|
| **inject_poses** | Copies frames, parses AR poses, runs SIFT + sequential matching, triangulates sparse model |
| **crop** | Filters sparse model to bbox, converts to COLMAP binary, undistorts images |
| **dense** | PatchMatch stereo (GPU) + stereo fusion → `dense_cropped/fused.ply` |

Total time: ~5 min (inject + crop) + ~8s per frame (dense GPU). 145 frames ≈ 20 min.

### Step 3 — View the result

```powershell
# Auto-detects best available result
.\run.ps1 view .\scenes\myscan

# Explicit type
.\run.ps1 view .\scenes\myscan dense_cropped
```

### Optional: validate bbox alignment before running dense

```powershell
.\run.ps1 ar .\session_1234567890\session_1234567890 .\scenes\myscan --validate --sparse
```

Opens an Open3D viewer: grey = full sparse cloud, green = points inside bbox, yellow = bbox wireframe. If the bbox looks misaligned, re-capture.

---

## AR Pipeline Flags

```powershell
.\run.ps1 ar <session> <workspace> [flags]
```

| Flag | What runs after inject |
|---|---|
| *(none)* | crop + dense_cropped (default) |
| `--sparse` | stop after sparse (no dense, no crop) |
| `--dense` | full dense on entire scene (no bbox crop) |
| `--crop-sparse` | crop sparse model only (no dense) |
| `--crop` | crop + dense_cropped (explicit) |
| `--validate` | open validation viewer after injection |
| `--force` | re-run injection even if sparse already exists |

---

## Regular Video / Photo Pipeline

```powershell
# Extract frames from video
ffmpeg -i myvideo.mp4 -vf fps=5 .\scenes\myscene\images\frame_%04d.jpg

# Sparse reconstruction (sequential for video)
.\run.ps1 sparse .\scenes\myscene --sequential

# Dense reconstruction
.\run.ps1 dense .\scenes\myscene

# View
.\run.ps1 view .\scenes\myscene
```

---

## Resetting a Scene

```powershell
# Regular scene
Remove-Item .\scenes\myscene\database.db, .\scenes\myscene\sparse, .\scenes\myscene\dense -Recurse -ErrorAction SilentlyContinue

# AR scene (also removes cropped outputs)
Remove-Item .\scenes\myscene\database.db, .\scenes\myscene\sparse, .\scenes\myscene\dense, .\scenes\myscene\dense_cropped -Recurse -ErrorAction SilentlyContinue
```

---

## Tips

- **More frames = better quality** — 100–150 frames is ideal. Move slowly and overlap your coverage.
- **Dense is slow** — ~8s per frame on an RTX 5070 Ti. 145 frames ≈ 20 min.
- **Bbox crop vs full dense** — `dense_cropped` uses all images but clips the final cloud to the bbox. Much denser than the old approach of restricting cameras.
- **Re-run dense only** — if you already have sparse and just want to rebuild dense:
  ```powershell
  .\run.ps1 dense .\scenes\myscene
  ```
- **Sequential matching** — always use `--sequential` for video input. Exhaustive matching is for unordered photos only.
