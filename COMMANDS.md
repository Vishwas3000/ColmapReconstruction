# COMMANDS

## Quick Reference — `run.ps1`

Single entry point for all pipeline operations. Activates the venv automatically.

```powershell
# AR capture — choose what to build after injecting poses
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan              # default: crop + dense_cropped
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --sparse       # inject only (sparse)
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --dense        # inject + full dense (no crop)
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --crop-sparse  # inject + crop sparse only
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --crop         # inject + crop + dense_cropped
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --validate  # add bbox validation step
.\run.ps1 ar  .\session_1234\session_1234  .\scenes\myscan  --force     # re-run from scratch

# Regular video/photo pipeline
.\run.ps1 sparse  .\scenes\panda                  # sparse only
.\run.ps1 sparse  .\scenes\panda  --sequential    # video input (use sequential matcher)
.\run.ps1 dense   .\scenes\panda                  # dense from existing sparse

# Bounding box crop + dense (AR scenes, after ar or inject)
.\run.ps1 crop  .\scenes\myscan

# View result in MeshLab (auto-detects best available PLY)
.\run.ps1 view  .\scenes\myscan
.\run.ps1 view  .\scenes\myscan  dense_cropped    # explicit type

# Validate bounding box alignment
.\run.ps1 validate  .\scenes\myscan
```

| `run.ps1` command | Steps |
|---|---|
| `ar <session> <workspace>` | inject_poses → crop → dense_cropped (default) |
| `ar ... --sparse` | inject_poses only |
| `ar ... --dense` | inject_poses → dense (full scene, no crop) |
| `ar ... --crop-sparse` | inject_poses → crop sparse only |
| `ar ... --crop` | inject_poses → crop → dense_cropped |
| `sparse <workspace>` | feature extraction → matching → SfM |
| `dense <workspace>` | undistort → depth maps → fusion |
| `crop <workspace>` | bbox crop → undistort → depth maps → fusion |
| `view <workspace> [type]` | open PLY in MeshLab |
| `validate <workspace>` | show bbox vs point cloud in Open3D |

View types: `sparse` `dense` `sparse_cropped` `dense_cropped` `sparse_aligned` `dense_aligned`

---

## Frame Extraction

Extract frames from a video into a workspace's `images/` folder.

```powershell
# Default: 2 fps, output to workspace/images
bash extract_frames.sh <video> [fps] [output_dir]

# Examples
bash extract_frames.sh myvideo.mp4               # 2 fps → workspace/images
bash extract_frames.sh myvideo.mp4 5             # 5 fps → workspace/images
bash extract_frames.sh myvideo.mp4 5 .\scenes\panda\images
```

Or directly with ffmpeg (no bash needed):
```powershell
ffmpeg -i myvideo.mp4 -vf fps=5 .\scenes\panda\images\frame_%04d.jpg
```

---

## COLMAP Pipeline (`main.py`)

### Full pipeline (sparse + dense)
```powershell
python main.py --workspace .\scenes\panda
```

### Sparse only (steps 1–3: feature extraction, matching, SfM)
```powershell
# Unordered photos (all images compared against each other)
python main.py --workspace .\scenes\panda --sparse-only

# Video sequence (only nearby frames matched — use this for videos)
python main.py --workspace .\scenes\panda --sparse-only --sequential
```

### Dense only (steps 4–6: undistort, depth maps, fusion)
Requires sparse to already exist.
```powershell
python main.py --workspace .\scenes\panda --dense-only
```

### Export sparse point cloud to PLY
Converts the sparse reconstruction to `sparse/sparse.ply` without running any other pipeline step.
```powershell
python main.py --workspace .\scenes\panda --export-sparse-ply
```

### Export bbox-cropped sparse point cloud to PLY
Requires `crop.py --sparse-only` to have been run first.
```powershell
python main.py --workspace .\scenes\ar_capture --export-cropped-ply
```

### All flags

| Flag | Description |
|---|---|
| `--workspace <path>` | Path to scene folder. Must contain an `images/` subfolder. Defaults to `workspace/` |
| `--sparse-only` | Run only sparse reconstruction (steps 1–3) |
| `--dense-only` | Run only dense reconstruction (steps 4–6) |
| `--sequential` | Use sequential matcher instead of exhaustive. Always use this for video input |
| `--export-sparse-ply` | Export sparse cloud to `sparse/sparse.ply` only |
| `--export-cropped-ply` | Export bbox-cropped sparse cloud to `sparse/sparse_cropped.ply` only |

---

## Viewing Results

### Open sparse cloud in MeshLab
```powershell
# First export sparse to PLY
python main.py --workspace .\scenes\panda --export-sparse-ply

# Then open it
.\view_ply.ps1 -workspace .\scenes\panda -type sparse
```

### Open dense cloud in MeshLab
```powershell
.\view_ply.ps1 -workspace .\scenes\panda -type dense
```

### Open bbox-cropped sparse cloud in MeshLab
```powershell
.\view_ply.ps1 -workspace .\scenes\ar_capture -type sparse_cropped
```

### Open bbox-cropped dense cloud in MeshLab
```powershell
.\view_ply.ps1 -workspace .\scenes\ar_capture -type dense_cropped
```

### Open sparse in COLMAP GUI
```powershell
$env:QT_QPA_PLATFORM_PLUGIN_PATH="C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"; & "C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe" gui
```
Then: **File → Import Model** → select `scenes\panda\sparse\0`

### All `-type` values for `view_ply.ps1`

| Type | File |
|---|---|
| `sparse` | `sparse\sparse.ply` |
| `dense` | `dense\fused.ply` |
| `sparse_aligned` | `sparse\sparse_aligned.ply` |
| `dense_aligned` | `dense\fused_aligned.ply` |
| `dense_cropped`  | `dense_cropped\fused.ply` |
| `sparse_cropped` | `sparse\sparse_cropped.ply` |

---

## Multi-scene Workflow

Each scene gets its own workspace folder with an `images/` subfolder inside.

```
scenes/
  panda/
    images/   ← frames extracted from panda.mp4
  bottle/
    images/   ← frames extracted from bottle.mp4
  room/
    images/   ← photos of a room
```

```powershell
# Set up a new scene from video
ffmpeg -i newscene.mp4 -vf fps=5 .\scenes\newscene\images\frame_%04d.jpg

# Run full pipeline
python main.py --workspace .\scenes\newscene --sequential

# View result
.\view_ply.ps1 -workspace .\scenes\newscene -type dense
```

---

## Alignment (`align.py`)

Aligns a point cloud to world axes so the bounding box is axis-aligned. Outputs `*_aligned.ply` and `transform.txt` next to the source PLY.

```powershell
# Align dense cloud using RANSAC (best for scenes with a floor or flat surface)
python align.py --workspace .\scenes\panda --source dense --method ransac

# Align sparse cloud using RANSAC
python align.py --workspace .\scenes\panda --source sparse --method ransac

# Align using PCA (best for compact objects with no clear flat surface)
python align.py --workspace .\scenes\panda --source dense --method pca
```

### Alignment flags

| Flag | Default | Description |
|---|---|---|
| `--workspace <path>` | required | Scene workspace folder |
| `--source` | `dense` | Which PLY to align: `dense` = `fused.ply`, `sparse` = `sparse.ply` |
| `--method` | `ransac` | `ransac`: uses dominant plane as Up. `pca`: uses smallest-variance axis as Up |
| `--voxel-size` | `0.02` | Downsampling voxel size before plane fitting. Increase for large scenes |
| `--ransac-threshold` | `0.02` | Max distance from plane to count as inlier |

### View aligned result

```powershell
.\view_ply.ps1 -workspace .\scenes\panda -type dense_aligned
.\view_ply.ps1 -workspace .\scenes\panda -type sparse_aligned
```

---

## iOS AR Capture Workflow

Use instead of plain video when you want gravity-aligned, metric-scale reconstruction with a user-defined bounding box.

**Step 1** — Build and run `app/objectCapture-iOS` on an iPhone:
1. Scan the surface → tap to place bounding box around your object
2. Resize with pinch (footprint) and two-finger drag (height)
3. Tap **Start Capturing** → walk slowly around the object
4. Tap **Stop & Export** → AirDrop or save the `.zip` to your PC
5. Unzip — the session folder is one level inside the zip (e.g. `session_1234\session_1234\`)

**Step 2** — Inject AR poses + run feature matching + triangulate:
```powershell
# Clear any previous run first
Remove-Item .\scenes\myscan\database.db, .\scenes\myscan\sparse -Recurse -ErrorAction SilentlyContinue

# Point --session at the inner folder (the one that contains frames/ and bounds.json)
python inject_poses.py --session .\session_1234\session_1234 --workspace .\scenes\myscan
```

`inject_poses.py` runs: feature extraction → sequential matching → point triangulation.  
Output is gravity-aligned and metric-scale — no need to run `align.py`.

**Step 3** — Validate bounding box alignment (optional but recommended):
```powershell
python validate.py --workspace .\scenes\myscan
```
Opens an Open3D viewer: grey = full cloud, green = points inside bbox, yellow = bbox wireframe.

**Step 4a** — Dense reconstruction of the full scene:
```powershell
python main.py --workspace .\scenes\myscan --dense-only
.\view_ply.ps1 -workspace .\scenes\myscan -type dense
```

**Step 4b** — Dense reconstruction cropped to the bounding box only:
```powershell
python crop.py --workspace .\scenes\myscan
.\view_ply.ps1 -workspace .\scenes\myscan -type dense_cropped
```

---

## Bounding Box Crop (`crop.py`)

Crops the sparse model to only the points inside `bounds.json`, then runs dense
reconstruction on those images only. Output goes to `dense_cropped/fused.ply`.

```powershell
# Crop sparse model only — no dense reconstruction
python crop.py --workspace .\scenes\ar_capture --sparse-only

# Crop sparse model + run full dense pipeline
python crop.py --workspace .\scenes\ar_capture

# Skip re-cropping — just re-run dense on an existing sparse/cropped/ model
# (always clear dense_cropped first to avoid COLMAP crash on partial output)
Remove-Item .\scenes\ar_capture\dense_cropped -Recurse -ErrorAction SilentlyContinue
python crop.py --workspace .\scenes\ar_capture --dense-only
```

### Crop flags

| Flag | Default | Description |
|---|---|---|
| `--workspace <path>` | required | Scene workspace folder (must have `bounds.json` and `sparse/0/`) |
| `--sparse-only` | off | Crop sparse model only, skip dense reconstruction |
| `--dense-only` | off | Skip cropping, re-run dense on existing `sparse/cropped/` |
| `--min-obs` | `2` | Min inside-box point observations required to keep an image |

Output structure:
```
scenes/myscan/
  sparse/cropped/     ← cropped cameras.bin + images.bin + points3D.bin
  dense_cropped/
    fused.ply         ← final dense cloud, bbox-cropped
```

---

## Reset a Scene

Delete all generated data and start fresh:
```powershell
Remove-Item .\scenes\panda\database.db, .\scenes\panda\sparse, .\scenes\panda\dense -Recurse -ErrorAction SilentlyContinue
```

For AR capture scenes (also removes cropped outputs):
```powershell
Remove-Item .\scenes\ar_capture\database.db, .\scenes\ar_capture\sparse, .\scenes\ar_capture\dense, .\scenes\ar_capture\dense_cropped -Recurse -ErrorAction SilentlyContinue
```
