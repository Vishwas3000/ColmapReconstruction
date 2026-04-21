# COMMANDS

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
Converts the sparse reconstruction to `sparse/sparse.ply` for viewing in MeshLab.
```powershell
python main.py --workspace .\scenes\panda --export-sparse-ply
```

### All flags

| Flag | Description |
|---|---|
| `--workspace <path>` | Path to scene folder. Must contain an `images/` subfolder. Defaults to `workspace/` |
| `--sparse-only` | Run only sparse reconstruction (steps 1–3) |
| `--dense-only` | Run only dense reconstruction (steps 4–6) |
| `--sequential` | Use sequential matcher instead of exhaustive. Always use this for video input |
| `--export-sparse-ply` | Export sparse cloud to `sparse/sparse.ply` |

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

### Open sparse in COLMAP GUI
```powershell
$env:QT_QPA_PLATFORM_PLUGIN_PATH="C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\plugins\platforms"; & "C:\Users\timeu\OneDrive\Documents\GitHub\colmap-bin\bin\colmap.exe" gui
```
Then: **File → Import Model** → select `scenes\panda\sparse\0`

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

## Reset a Scene

Delete all generated data and start fresh:
```powershell
Remove-Item .\scenes\panda\database.db, .\scenes\panda\sparse, .\scenes\panda\dense -Recurse -ErrorAction SilentlyContinue
```
