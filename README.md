# COLMAP Learning

A Python pipeline for 3D reconstruction using COLMAP. Supports regular video/photo input and iOS AR capture with gravity-aligned, metric-scale reconstruction.

```
Video/Photos → Feature Extraction → Matching → SfM (sparse) → PatchMatch (dense) → fused.ply
AR Session   → Inject AR Poses   → Matching → Triangulation → Crop to bbox → dense_cropped/fused.ply
```

---

## Getting Started

See **[SETUP.md](SETUP.md)** for full setup instructions:
- Installing prerequisites (Python, COLMAP, MeshLab)
- Creating the Python virtual environment
- Building and running the iOS capture app
- End-to-end AR capture walkthrough

See **[COMMANDS.md](COMMANDS.md)** for the full command reference:
- All `run.ps1` commands and flags
- Pipeline flags for `main.py`, `crop.py`, `align.py`
- Frame extraction, viewing, and scene reset commands
