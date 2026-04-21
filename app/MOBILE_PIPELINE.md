# Mobile AR Capture Pipeline

Using ARKit (iOS) or ARCore (Android) to replace COLMAP's arbitrary coordinate system with a metric, gravity-aligned world frame. This eliminates the need for post-hoc alignment scripts and gives you camera poses, metric scale, and a user-defined bounding volume for free.

---

## Why This Is Better Than Plain Photos

| | Plain Photos + COLMAP | AR Capture App |
|---|---|---|
| Gravity alignment | Manual (RANSAC/PCA after the fact) | Built-in — AR session is gravity-aligned |
| Camera poses | Solved by COLMAP (can fail) | Recorded per-frame from IMU — always accurate |
| Scale | Arbitrary (unit-less) | Metric (meters) |
| Bounding box | Derived from point cloud statistics | User-placed AR anchors during capture |
| Background masking | Manual or none | Screen-space brush tool during capture |

---

## Data the App Exports Per Frame

For every captured frame, the app writes two files:

**`frame_0001.jpg`** — the captured image

**`metadata_0001.json`**
```json
{
  "frame_id": 1,
  "timestamp": 1745123456.789,
  "intrinsics": {
    "fx": 1462.3,
    "fy": 1462.3,
    "cx": 960.0,
    "cy": 540.0
  },
  "camera_pose": [
    [r00, r01, r02, tx],
    [r10, r11, r12, ty],
    [r20, r21, r22, tz],
    [0,   0,   0,   1 ]
  ],
  "gravity": [0.0, -1.0, 0.0]
}
```

**`bounds.json`** — written once when the user confirms their bounding box
```json
{
  "center": [x, y, z],
  "extents": [width, height, depth],
  "rotation": [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
}
```

---

## Coordinate Convention Difference

ARKit and COLMAP use different conventions — a conversion is required:

| | ARKit | COLMAP |
|---|---|---|
| Y axis | Up | Down |
| Z axis | Toward viewer (-Z forward) | Away from viewer (+Z forward) |
| Pose | Camera-to-World (C2W) | World-to-Camera (W2C) |

```python
def arkit_to_colmap(c2w_arkit: np.ndarray) -> np.ndarray:
    flip = np.diag([1, -1, -1, 1]).astype(np.float64)
    c2w_colmap = c2w_arkit @ flip
    return np.linalg.inv(c2w_colmap)   # → World-to-Camera
```

---

## iOS Implementation Plan (ARKit)

### Phase 1 — AR Session + Frame Capture
- Set `worldAlignment = .gravity` on `ARWorldTrackingConfiguration`
- On capture button tap: grab `ARFrame`, extract `capturedImage`, `camera.transform` (4×4 C2W), `camera.intrinsics`, gravity from `camera.eulerAngles` or `physicalEnvironment`
- Write `frame_XXXX.jpg` + `metadata_XXXX.json` to the app's Documents directory

### Phase 2 — Bounding Box UI
- On tap: cast a ray from screen point using `ARSession.raycast`
- Place `ARAnchor` at hit point
- After 2 anchor placements: render a `SCNBox` (SceneKit) between them as a transparent overlay
- On confirm: write `bounds.json` with the box's center, extents, and rotation in world space

### Phase 3 — Background Masking
- Add a brush overlay (`UIBezierPath` or Metal shader) in screen space
- User paints the object — pixels outside the brush region get an alpha mask
- Export masked frames alongside normal frames

### Phase 4 — Export & Transfer
- Zip the session folder (`frames/`, `metadata/`, `bounds.json`)
- Share via AirDrop, Files app, or HTTP server (`GCDWebServer`) for direct transfer to Mac/PC

---

## Python Integration (`inject_poses.py`)

Instead of running `feature_matching`, inject the app's poses directly into COLMAP and triangulate only:

```
App export (frames + metadata)
    ↓  inject_poses.py
COLMAP images.txt + cameras.txt  (pre-filled with AR poses)
    ↓  colmap point_triangulator
sparse/   (points only, no pose solving needed)
    ↓  main.py --dense-only
dense/fused.ply  (metric-scale, gravity-aligned)
```

COLMAP commands used:
```bash
# Skip feature_extraction + exhaustive_matcher + mapper
# Instead:
colmap feature_extractor ...          # still needed for descriptors
colmap point_triangulator \
    --database_path database.db \
    --image_path images/ \
    --input_path sparse/known_poses/ \
    --output_path sparse/0/
```

---

## Android (Future)

Same pipeline using **ARCore** instead of ARKit. API differences:

| Feature | ARKit (iOS) | ARCore (Android) |
|---|---|---|
| World alignment | `ARWorldTrackingConfiguration.worldAlignment = .gravity` | `Config.setWorldAlignment(WorldAlignment.GRAVITY)` |
| Camera pose | `ARFrame.camera.transform` | `Frame.getCamera().getPose()` |
| Intrinsics | `ARCamera.intrinsics` | `CameraIntrinsics` from `Frame.getCamera()` |
| Plane detection | `ARPlaneAnchor` | `Plane` trackable |
| Ray casting | `ARSession.raycast` | `Frame.hitTest` |

Coordinate convention: ARCore uses **OpenGL** convention (Y-up, -Z forward) — same flip matrix as ARKit applies.

Implementation can reuse all Python-side code (`inject_poses.py`, `align.py`) unchanged. Only the app-side capture differs.

---

## File Structure After Capture

```
session_001/
├── frames/
│   ├── frame_0001.jpg
│   ├── frame_0002.jpg
│   └── ...
├── metadata/
│   ├── metadata_0001.json
│   ├── metadata_0002.json
│   └── ...
├── masks/           ← optional, from Phase 3
│   ├── mask_0001.png
│   └── ...
└── bounds.json
```

Pass the session folder to the pipeline:
```powershell
python inject_poses.py --session .\session_001 --workspace .\scenes\myscan
python main.py --workspace .\scenes\myscan --dense-only
```
