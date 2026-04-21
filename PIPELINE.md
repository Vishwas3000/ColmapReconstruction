# Pipeline Deep Dive

A bottom-up explanation of every stage in this photogrammetry pipeline —
from raw pixels to a dense 3D point cloud. Each section covers what happens,
why it works, and pointers to the math underneath.

---

## Table of Contents

1. [Big Picture](#1-big-picture)
2. [Coordinate Systems](#2-coordinate-systems)
3. [Path A — Video Input](#3-path-a--video-input)
4. [Path B — iOS AR Capture](#4-path-b--ios-ar-capture)
5. [Stage 1 — Feature Extraction (SIFT)](#5-stage-1--feature-extraction-sift)
6. [Stage 2 — Feature Matching](#6-stage-2--feature-matching)
7. [Stage 3 — Structure from Motion (Mapper)](#7-stage-3--structure-from-motion-mapper)
8. [Stage 3b — Point Triangulator (AR path)](#8-stage-3b--point-triangulator-ar-path)
9. [Stage 4 — Image Undistortion](#9-stage-4--image-undistortion)
10. [Stage 5 — PatchMatch Stereo](#10-stage-5--patchmatch-stereo)
11. [Stage 6 — Stereo Fusion](#11-stage-6--stereo-fusion)
12. [Stage 7 — Bounding Box Crop](#12-stage-7--bounding-box-crop)
13. [Math Reference](#13-math-reference)

---

## 1. Big Picture

Two entry points, one output format.

```
Video ──────────────────────────────────────────────┐
  ffmpeg → images                                   │
  feature_extractor → database.db (keypoints)       │
  exhaustive/sequential_matcher → database.db        │    sparse/0/
  mapper (SfM: solve poses + points) ───────────────┤──► cameras.bin
                                                    │    images.bin
iPhone ARKit ───────────────────────────────────────┤    points3D.bin
  inject_poses.py                                   │
    known poses from ARKit (metric, gravity-aligned) │
    feature_extractor + sequential_matcher           │
    point_triangulator (solve points only) ──────────┘
                                                         │
                                              ┌──────────▼──────────┐
                                              │  Dense Pipeline      │
                                              │  image_undistorter   │
                                              │  patch_match_stereo  │
                                              │  stereo_fusion       │
                                              └──────────┬──────────┘
                                                         │
                                                    fused.ply
                                                         │
                                              (AR path only)
                                              crop.py → dense_cropped/fused.ply
```

The sparse model (cameras + images + points3D) is the shared currency.
Everything downstream reads it. Everything upstream produces it.

---

## 2. Coordinate Systems

Understanding this prevents hours of debugging.

### Camera conventions

| System | X | Y | Z | Pose format |
|---|---|---|---|---|
| ARKit | right | up | backward (away from scene) | Camera-to-World (C2W) |
| COLMAP | right | down | forward (into scene) | World-to-Camera (W2C) |
| OpenCV | right | down | forward (into scene) | W2C |

ARKit and COLMAP agree on X but flip Y and Z.

### Converting ARKit C2W → COLMAP W2C

```python
flip = np.diag([1.0, -1.0, -1.0, 1.0])
c2w_colmap = c2w @ flip      # flip camera Y and Z axes
w2c = np.linalg.inv(c2w_colmap)
```

`c2w @ flip` changes the *camera axes* from ARKit convention to COLMAP
convention. The **world frame is untouched** — column 3 (camera position)
is unchanged by the flip.

Result: the COLMAP world frame = ARKit world frame (Y-up, gravity-aligned,
metric scale in metres).

### Why this matters for bounds.json

`bounds.json` stores the bounding box center in ARKit world space.
`points3D.bin` stores 3D points also in ARKit world space (because that is
what the injected W2C matrices define as the world).

**No coordinate flip is needed when comparing them.** The earlier bug in
`validate.py` applied `center * [1,-1,-1]` which moved the box into a
space that does not exist anywhere in the pipeline.

### Camera projection

A 3D world point `P` projects to a 2D image point `p` as:

```
p = K * [R | t] * P

where:
  K = intrinsic matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
  R = rotation (world → camera)
  t = translation (camera position in world, negated and rotated)
  [R | t] = W2C matrix (top 3 rows of the 4×4)
```

### Intrinsics (PINHOLE model)

```
fx, fy  — focal lengths in pixels
cx, cy  — principal point (image centre, approximately)

ARKit gives these in sensor (landscape) orientation.
COLMAP reads raw pixel dimensions from JPEG (ignores EXIF rotation).
→ Use intrinsics as-is, no swapping.
```

---

## 3. Path A — Video Input

```
video.mp4
  │  ffmpeg -vf fps=N
  ▼
images/frame_%04d.jpg        (N fps × duration frames)
  │  COLMAP feature_extractor
  ▼
database.db                  (keypoints + descriptors per image)
  │  COLMAP exhaustive_matcher  (photos)
  │  COLMAP sequential_matcher  (video — only match nearby frames)
  ▼
database.db                  (+ feature matches between pairs)
  │  COLMAP mapper
  ▼
sparse/0/                    (cameras + images + points3D)
```

**When to use sequential_matcher:** Video frames are ordered — frame 5 and
frame 6 overlap heavily, frame 5 and frame 200 do not. Exhaustive matching
wastes O(N²) time on pairs that will never match. Sequential only matches
a sliding window of nearby frames, making it O(N).

More importantly: exhaustive matching on a video sequence often destabilises
bundle adjustment because near-duplicate frames introduce degenerate geometry.

---

## 4. Path B — iOS AR Capture

```
iPhone (ARKit session)
  │  CaptureManager.swift — 2fps auto or manual shutter
  │  Per frame: JPEG + metadata JSON (pose, intrinsics, bbox)
  │  Stop & Export → .zip
  ▼
session_XXXX/
  frames/frame_0000.jpg ...
  metadata/metadata_0000.json ...
  bounds.json
  │  inject_poses.py
  │    1. Copy frames → workspace/images/
  │    2. Convert ARKit C2W → COLMAP W2C  (arkit_to_colmap)
  │    3. Write sparse/known/cameras.txt + images.txt
  │    4. COLMAP feature_extractor  (PINHOLE, known intrinsics)
  │    5. COLMAP sequential_matcher
  │    6. COLMAP model_converter    (TXT → BIN)
  │    7. COLMAP point_triangulator (known poses → solve only points)
  ▼
sparse/0/
```

**Why AR capture is better than video for objects:**
- Metric scale — ARKit poses are in real-world metres. No scale ambiguity.
- Gravity alignment — Y-axis is always world up. No need to run `align.py`.
- Known intrinsics — ARKit reports per-frame `fx, fy, cx, cy` from the
  actual lens calibration. COLMAP does not need to estimate them.
- Bounding box — the user defines the object volume in world space at
  capture time. Stored in `bounds.json` for cropping later.

**What ARKit gives vs what COLMAP still needs to do:**
- ARKit gives: camera poses ✓, intrinsics ✓, scale ✓, gravity ✓
- COLMAP still does: feature extraction, feature matching, triangulation

---

## 5. Stage 1 — Feature Extraction (SIFT)

**Goal:** Find a set of points in each image that are:
- Repeatable — found in the same physical location across different images
- Distinctive — described in a way that allows matching

### Scale space

SIFT (Scale-Invariant Feature Transform) builds a Gaussian pyramid —
the image convolved with Gaussians at increasing σ:

```
L(x, y, σ) = G(x, y, σ) * I(x, y)
```

Differences of Gaussians (DoG) approximate the Laplacian of Gaussian,
which is a blob detector:

```
D(x, y, σ) = L(x, y, kσ) - L(x, y, σ)
```

Local extrema of D across space AND scale are keypoint candidates.

### Orientation assignment

For each keypoint, a dominant gradient orientation is computed from a
local histogram of image gradients. This makes the descriptor
rotation-invariant.

### Descriptor

A 4×4 grid of 8-bin gradient histograms around the keypoint → 128-dim
vector. Normalised to be invariant to linear illumination changes.

### Output

`database.db` — for each image: list of (x, y, scale, orientation) keypoints
and their 128-dim descriptors.

---

## 6. Stage 2 — Feature Matching

**Goal:** For each pair of images, find which keypoints correspond to the
same 3D point.

### Nearest-neighbour search

For each descriptor `d` in image A, find the closest descriptor in image B
using Euclidean distance in 128-dim space.

### Lowe's ratio test

A match is accepted only if:
```
dist(d, nearest) / dist(d, second_nearest) < 0.8
```

If the nearest and second-nearest are similar distances apart, the match is
ambiguous — reject it. This removes ~90% of false matches.

### RANSAC geometric verification

Even after the ratio test, some wrong matches remain. RANSAC fits a
fundamental matrix F (encodes the epipolar geometry between two views)
to the matches:

```
p2^T * F * p1 = 0   (epipolar constraint)
```

Matches that satisfy this are inliers; the rest are outliers and discarded.

### Sequential vs exhaustive

- `exhaustive_matcher` — all N×N pairs. O(N²). Use for unordered photos.
- `sequential_matcher` — sliding window of size W around each frame.
  O(N×W). Use for video.

### Output

`database.db` — verified inlier matches for each image pair.

---

## 7. Stage 3 — Structure from Motion (Mapper)

**Goal:** Given 2D keypoint matches, recover:
- 6-DOF camera pose for every image (R, t)
- 3D position for every matched keypoint

This is the hardest stage. It runs iteratively.

### Incremental SfM

COLMAP uses incremental SfM:

1. **Initialise** — pick the two images with the most matches and good
   baseline (not too close, not too far). Triangulate an initial set of
   points.

2. **Register** — find the next image that sees the most already-triangulated
   points. Solve its pose using PnP (Perspective-n-Point):
   given N 3D↔2D correspondences, find R and t.

3. **Triangulate** — for newly registered image, triangulate new 3D points
   from its matches with already-registered images.

4. **Bundle adjust** — refine ALL poses and ALL point positions jointly to
   minimise total reprojection error.

5. **Repeat** until all images are registered.

### Triangulation

Given a 3D point visible in two cameras, its position is the intersection
of two rays:

```
ray_1: C1 + λ1 * d1
ray_2: C2 + λ2 * d2
```

Rays rarely intersect exactly (noise), so solve in least-squares:
minimise the distance between the two rays. This is a linear system
solved via SVD.

### Bundle adjustment

The core optimisation. Minimise:

```
E = Σ_ij  ||p_ij  -  π(K_i, R_i, t_i, X_j)||²

where:
  p_ij    = observed 2D keypoint of point j in image i
  π(...)  = projection function (world point → image pixel)
  X_j     = 3D position of point j
  R_i, t_i = pose of camera i
  K_i     = intrinsics of camera i
```

This is a non-linear least-squares problem solved with
**Levenberg-Marquardt (LM)**. LM interpolates between gradient descent
(far from solution) and Gauss-Newton (near solution).

The Jacobian of the projection function with respect to poses and points
has a sparse block structure — exploited by sparse linear solvers
(Schur complement trick) for efficiency.

### Gauge freedom

SfM has an inherent 7-DOF ambiguity: any similarity transform
(rotation, translation, scale) applied to the entire reconstruction is
equally valid. COLMAP fixes this by anchoring the first camera.
AR capture fixes it by using real-world poses from ARKit.

### Output

```
sparse/0/
  cameras.bin   — intrinsics per camera
  images.bin    — pose (qvec + tvec) per image + 2D keypoint list
  points3D.bin  — 3D position + RGB + reprojection error + track
```

---

## 8. Stage 3b — Point Triangulator (AR path)

Same math as Stage 3 triangulation, but **poses are fixed** (taken from
ARKit). Only point positions are solved. No bundle adjustment over poses.

This is much faster and more stable than full SfM because:
- No incremental registration needed
- No pose initialisation problem
- Scale and orientation already correct from ARKit

The trade-off: if an ARKit pose is slightly wrong, there is no global
optimisation to correct it.

---

## 9. Stage 4 — Image Undistortion

Real lenses distort images — straight lines appear curved near the edges
(radial distortion). COLMAP's dense pipeline requires undistorted images
so that the epipolar geometry is exact.

```
Distorted pixel (xd, yd)
  → apply inverse distortion model
  → undistorted pixel (xu, yu)
```

For PINHOLE cameras (no distortion params), images are simply copied.
For SIMPLE_RADIAL cameras: `xu = xd * (1 + k1*r²)` where r is distance
from principal point.

Output: `dense/images/` — undistorted frames, one per registered camera.

---

## 10. Stage 5 — PatchMatch Stereo

**Goal:** For every pixel in every image, estimate its depth (distance
from camera).

### Setup

For each reference image, COLMAP selects ~20 source images (those with
good overlap and baseline). This is a multi-view stereo problem.

### Depth hypothesis

Each pixel `p` in the reference image has:
- A depth hypothesis `d`
- A normal hypothesis `n` (surface orientation)

These define a plane. The plane, combined with camera intrinsics and the
relative pose to a source image, gives a homography H that warps the
reference patch into the source image.

### Photometric consistency

A patch around `p` in the reference image should look similar to its
warped version in the source image (assuming Lambertian surfaces). The
similarity is measured with NCC (Normalised Cross-Correlation):

```
NCC(p_ref, p_src) ∈ [-1, 1]
  1  = perfect match
 -1  = inverse match
  0  = no correlation
```

### PatchMatch algorithm

Avoids testing all possible depths by:
1. **Random initialisation** — assign random depth + normal to each pixel
2. **Propagation** — if a neighbour has a good hypothesis, try it here too
   (good depths tend to cluster on the same surface)
3. **Random perturbation** — randomly jitter depth and normal, keep if better
4. **Iterate** — alternating top-to-bottom and bottom-to-top sweeps

Converges in a few iterations because propagation spreads good solutions fast.

### Two passes

1. **Photometric** — optimise for patch similarity only
2. **Geometric consistency** — re-project the estimated depth from image A
   into image B, check if image B's own depth estimate agrees. Keeps only
   mutually consistent depths. Removes most outliers.

### Output

```
dense/stereo/
  depth_maps/frame_XXXX.jpg.geometric.bin   — float32 depth per pixel
  normal_maps/frame_XXXX.jpg.geometric.bin  — float32 normal per pixel
```

---

## 11. Stage 6 — Stereo Fusion

**Goal:** Merge all depth maps into a single consistent point cloud.

### Problem

Each pixel in each image independently estimated a depth. The same
physical surface point appears in many images and generates many slightly
different 3D estimates. Naive concatenation gives a bloated, noisy cloud
with duplicated surfaces.

### Fusion

For each pixel `p` in image `i` with depth `d`:
1. Unproject to 3D: `X = R_i^T * (K^-1 * p * d - t_i)`
2. Project `X` into all other images
3. Check if their depth maps agree at that pixel (within a threshold)
4. Count votes — how many images confirm this point?
5. Keep point only if it has enough votes (geometric consensus)
6. Average the 3D positions of all agreeing estimates for the final point

Colour is taken from the reference image.

### Output

`dense/fused.ply` — a single coloured point cloud, typically millions of
points at sub-millimetre density for close-range captures.

---

## 12. Stage 7 — Bounding Box Crop

Two levels of cropping happen:

### Sparse crop (`crop.py` — `--sparse-only`)

Reads `points3D.bin`, tests each point against the oriented bounding box:

```python
local = (xyz - center) @ R.T    # world → box-local frame
inside = all(|local| <= extents/2)
```

`R` rotates from box-local to world (columns = box axes in world space),
so `R.T` is the inverse — world to box. Then a simple AABB test in
box-local space.

Images with fewer than `--min-obs` surviving points are dropped. A new
binary model is written to `sparse/cropped/`.

### Dense crop (post-fusion filter in `crop.py`)

`patch_match_stereo` computes depth for every pixel — it cannot be
told to ignore regions outside the bbox. So `fused.ply` always contains
the full scene. After fusion, the same box test is applied to every
point in `fused.ply` using Open3D:

```python
mask = inside_box(np.asarray(pcd.points), bounds)
clipped = pcd.select_by_index(np.where(mask)[0])
```

The clipped cloud overwrites `fused.ply`.

---

## 13. Math Reference

Topics to study in order, with what stage they unlock:

### Linear algebra foundations
- Homogeneous coordinates — why we use 4-vectors and 4×4 matrices
- SE(3) — the group of rigid transforms (rotation + translation)
- SO(3) — the group of rotations
- Matrix inverse, transpose, SVD

→ Unlocks: understanding every pose matrix in the pipeline

### Projective geometry
- Pinhole camera model
- Projection matrix P = K[R|t]
- Epipolar geometry, fundamental matrix F, essential matrix E
- Homography H

→ Unlocks: feature matching (RANSAC with F), PatchMatch (homography warp)

### Optimisation
- Non-linear least squares
- Jacobian, gradient, Hessian
- Gauss-Newton method
- Levenberg-Marquardt
- Sparse matrix structure in bundle adjustment (Schur complement)

→ Unlocks: bundle adjustment, why SfM sometimes diverges

### Quaternions
- Unit quaternion as rotation representation
- qw, qx, qy, qz → rotation matrix conversion
- Why quaternions instead of Euler angles (no gimbal lock, interpolation)

→ Unlocks: reading/writing COLMAP cameras.bin and images.bin

### Probability & statistics
- RANSAC — random sampling, inlier probability, iteration count formula
- Robust estimators (Huber loss, Cauchy loss) used in bundle adjustment

→ Unlocks: understanding why feature matching is robust to outliers

### Signal processing
- Gaussian blur, scale space
- Laplacian of Gaussian as blob detector
- Gradient histograms

→ Unlocks: SIFT feature extraction

### Stereo vision
- Epipolar constraint
- Disparity and depth relationship
- NCC, SSD patch similarity metrics
- PatchMatch convergence proof

→ Unlocks: PatchMatch stereo, depth map quality

---

*This document grows as the pipeline grows. Each section will be expanded
with worked examples and equations as we dive deeper.*
