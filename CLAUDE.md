# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A learning scaffold for building a COLMAP photogrammetry pipeline in Python. The goal is to implement a 3-step Structure from Motion (SfM) pipeline — feature extraction, feature matching, and sparse reconstruction — by driving the COLMAP CLI via Python's `subprocess` module.

## Running the Pipeline

```bash
python main.py
```

COLMAP must be installed and accessible on `PATH` as `colmap`. If it is installed elsewhere, update the `COLMAP_EXECUTABLE` variable at the top of `main.py`.

## Project Structure

```
workspace/
  images/   ← place input images here before running
  sparse/   ← COLMAP writes sparse reconstruction output here
  dense/    ← reserved for future dense reconstruction output
  database.db  ← COLMAP creates this during feature extraction
main.py     ← pipeline implementation (all TODOs live here)
```

## What Needs to Be Implemented

Three functions in `main.py` have `TODO` stubs:

1. **`run_colmap_command(args)`** — core helper; call `subprocess.run(args, check=True)` and propagate errors.
2. **`feature_extraction()`** — calls `colmap feature_extractor --database_path <DATABASE_PATH> --image_path <IMAGES_DIR>`.
3. **`feature_matching()`** — calls `colmap exhaustive_matcher --database_path <DATABASE_PATH>`.
4. **`mapper()`** — calls `colmap mapper --database_path <DATABASE_PATH> --image_path <IMAGES_DIR> --output_path <SPARSE_DIR>`.

All path constants (`DATABASE_PATH`, `IMAGES_DIR`, `SPARSE_DIR`) are already defined at the top of `main.py`.

## Dependencies

No pip packages are required for the core implementation. Optional packages listed in `requirements.txt` (`numpy`, `tqdm`) are useful for post-processing or inspecting COLMAP's output database/binary files.
