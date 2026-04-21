#!/bin/bash

VIDEO="${1}"
FPS="${2:-2}"
OUTPUT_DIR="${3:-./workspace/images}"

mkdir -p "$OUTPUT_DIR"

ffmpeg -i "$VIDEO" -vf fps=$FPS "$OUTPUT_DIR/frame_%04d.jpg"
