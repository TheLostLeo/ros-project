#!/bin/bash
set -e
DEST="$HOME/catkin_ws/src/ai/test_videos"
mkdir -p "$DEST"
URL="https://sample-videos.com/video123/mp4/720/big_buck_bunny_720p_1mb.mp4"
OUT="$DEST/sample.mp4"
if [ -f "$OUT" ]; then
  echo "Sample video already exists: $OUT"
  exit 0
fi
echo "Downloading sample video to $OUT..."
curl -L -o "$OUT" "$URL"
echo "Downloaded."
