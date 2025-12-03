# Training (host-side)

This folder contains simple scripts to train models for fish presence (images) and fish behavior (videos) outside of Docker. The resulting ONNX files are saved under the repository `models/` folder so the runtime nodes can load them.

## Environment setup
Install Python packages (Linux, system Python recommended):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r training/requirements.txt
```

Ensure FFmpeg is installed so OpenCV can read most video formats:

```bash
# Debian/Ubuntu
sudo apt-get update && sudo apt-get install -y ffmpeg
```

## Datasets

- Presence (images):
  - ImageFolder format
  - Layout:
    ```
    dataset_root/
      fish/
        img001.jpg
        ...
      no_fish/
        img101.jpg
        ...
    ```

- Behavior (videos):
  - Class-labeled video clips
  - Layout:
    ```
    dataset_root/
      calm/
        clip001.mp4
        ...
      active/
        ...
      schooling/
        ...
      stressed/
        ...
    ```

## Train fish presence (images)

```bash
python3 training/train_fish_presence.py \
  --data /path/to/image_dataset \
  --epochs 5 \
  --out models/fish_presence.onnx
```

Artifacts:
- `models/fish_presence.onnx` (ONNX model)
- `models/fish_presence.pt` (PyTorch weights + class names)

## Train fish behavior (videos)

```bash
python3 training/train_fish_behavior_video.py \
  --data /path/to/video_dataset \
  --epochs 10 \
  --clip-len 16 --size 112 \
  --out models/fish_behavior.onnx
```

Artifacts:
- `models/fish_behavior.onnx` (ONNX model)
- `models/fish_behavior.pt` (PyTorch weights + class names)
- `models/fish_behavior.classes.json` (classes and clip config)

## Tips
- Set `--cpu` to force CPU if you don’t have a compatible GPU.
- Increase `--clip-len` (e.g., 32) and `--size` (e.g., 160) for better accuracy at the cost of speed and memory.
- Use `--freeze-backbone` to train only the classifier if you have a small dataset.

## Basic OpenCV fish detector (no ROS)

Install minimal deps:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r training/requirements-opencv.txt
```

Run with webcam:

```bash
python3 training/opencv_fish_detector.py --source 0
```

Run with a video file:

```bash
python3 training/opencv_fish_detector.py --source /path/to/video.mp4 --method hsv
```

- Press `m` to toggle method (shape/hsv); `q` to quit.
- The script prints fish presence and a random mood (happy/disturbed) and overlays results in a window.

Auto-start the bot via teleop web when fish are detected (presence rising edge):

```bash
python3 training/opencv_fish_detector.py --source 0 \
  --send-start-on-detect \
  --teleop-url http://localhost:8080 \
  --start-cooldown-sec 10
```

Notes:
- The teleop web server must be running and reachable at the given URL.
- A cooldown prevents repeated triggers if detection flickers.
