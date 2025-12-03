#!/usr/bin/env python3
"""
Train a video classification model for fish behavior from class-labeled videos and export ONNX.

Dataset layout (ImageFolder-like, but with videos):
  dataset_root/
    calm/
      clip001.mp4
      clip002.avi
      ...
    active/
      ...
    schooling/
      ...
    stressed/
      ...

Usage (host-side, not in Docker):
  python3 training/train_fish_behavior_video.py \
    --data /path/to/dataset \
    --epochs 10 \
    --clip-len 16 --size 112 \
    --out models/fish_behavior.onnx

Notes
- Uses torchvision r3d_18 backbone. Default loads pretrained weights if available (internet may be required once).
- Reads frames with OpenCV; ensure ffmpeg is installed on your system for best codec support.
- Exports ONNX with input shape (B, 3, T, H, W). Default T=16, H=W=112.
"""
import argparse
import os
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split

import torchvision.transforms as T
from torchvision.models.video import r3d_18
try:
    # Weights enum available in newer torchvision
    from torchvision.models.video import R3D_18_Weights  # type: ignore
    DEFAULT_WEIGHTS = R3D_18_Weights.DEFAULT
except Exception:  # pragma: no cover
    DEFAULT_WEIGHTS = None


def list_videos(root: Path) -> Tuple[List[Tuple[str, int]], List[str]]:
    exts = {'.mp4', '.avi', '.mov', '.mkv', '.webm'}
    cls_to_idx = {}
    samples: List[Tuple[str, int]] = []
    classes: List[str] = []
    for i, cls in enumerate(sorted([d.name for d in root.iterdir() if d.is_dir()])):
        cls_to_idx[cls] = i
        classes.append(cls)
        for p in sorted((root / cls).rglob('*')):
            if p.suffix.lower() in exts:
                samples.append((str(p), i))
    if not samples:
        raise RuntimeError(f"No video files found under {root}. Expected class subfolders with video files.")
    return samples, classes


class VideoClipDataset(Dataset):
    def __init__(self, root: str, clip_len: int = 16, size: int = 112):
        self.root = Path(root)
        self.clip_len = clip_len
        self.size = size
        self.samples, self.classes = list_videos(self.root)
        # Preprocessing to tensor per frame
        self.frame_tf = T.Compose([
            T.ToPILImage(),
            T.Resize((size, size)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

    def __len__(self):
        return len(self.samples)

    @staticmethod
    def _read_all_frames(path: str) -> List[np.ndarray]:
        cap = cv2.VideoCapture(path)
        frames = []
        if not cap.isOpened():
            return frames
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frames.append(frame)
        cap.release()
        return frames

    def _sample_indices(self, num_frames: int) -> List[int]:
        # Uniformly sample clip_len indices over the video; if too short, pad with last index
        if num_frames <= 0:
            return [0] * self.clip_len
        idxs = np.linspace(0, max(0, num_frames - 1), num=self.clip_len)
        idxs = np.clip(np.round(idxs).astype(int), 0, max(0, num_frames - 1))
        return idxs.tolist()

    def __getitem__(self, idx: int):
        path, label = self.samples[idx]
        frames = self._read_all_frames(path)
        if len(frames) == 0:
            # Fallback to a black clip
            frames = [np.zeros((self.size, self.size, 3), dtype=np.uint8)]
        inds = self._sample_indices(len(frames))
        clip_t = [self.frame_tf(frames[i]) for i in inds]  # list of (C,H,W)
        clip = torch.stack(clip_t, dim=1)  # (C,T,H,W)
        return clip, label


class BehaviorModel(nn.Module):
    def __init__(self, num_classes: int, pretrained: bool = True, freeze_backbone: bool = False):
        super().__init__()
        weights = DEFAULT_WEIGHTS if pretrained and DEFAULT_WEIGHTS is not None else None
        self.backbone = r3d_18(weights=weights)
        if freeze_backbone:
            for p in self.backbone.parameters():
                p.requires_grad = False
        in_features = self.backbone.fc.in_features
        self.backbone.fc = nn.Linear(in_features, num_classes)

    def forward(self, x):  # x: (B, C, T, H, W)
        return self.backbone(x)


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, device: torch.device) -> float:
    model.eval()
    correct = 0
    total = 0
    for x, y in loader:
        x, y = x.to(device), torch.as_tensor(y, device=device)
        logits = model(x)
        preds = logits.argmax(1)
        correct += (preds == y).sum().item()
        total += y.numel()
    return correct / max(1, total)


def train(args):
    device = torch.device('cuda' if torch.cuda.is_available() and not args.cpu else 'cpu')
    dataset = VideoClipDataset(args.data, clip_len=args.clip_len, size=args.size)
    num_classes = len(dataset.classes)

    # Split train/val
    n_total = len(dataset)
    n_train = int(0.8 * n_total)
    n_val = n_total - n_train
    train_set, val_set = random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_set, batch_size=args.batch, shuffle=True, num_workers=args.workers, pin_memory=True)
    val_loader = DataLoader(val_set, batch_size=args.batch, shuffle=False, num_workers=args.workers, pin_memory=True)

    model = BehaviorModel(num_classes=num_classes, pretrained=not args.no_pretrained, freeze_backbone=args.freeze_backbone).to(device)

    # If we froze the backbone, only train the classifier
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = optim.Adam(params, lr=args.lr, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss()

    best_acc = 0.0
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    for epoch in range(args.epochs):
        model.train()
        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{args.epochs}")
        for clips, labels in pbar:
            clips = clips.to(device)
            labels = torch.as_tensor(labels, device=device)
            optimizer.zero_grad()
            logits = model(clips)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()
            pbar.set_postfix(loss=float(loss))

        acc = evaluate(model, val_loader, device)
        print(f"Val accuracy: {acc:.3f}")
        if acc > best_acc:
            best_acc = acc
            # Save torch checkpoint with class names
            ckpt_path = Path(args.out).with_suffix('.pt')
            torch.save({'model': model.state_dict(), 'classes': dataset.classes, 'clip_len': args.clip_len, 'size': args.size}, ckpt_path)
            # Export ONNX
            model_cpu = model.to('cpu').eval()
            dummy = torch.randn(1, 3, args.clip_len, args.size, args.size)
            torch.onnx.export(
                model_cpu, dummy, args.out,
                input_names=['input'], output_names=['logits'],
                dynamic_axes={'input': {0: 'batch'}, 'logits': {0: 'batch'}},
                opset_version=13
            )
            # Save sidecar JSON with classes and clip config for inference
            try:
                import json
                sidecar = {
                    'classes': dataset.classes,
                    'clip_len': args.clip_len,
                    'size': args.size
                }
                with open(Path(args.out).with_suffix('.classes.json'), 'w') as f:
                    json.dump(sidecar, f, indent=2)
            except Exception as e:  # pragma: no cover
                print(f"Warning: failed to write classes JSON: {e}")
            print(f"Saved best model to {args.out}")
            model.to(device)

    print(f"Best val acc: {best_acc:.3f}")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=str, required=True, help='Path to dataset root with class subfolders of videos')
    ap.add_argument('--out', type=str, default='models/fish_behavior.onnx', help='Output ONNX path relative to repo root')
    ap.add_argument('--epochs', type=int, default=10)
    ap.add_argument('--batch', type=int, default=4)
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--clip-len', type=int, default=16, help='Number of frames per clip')
    ap.add_argument('--size', type=int, default=112, help='Frame spatial size (square)')
    ap.add_argument('--no-pretrained', action='store_true', help='Do not load pretrained weights for backbone')
    ap.add_argument('--freeze-backbone', action='store_true', help='Freeze backbone conv weights and only train classifier')
    ap.add_argument('--cpu', action='store_true', help='Force CPU training')
    return ap.parse_args()


if __name__ == '__main__':
    args = parse_args()
    # Resolve output path relative to repo root
    repo_root = Path(__file__).resolve().parents[1]
    args.out = str((Path(repo_root) / args.out).resolve())
    train(args)
