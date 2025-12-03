#!/usr/bin/env python3
"""
Train a simple CNN to detect fish presence in images and export ONNX.

This is a stand-in training script that expects a folder dataset:
dataset_root/
  fish/      (images with fish)
  no_fish/   (images without fish)

Usage:
  python3 training/train_fish_presence.py --data ./my_dataset --epochs 5 --out models/fish_presence.onnx

Training runs on the host; the produced ONNX is stored under repo models/ so
Docker can use it for inference.
"""
import argparse
import os
from pathlib import Path
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as T
from torchvision.datasets import ImageFolder
from torch.utils.data import DataLoader


class TinyCNN(nn.Module):
    def __init__(self, num_classes=2):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.AdaptiveAvgPool2d((1,1))
        )
        self.fc = nn.Linear(64, num_classes)

    def forward(self, x):
        x = self.net(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)
        return x


def train(args):
    device = torch.device('cuda' if torch.cuda.is_available() and not args.cpu else 'cpu')
    transform = T.Compose([
        T.Resize((128, 128)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    data = ImageFolder(args.data, transform=transform)
    # Simple split
    n = len(data)
    n_train = int(0.9 * n)
    train_set, val_set = torch.utils.data.random_split(data, [n_train, n - n_train])
    train_loader = DataLoader(train_set, batch_size=args.batch, shuffle=True, num_workers=2)
    val_loader = DataLoader(val_set, batch_size=args.batch, shuffle=False, num_workers=2)

    model = TinyCNN(num_classes=len(data.classes)).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    best_acc = 0.0
    for epoch in range(args.epochs):
        model.train()
        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{args.epochs}")
        for imgs, labels in pbar:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            logits = model(imgs)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()
            pbar.set_postfix(loss=float(loss))

        # Validation
        model.eval()
        correct = 0
        total = 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                logits = model(imgs)
                preds = logits.argmax(1)
                correct += (preds == labels).sum().item()
                total += labels.numel()
        acc = correct / max(1, total)
        print(f"Val accuracy: {acc:.3f}")
        if acc > best_acc:
            best_acc = acc
            os.makedirs(os.path.dirname(args.out), exist_ok=True)
            # Save PyTorch for reference
            torch.save({'model': model.state_dict(), 'classes': data.classes}, Path(args.out).with_suffix('.pt'))
            # Export ONNX
            model_cpu = model.to('cpu').eval()
            dummy = torch.randn(1, 3, 128, 128)
            torch.onnx.export(
                model_cpu, dummy, args.out,
                input_names=['input'], output_names=['logits'],
                dynamic_axes={'input': {0: 'batch'}, 'logits': {0: 'batch'}},
                opset_version=13
            )
            print(f"Saved best model to {args.out}")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=str, required=True, help='Path to dataset root (ImageFolder format)')
    ap.add_argument('--out', type=str, default='models/fish_presence.onnx', help='Output ONNX path relative to repo root')
    ap.add_argument('--epochs', type=int, default=5)
    ap.add_argument('--batch', type=int, default=32)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--cpu', action='store_true', help='Force CPU training')
    return ap.parse_args()


if __name__ == '__main__':
    args = parse_args()
    # Resolve output path relative to repo root
    repo_root = Path(__file__).resolve().parents[1]
    args.out = str((Path(repo_root) / args.out).resolve())
    train(args)
