#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import cv2
import numpy as np

from path_config import MODEL_ROOT

def package_path(name: str) -> str:
    try:
        module = __import__(name)
        return str(getattr(module, "__file__", "built-in"))
    except Exception as exc:
        return f"IMPORT_FAILED: {type(exc).__name__}: {exc}"


def run_command(cmd: list[str]) -> str:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True, timeout=5)
        return out.strip()
    except Exception as exc:
        return f"FAILED: {type(exc).__name__}: {exc}"


def benchmark(model_path: str, imgsz: int, device: str, half: bool, loops: int) -> None:
    from ultralytics import YOLO

    image = np.zeros((512, 640, 3), dtype=np.uint8)
    model = YOLO(model_path, task="segment")
    for _ in range(5):
        model.predict(
            image,
            imgsz=imgsz,
            conf=0.7,
            verbose=False,
            device=device,
            half=half,
            max_det=5,
            retina_masks=False,
        )

    start = time.perf_counter()
    for _ in range(loops):
        model.predict(
            image,
            imgsz=imgsz,
            conf=0.7,
            verbose=False,
            device=device,
            half=half,
            max_det=5,
            retina_masks=False,
        )
    elapsed = time.perf_counter() - start
    print(f"BENCH {model_path}")
    print(f"  loops={loops}, elapsed={elapsed:.3f}s, fps={loops / elapsed:.2f}, avg_ms={elapsed * 1000.0 / loops:.1f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose YOLO/TensorRT/CUDA runtime performance.")
    parser.add_argument(
        "--engine",
        default=str(MODEL_ROOT / "box_segmentation" / "best.engine"),
    )
    parser.add_argument(
        "--pt",
        default=str(MODEL_ROOT / "box_segmentation" / "best.pt"),
    )
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="0")
    parser.add_argument("--loops", type=int, default=50)
    args = parser.parse_args()

    print("=== Runtime ===")
    print(f"python={sys.executable}")
    print(f"version={sys.version.replace(chr(10), ' ')}")
    print(f"platform={platform.platform()}")
    print(f"cwd={os.getcwd()}")
    print(f"PYTHONNOUSERSITE={os.environ.get('PYTHONNOUSERSITE')}")
    print(f"CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES')}")
    print()

    print("=== Packages ===")
    for name in ("ultralytics", "torch", "cv2", "numpy", "tensorrt"):
        print(f"{name}: {package_path(name)}")
    print(f"opencv={cv2.__version__}")
    print()

    print("=== NVIDIA ===")
    print(run_command(["nvidia-smi"]))
    print()

    print("=== Torch CUDA ===")
    cuda_available = False
    try:
        import torch

        print(f"torch={torch.__version__}")
        print(f"torch.version.cuda={torch.version.cuda}")
        cuda_available = bool(torch.cuda.is_available())
        print(f"cuda_available={cuda_available}")
        print(f"cuda_device_count={torch.cuda.device_count()}")
        if cuda_available:
            print(f"cuda_device_name={torch.cuda.get_device_name(0)}")
            print(f"cudnn={torch.backends.cudnn.version()}")
    except Exception as exc:
        print(f"Torch check failed: {type(exc).__name__}: {exc}")
    print()

    print("=== Model Files ===")
    for path_text in (args.engine, args.pt):
        path = Path(path_text)
        exists = path.exists()
        size_mb = path.stat().st_size / 1024 / 1024 if exists else 0.0
        print(f"{path}: exists={exists}, size={size_mb:.1f}MB")
    print()

    if args.device != "cpu" and not cuda_available:
        print("SKIP BENCH: CUDA device was requested but torch cannot see CUDA.")
        print("Run this on the real laptop terminal, not inside a restricted sandbox.")
        return

    if Path(args.engine).exists():
        benchmark(args.engine, args.imgsz, args.device, True, args.loops)
    if Path(args.pt).exists():
        benchmark(args.pt, args.imgsz, args.device, True, max(10, args.loops // 5))


if __name__ == "__main__":
    main()
