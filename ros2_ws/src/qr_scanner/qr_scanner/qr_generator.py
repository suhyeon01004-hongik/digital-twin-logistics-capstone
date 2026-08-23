#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
from pathlib import Path

import qrcode
from PIL import Image, ImageDraw, ImageFont


DEFAULT_OUTPUT_DIR = Path(
    os.environ.get("MILEMATE_QR_OUTPUT_DIR", str(Path.home() / "milemate_data" / "qr"))
).expanduser()
REGIONS = ("A", "B", "C", "D")
PACKAGE_SIZES = (1, 2, 3, 4)
QRS_PER_REGION = 25


def load_label_font(size):
    font_paths = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    )
    for font_path in font_paths:
        if os.path.exists(font_path):
            return ImageFont.truetype(font_path, size)
    return ImageFont.load_default()


def make_qr_image(data):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=18,
        border=4,
    )
    qr.add_data(data)
    qr.make(fit=False)
    return qr.make_image(fill_color="black", back_color="white").convert("RGB")


def add_readable_label(qr_img, label):
    label_font = load_label_font(32)
    draw_probe = ImageDraw.Draw(qr_img)
    bbox = draw_probe.textbbox((0, 0), label, font=label_font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]

    top_padding = 8
    bottom_padding = 16
    canvas_width = qr_img.width
    canvas_height = qr_img.height + top_padding + text_height + bottom_padding
    canvas = Image.new("RGB", (canvas_width, canvas_height), "white")
    canvas.paste(qr_img, (0, 0))

    draw = ImageDraw.Draw(canvas)
    text_x = (canvas_width - text_width) // 2
    text_y = qr_img.height + top_padding
    draw.text((text_x, text_y), label, fill="black", font=label_font)
    return canvas


def code_for(region, package_size, sequence):
    return f"{region}{package_size}-{sequence:03d}"


def generate_batch(output_dir=DEFAULT_OUTPUT_DIR):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    generated = []
    sequence = 1
    for region in REGIONS:
        for index in range(QRS_PER_REGION):
            package_size = PACKAGE_SIZES[index % len(PACKAGE_SIZES)]
            code = code_for(region, package_size, sequence)
            img = add_readable_label(make_qr_image(code), code)
            filename = output_dir / f"QR_{code}.png"
            img.save(filename)
            generated.append(filename)
            sequence += 1

    print(f"[성공] QR {len(generated)}개 생성 완료")
    print(f"저장 경로: {output_dir.resolve()}")
    print("예시 파일:")
    for filename in generated[:5]:
        print(f"- {filename}")


def generate_single(region, package_size, sequence, output_dir=DEFAULT_OUTPUT_DIR):
    region = region.upper()
    if region not in REGIONS:
        raise ValueError("배송 지역은 A, B, C, D 중 하나여야 합니다.")
    if package_size not in PACKAGE_SIZES:
        raise ValueError("택배 상자 크기는 1, 2, 3, 4 중 하나여야 합니다.")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    code = code_for(region, package_size, sequence)
    img = add_readable_label(make_qr_image(code), code)
    filename = output_dir / f"QR_{code}.png"
    img.save(filename)
    print(f"[성공] {filename.resolve()} 생성 완료")


def generate_logistic_qr():
    parser = argparse.ArgumentParser(description="택배 분류용 QR 생성기")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="QR 이미지 저장 경로")
    parser.add_argument("--single", nargs=3, metavar=("REGION", "SIZE", "NUMBER"), help="단일 QR 생성 예: --single A 1 23")
    args = parser.parse_args()

    if args.single:
        region, package_size, sequence = args.single
        generate_single(region, int(package_size), int(sequence), args.output_dir)
    else:
        generate_batch(args.output_dir)


if __name__ == "__main__":
    try:
        generate_logistic_qr()
    except KeyboardInterrupt:
        print("\n중단되었습니다.")
        sys.exit(0)
