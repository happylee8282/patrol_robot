#!/usr/bin/env python3
"""Downsample a ROS occupancy map while preserving its world coordinates."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re

import numpy as np
from PIL import Image


VALUE_PATTERN = r"^(?P<prefix>\s*{key}\s*:\s*)(?P<value>[^#\r\n]+)(?P<suffix>\s*(?:#.*)?)$"


def yaml_value(text: str, key: str) -> str:
    pattern = re.compile(VALUE_PATTERN.format(key=re.escape(key)), re.MULTILINE)
    match = pattern.search(text)
    if match is None:
        raise ValueError(f"YAML에 '{key}' 항목이 없습니다.")
    return match.group("value").strip().strip("'\"")


def replace_yaml_value(text: str, key: str, value: str) -> str:
    pattern = re.compile(VALUE_PATTERN.format(key=re.escape(key)), re.MULTILINE)
    if pattern.search(text) is None:
        raise ValueError(f"YAML에 '{key}' 항목이 없습니다.")
    return pattern.sub(lambda match: f"{match.group('prefix')}{value}{match.group('suffix')}", text, count=1)


def downsample_conservatively(image: Image.Image, factor: int, unknown: int) -> Image.Image:
    """Use the darkest pixel in each block so thin obstacles are not lost."""
    rgba = np.asarray(image.convert("RGBA"))
    if not np.all(rgba[:, :, 3] == 255):
        raise ValueError("투명도가 있는 지도는 지원하지 않습니다.")
    if not (np.array_equal(rgba[:, :, 0], rgba[:, :, 1]) and
            np.array_equal(rgba[:, :, 1], rgba[:, :, 2])):
        raise ValueError("흑백 occupancy map만 지원합니다.")

    gray = rgba[:, :, 0]
    height, width = gray.shape
    output_width = math.ceil(width / factor)
    output_height = math.ceil(height / factor)

    # ROS map origin is the lower-left corner. Array row 0 is the upper edge,
    # so padding goes at the top and right to leave the origin unchanged.
    padded = np.full(
        (output_height * factor, output_width * factor), unknown, dtype=np.uint8
    )
    top_padding = padded.shape[0] - height
    padded[top_padding:, :width] = gray

    result = padded.reshape(output_height, factor, output_width, factor).min(axis=(1, 3))
    return Image.fromarray(result, mode="L")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ROS occupancy map PNG와 YAML의 해상도를 함께 변환합니다."
    )
    parser.add_argument("yaml", type=Path, help="입력 map YAML 경로")
    parser.add_argument("resolution", type=float, help="출력 해상도 (m/pixel)")
    parser.add_argument("--output-stem", help="출력 파일명(확장자 제외)")
    parser.add_argument("--overwrite", action="store_true", help="기존 출력 파일 덮어쓰기")
    args = parser.parse_args()

    yaml_path = args.yaml.resolve()
    yaml_text = yaml_path.read_text(encoding="utf-8")
    image_path = (yaml_path.parent / yaml_value(yaml_text, "image")).resolve()
    old_resolution = float(yaml_value(yaml_text, "resolution"))
    new_resolution = args.resolution

    if old_resolution <= 0 or new_resolution <= 0:
        raise ValueError("해상도는 0보다 커야 합니다.")
    ratio = new_resolution / old_resolution
    factor = round(ratio)
    if factor < 1 or not math.isclose(ratio, factor, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("출력 해상도는 입력 해상도의 양의 정수 배수여야 합니다.")

    output_stem = args.output_stem or f"{yaml_path.stem}_{new_resolution:g}m"
    output_image = yaml_path.parent / f"{output_stem}.png"
    output_yaml = yaml_path.parent / f"{output_stem}.yaml"
    if not args.overwrite and (output_image.exists() or output_yaml.exists()):
        raise FileExistsError("출력 파일이 이미 있습니다. --overwrite를 사용하세요.")

    free_thresh = float(yaml_value(yaml_text, "free_thresh"))
    occupied_thresh = float(yaml_value(yaml_text, "occupied_thresh"))
    unknown_occupancy = (free_thresh + occupied_thresh) / 2.0
    unknown_gray = round(255 * (1.0 - unknown_occupancy))

    with Image.open(image_path) as image:
        old_size = image.size
        converted = downsample_conservatively(image, factor, unknown_gray)
        converted.save(output_image, optimize=True)

    output_text = replace_yaml_value(yaml_text, "image", output_image.name)
    output_text = replace_yaml_value(output_text, "resolution", f"{new_resolution:g}")
    output_yaml.write_text(output_text, encoding="utf-8")

    print(f"완료: {old_size[0]}x{old_size[1]} -> {converted.width}x{converted.height} px")
    print(f"해상도: {old_resolution:g} -> {new_resolution:g} m/pixel")
    print(f"이미지: {output_image}")
    print(f"YAML:   {output_yaml}")


if __name__ == "__main__":
    main()
