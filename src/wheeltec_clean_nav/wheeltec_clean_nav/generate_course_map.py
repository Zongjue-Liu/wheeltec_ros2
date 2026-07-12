#!/usr/bin/env python3
"""Generate the dimensioned 3.23 m x 3.55 m training-course map."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


RESOLUTION = 0.01
MARGIN = 0.10
COURSE_WIDTH = 3.23
COURSE_HEIGHT = 3.55
WALL_THICKNESS = 0.03

# Coordinates are measured from the physical lower-left course corner.
ISLANDS = (
    (0.50, 0.50, 1.45, 1.25),
    (1.93, 0.50, 2.73, 1.25),
    (0.50, 1.74, 1.45, 3.05),
    (1.93, 1.74, 2.73, 3.05),
)


def px(value: float) -> int:
    return round((value + MARGIN) / RESOLUTION)


def py(value: float, image_height: int) -> int:
    return image_height - 1 - px(value)


def draw_rounded_obstacle(
        image: np.ndarray, bounds: tuple[float, float, float, float], radius: float) -> None:
    x0, y0, x1, y1 = bounds
    left, right = px(x0), px(x1)
    top, bottom = py(y1, image.shape[0]), py(y0, image.shape[0])
    r = round(radius / RESOLUTION)
    cv2.rectangle(image, (left + r, top), (right - r, bottom), 0, -1)
    cv2.rectangle(image, (left, top + r), (right, bottom - r), 0, -1)
    for center in (
            (left + r, top + r), (right - r, top + r),
            (left + r, bottom - r), (right - r, bottom - r)):
        cv2.circle(image, center, r, 0, -1)


def generate(output_dir: Path) -> None:
    width = round((COURSE_WIDTH + 2 * MARGIN) / RESOLUTION)
    height = round((COURSE_HEIGHT + 2 * MARGIN) / RESOLUTION)
    image = np.full((height, width), 205, dtype=np.uint8)

    # The road is free. The gray margin is unknown and cannot be planned through.
    cv2.rectangle(
        image,
        (px(0.0), py(COURSE_HEIGHT, height)),
        (px(COURSE_WIDTH), py(0.0, height)),
        254,
        -1,
    )

    wall_px = max(1, round(WALL_THICKNESS / RESOLUTION))
    cv2.rectangle(
        image,
        (px(0.0), py(COURSE_HEIGHT, height)),
        (px(COURSE_WIDTH), py(0.0, height)),
        0,
        wall_px,
    )
    for island in ISLANDS:
        draw_rounded_obstacle(image, island, radius=0.10)

    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / 'WHEELTEC.pgm'
    for path, extension in (
            (pgm_path, '.pgm'),
            (output_dir / 'WHEELTEC_DIMENSIONED_PREVIEW.png', '.png')):
        encoded, data = cv2.imencode(extension, image)
        if not encoded:
            raise RuntimeError(f'failed to encode map image: {path}')
        data.tofile(str(path))
    metadata = {
        'image': 'WHEELTEC.pgm',
        'mode': 'trinary',
        'resolution': RESOLUTION,
        'origin': [-MARGIN, -MARGIN, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with (output_dir / 'WHEELTEC.yaml').open('w', encoding='ascii') as handle:
        yaml.safe_dump(metadata, handle, sort_keys=False)


def default_output_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('wheeltec_clean_nav')) / 'maps'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'maps'


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Generate the dimensioned WHEELTEC course map.')
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='map output directory; defaults to the installed package maps directory',
    )
    parsed = parser.parse_args(args)
    generate(parsed.output_dir or default_output_dir())


if __name__ == '__main__':
    main()
