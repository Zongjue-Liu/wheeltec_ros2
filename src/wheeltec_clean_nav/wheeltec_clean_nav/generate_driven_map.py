#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def load_trajectory(path):
    points = []
    with open(path, newline='') as handle:
        for row in csv.DictReader(handle):
            points.append((float(row['x']), float(row['y'])))
    if len(points) < 2:
        raise RuntimeError('trajectory contains fewer than two points')
    return points


def lookup_map_to_odom():
    rclpy.init()
    node = Node('driven_map_tf_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    transform = None
    deadline = time.monotonic() + 8.0
    while rclpy.ok() and transform is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            transform = buffer.lookup_transform(
                'map', 'odom_combined', rclpy.time.Time())
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()
    if transform is None:
        raise RuntimeError('map -> odom_combined transform is unavailable')
    t = transform.transform.translation
    q = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return t.x, t.y, yaw


def transform_points(points, transform):
    tx, ty, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(tx + c * x - s * y, ty + s * x + c * y) for x, y in points]


def world_to_pixel(point, origin, resolution, height):
    x, y = point
    ox, oy, oyaw = origin
    dx = x - ox
    dy = y - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    col = int(math.floor(local_x / resolution))
    grid_y = int(math.floor(local_y / resolution))
    return col, height - 1 - grid_y


def generate(args):
    with open(args.map_yaml) as handle:
        metadata = yaml.safe_load(handle)
    image_path = metadata['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(args.map_yaml), image_path)
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f'failed to read map image: {image_path}')

    resolution = float(metadata['resolution'])
    origin = [float(value) for value in metadata['origin']]
    points = transform_points(load_trajectory(args.trajectory), lookup_map_to_odom())
    pixels = np.array(
        [world_to_pixel(point, origin, resolution, image.shape[0]) for point in points],
        dtype=np.int32,
    )

    corridor = np.zeros(image.shape, dtype=np.uint8)
    radius_pixels = max(1, int(math.ceil(args.corridor_radius / resolution)))
    cv2.polylines(
        corridor, [pixels.reshape((-1, 1, 2))], False, 255,
        thickness=2 * radius_pixels, lineType=cv2.LINE_8)
    for col, row in pixels:
        cv2.circle(corridor, (int(col), int(row)), radius_pixels, 255, -1)

    negate = int(metadata.get('negate', 0))
    occupied_threshold = float(metadata.get('occupied_thresh', 0.65))
    free_threshold = float(metadata.get('free_thresh', 0.25))
    normalized = image.astype(np.float32) / 255.0
    occupancy = normalized if negate else 1.0 - normalized
    occupied = occupancy >= occupied_threshold
    free = occupancy <= free_threshold

    output = np.full(image.shape, 205, dtype=np.uint8)
    output[occupied] = 0
    output[(corridor > 0) & free] = 254

    output_dir = os.path.dirname(args.output_base)
    os.makedirs(output_dir, exist_ok=True)
    output_image = args.output_base + '.pgm'
    output_yaml = args.output_base + '.yaml'
    if not cv2.imwrite(output_image, output):
        raise RuntimeError(f'failed to write {output_image}')
    output_metadata = {
        'image': os.path.basename(output_image),
        'mode': metadata.get('mode', 'trinary'),
        'resolution': resolution,
        'origin': origin,
        'negate': negate,
        'occupied_thresh': occupied_threshold,
        'free_thresh': free_threshold,
    }
    with open(output_yaml, 'w') as handle:
        yaml.safe_dump(output_metadata, handle, sort_keys=False)
    print(f'driven map image: {output_image}')
    print(f'driven map yaml:  {output_yaml}')
    print(f'corridor radius:  {args.corridor_radius:.3f} m')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--trajectory', required=True)
    parser.add_argument('--output-base', required=True)
    parser.add_argument('--corridor-radius', type=float, default=0.24)
    generate(parser.parse_args())


if __name__ == '__main__':
    main()
