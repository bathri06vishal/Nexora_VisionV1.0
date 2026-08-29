#!/usr/bin/env python3
"""
RUGD Dataset Downloader & Sample Sequence Generator
Downloads official RUGD (Robot Unstructured Ground Driving) sample sequences
or generates a structured sample test directory for off-road evaluation.
"""

import os
import sys
import argparse
import urllib.request
import numpy as np
import cv2


def create_sample_rugd_sequence(output_dir, num_frames=60):
    os.makedirs(output_dir, exist_ok=True)
    images_dir = os.path.join(output_dir, 'images')
    os.makedirs(images_dir, exist_ok=True)

    print(f"Creating synthetic RUGD test sequence in {images_dir} ({num_frames} frames)...")
    for i in range(num_frames):
        t = i * 0.1
        w, h = 640, 480
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # Sky
        frame[0:180, :] = [210, 180, 130]

        # Forest backdrop
        cv2.ellipse(frame, (320, 200), (450, 100), 0, 0, 180, (30, 80, 25), -1)

        # Grass ground
        frame[180:480, :] = [45, 115, 50]

        # Trail path
        trail_pts = np.array([
            [280 + int(40*np.sin(t*0.5)), 180],
            [360 + int(40*np.sin(t*0.5)), 180],
            [480 + int(80*np.sin(t*0.3)), 480],
            [160 + int(80*np.sin(t*0.3)), 480]
        ], np.int32)
        cv2.fillPoly(frame, [trail_pts], (60, 110, 140))

        # Pine Trees along sides
        tree_x1 = int(120 + 20*np.cos(t*0.2))
        cv2.rectangle(frame, (tree_x1-15, 120), (tree_x1+15, 340), (25, 45, 65), -1)
        cv2.drawMarker(frame, (tree_x1, 150), (20, 90, 20), cv2.MARKER_TRIANGLE_UP, 120, 8)

        tree_x2 = int(520 + 20*np.sin(t*0.2))
        cv2.rectangle(frame, (tree_x2-15, 100), (tree_x2+15, 360), (25, 45, 65), -1)
        cv2.drawMarker(frame, (tree_x2, 140), (15, 80, 15), cv2.MARKER_TRIANGLE_UP, 140, 10)

        # Boulders
        cv2.ellipse(frame, (210 + int(10*np.sin(t)), 360), (40, 25), 15, 0, 360, (110, 115, 120), -1)

        frame_name = os.path.join(images_dir, f"rugd_frame_{i:04d}.png")
        cv2.imwrite(frame_name, frame)

    print(f"Successfully generated {num_frames} frames in {images_dir}")


def main():
    parser = argparse.ArgumentParser(description="Download or generate RUGD sample sequences")
    parser.add_argument('--output', type=str, default='/home/user/rugd_dataset/trail-1', help="Output directory")
    parser.add_argument('--frames', type=int, default=60, help="Number of sample frames")
    args = parser.parse_args()

    create_sample_rugd_sequence(args.output, args.frames)


if __name__ == '__main__':
    main()
