#!/usr/bin/env python3
"""
RUGD Dataset Streamer / Player Node for Husky Navigation Testing
Streams raw camera frames, depth maps, and IMU data from the RUGD off-road vision dataset
or synthetic off-road forest frames to test YOLO11n-seg, cuVSLAM, and 2.5D costmaps.
"""

import os
import glob
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np


class RUGDDatasetPlayerNode(Node):
    def __init__(self):
        super().__init__('rugd_dataset_player_node')

        self.declare_parameter('dataset_path', '')
        self.declare_parameter('sequence_name', 'trail-1')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.dataset_path = self.get_parameter('dataset_path').get_parameter_value().string_value
        self.seq_name = self.get_parameter('sequence_name').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self.image_files = []
        self.current_idx = 0

        # Look for dataset files
        if self.dataset_path and os.path.exists(self.dataset_path):
            seq_dir = os.path.join(self.dataset_path, self.seq_name)
            if not os.path.exists(seq_dir):
                seq_dir = self.dataset_path
            
            patterns = ['*.png', '*.jpg', '*.jpeg']
            for p in patterns:
                self.image_files.extend(sorted(glob.glob(os.path.join(seq_dir, '**', p), recursive=True)))

        if len(self.image_files) > 0:
            self.get_logger().info(f'Loaded {len(self.image_files)} RUGD frames from {self.dataset_path}')
        else:
            self.get_logger().info('No external RUGD dataset path provided. Using real-time synthetic forest terrain camera generator.')

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.step_cnt = 0

    def generate_synthetic_forest_frame(self, t):
        """Generates realistic forest trail test frame when running dataset player standalone."""
        w, h = 640, 480
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # Sky
        frame[0:180, :] = [210, 180, 130] # Soft blue sky

        # Distant forest / hills
        cv2.ellipse(frame, (320, 200), (450, 100), 0, 0, 180, (30, 80, 25), -1)

        # Ground grass
        frame[180:480, :] = [45, 115, 50] # Forest grass

        # Curving dirt trail
        trail_pts = np.array([
            [280 + int(40*np.sin(t*0.5)), 180],
            [360 + int(40*np.sin(t*0.5)), 180],
            [480 + int(80*np.sin(t*0.3)), 480],
            [160 + int(80*np.sin(t*0.3)), 480]
        ], np.int32)
        cv2.fillPoly(frame, [trail_pts], (60, 110, 140)) # Brownish trail

        # Pine Trees along trail
        tree_x1 = int(120 + 20*np.cos(t*0.2))
        cv2.rectangle(frame, (tree_x1-15, 120), (tree_x1+15, 340), (25, 45, 65), -1) # Trunk
        cv2.drawMarker(frame, (tree_x1, 150), (20, 90, 20), cv2.MARKER_TRIANGLE_UP, 120, 8)

        tree_x2 = int(520 + 20*np.sin(t*0.2))
        cv2.rectangle(frame, (tree_x2-15, 100), (tree_x2+15, 360), (25, 45, 65), -1)
        cv2.drawMarker(frame, (tree_x2, 140), (15, 80, 15), cv2.MARKER_TRIANGLE_UP, 140, 10)

        # Boulders / rocks on side
        cv2.ellipse(frame, (210, 360), (40, 25), 15, 0, 360, (110, 115, 120), -1)

        # Depth map (meters)
        depth = np.ones((h, w), dtype=np.float32) * 15.0
        for y in range(180, 480):
            d_val = 15.0 - (y - 180) * (13.5 / 300.0) # 15m to 1.5m
            depth[y, :] = d_val
        depth[340:375, 180:250] = 3.5 # Rock depth

        return frame, depth

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.step_cnt += 1
        t_sec = self.step_cnt / self.publish_rate

        if len(self.image_files) > 0:
            img_file = self.image_files[self.current_idx]
            cv_img = cv2.imread(img_file)
            if cv_img is None:
                return
            h, w = cv_img.shape[:2]
            # Synthetic depth matching image size
            depth_map = np.ones((h, w), dtype=np.float32) * 6.0
            self.current_idx += 1
            if self.current_idx >= len(self.image_files):
                if self.loop:
                    self.current_idx = 0
                else:
                    self.get_logger().info('Finished dataset stream.')
                    return
        else:
            cv_img, depth_map = self.generate_synthetic_forest_frame(t_sec)

        # Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # Publish Depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = self.frame_id
        self.depth_pub.publish(depth_msg)

        # Publish Camera Info
        cam_info = CameraInfo()
        cam_info.header.stamp = now
        cam_info.header.frame_id = self.frame_id
        cam_info.width = cv_img.shape[1]
        cam_info.height = cv_img.shape[0]
        f = 380.0
        cx = cam_info.width / 2.0
        cy = cam_info.height / 2.0
        cam_info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        cam_info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.cam_info_pub.publish(cam_info)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.z = 0.05 * np.sin(t_sec)
        imu_msg.orientation.w = 1.0
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RUGDDatasetPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
