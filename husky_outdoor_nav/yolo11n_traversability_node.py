#!/usr/bin/env python3
"""
YOLO11n-seg Semantic Traversability & Perception Node for Husky Outdoor Navigation
Integrates Ultralytics YOLOv11 nano segmentation with RUGD off-road semantic cost mapping.
Publishes:
  - /perception/segmented_image (RGB with masks)
  - /perception/traversability_cost_image (Mono8 cost map: 0=free, 254=lethal)
  - /perception/semantic_pointcloud (PointCloud2 with semantic cost per 3D point)
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import struct
from ultralytics import YOLO


class YOLO11nTraversabilityNode(Node):
    def __init__(self):
        super().__init__('yolo11n_traversability_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        self.declare_parameter('enable_cuda', True)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.publish_pc = self.get_parameter('publish_pointcloud').get_parameter_value().bool_value
        use_cuda = self.get_parameter('enable_cuda').get_parameter_value().bool_value

        self.device = 'cuda:0' if (use_cuda and torch.cuda.is_available()) else 'cpu'
        self.get_logger().info(f'Loading YOLO11n-seg model from {self.model_path} on device: {self.device}')

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO11n-seg initialized successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None

        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = 381.36
        self.fy = 381.36
        self.cx = 320.0
        self.cy = 240.0
        self.cam_info_received = False

        # Latest depth frame cache
        self.latest_depth = None
        self.latest_depth_header = None

        # Publishers
        self.seg_pub = self.create_publisher(Image, '/perception/segmented_image', 10)
        self.cost_pub = self.create_publisher(Image, '/perception/traversability_cost_image', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/perception/semantic_pointcloud', 10)

        # Subscribers
        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.cam_info_topic, self.cam_info_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        # RUGD & General Off-Road Semantic Cost Matrix (0 = Smooth Trail, 254 = Lethal)
        # Standard COCO classes mapping + Terrain Heuristics
        self.class_cost_map = {
            # Obstacles (Lethal = 254)
            'person': 254, 'bicycle': 254, 'car': 254, 'motorcycle': 254,
            'bus': 254, 'truck': 254, 'bench': 254, 'dog': 254, 'horse': 254,
            'cow': 254, 'elephant': 254, 'bear': 254, 'potted plant': 254,
            'chair': 254, 'couch': 254, 'tree': 254, 'rock': 254, 'boulder': 254,
            'trunk': 254, 'log': 254, 'water': 254,
            # Traversable / Semi-traversable
            'grass': 40,
            'dirt': 10,
            'trail': 0,
            'bush': 120,
            'gravel': 80,
            'stones': 160
        }

        self.get_logger().info('YOLO11n-seg Traversability Node Ready.')

    def cam_info_callback(self, msg: CameraInfo):
        if not self.cam_info_received:
            self.fx = msg.k[0] if msg.k[0] > 0 else self.fx
            self.fy = msg.k[4] if msg.k[4] > 0 else self.fy
            self.cx = msg.k[2] if msg.k[2] > 0 else self.cx
            self.cy = msg.k[5] if msg.k[5] > 0 else self.cy
            self.cam_info_received = True

    def depth_callback(self, msg: Image):
        try:
            if '32F' in msg.encoding or '32f' in msg.encoding:
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.latest_depth = depth_16.astype(np.float32) / 1000.0  # mm to meters
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def compute_terrain_cost_mask(self, rgb_img, yolo_results):
        """
        Combines YOLO11n-seg instance segmentations and outdoor color/texture segmentation
        to produce a continuous 2D traversability cost image (0-254).
        """
        h, w = rgb_img.shape[:2]
        cost_mask = np.zeros((h, w), dtype=np.uint8)
        overlay_img = rgb_img.copy()

        # 1. Base terrain classification via HSV color space (RUGD off-road heuristics)
        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        
        # Dirt / Trail detection (Brownish / tan colors): Hue 10-25, Sat 40-200, Val 50-220
        lower_dirt = np.array([8, 30, 40])
        upper_dirt = np.array([30, 200, 220])
        dirt_mask = cv2.inRange(hsv, lower_dirt, upper_dirt)
        
        # Grass / Vegetation detection (Greenish colors): Hue 32-85, Sat 40-255, Val 40-240
        lower_grass = np.array([30, 35, 35])
        upper_grass = np.array([88, 255, 245])
        grass_mask = cv2.inRange(hsv, lower_grass, upper_grass)

        # Rock / Stone detection (Greyish, low saturation): Sat < 40, Val 50-180
        lower_stone = np.array([0, 0, 50])
        upper_stone = np.array([180, 45, 180])
        stone_mask = cv2.inRange(hsv, lower_stone, upper_stone)

        # Assign base costs
        cost_mask[dirt_mask > 0] = 0        # Trail / clear ground: 0 cost
        cost_mask[grass_mask > 0] = 35     # Light grass: 35 cost
        cost_mask[stone_mask > 0] = 120    # Rough stone / gravel: 120 cost

        # 2. Integrate YOLO11n-seg object & obstacle detections
        if yolo_results and len(yolo_results) > 0:
            res = yolo_results[0]
            if res.masks is not None:
                masks = res.masks.data.cpu().numpy() # [N, H_mask, W_mask]
                classes = res.boxes.cls.cpu().numpy().astype(int)
                confs = res.boxes.conf.cpu().numpy()

                for mask, cls_id, conf in zip(masks, classes, confs):
                    if conf < self.conf_thresh:
                        continue
                    class_name = res.names.get(cls_id, 'obstacle')
                    cost_val = self.class_cost_map.get(class_name, 200)

                    # Resize mask to original image size
                    mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                    binary_mask = (mask_resized > 0.5)

                    cost_mask[binary_mask] = np.maximum(cost_mask[binary_mask], cost_val)

                    # Color overlay for visualization
                    color = (0, 0, 255) if cost_val >= 200 else (0, 255, 0) if cost_val <= 40 else (0, 255, 255)
                    overlay_img[binary_mask] = cv2.addWeighted(overlay_img[binary_mask], 0.5, np.full_like(overlay_img[binary_mask], color), 0.5, 0)
                    
                    # Draw label text
                    ys, xs = np.where(binary_mask)
                    if len(xs) > 0:
                        cx, cy = int(np.mean(xs)), int(np.mean(ys))
                        cv2.putText(overlay_img, f'{class_name} {conf:.2f}', (cx-20, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        return cost_mask, overlay_img

    def generate_semantic_pointcloud(self, depth_img, cost_mask, rgb_img, header):
        """
        Unprojects depth image and cost mask into a 3D PointCloud2 with (x, y, z, rgb, cost).
        """
        h, w = depth_img.shape
        step = 4  # Downsample factor for high-rate transmission

        u_coords, v_coords = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
        z_vals = depth_img[::step, ::step]

        # Valid depth mask (0.3m to 20m)
        valid = (z_vals > 0.3) & (z_vals < 20.0) & np.isfinite(z_vals)
        if not np.any(valid):
            return None

        u_valid = u_coords[valid]
        v_valid = v_coords[valid]
        z_valid = z_vals[valid]

        # Unproject to camera frame
        x_valid = (u_valid - self.cx) * z_valid / self.fx
        y_valid = (v_valid - self.cy) * z_valid / self.fy

        costs = cost_mask[::step, ::step][valid].astype(np.float32)
        rgb_sub = rgb_img[::step, ::step][valid] # [M, 3] in BGR

        # Pack RGB into float32
        rgb_packed = []
        for bgr in rgb_sub:
            rgb_int = (int(bgr[2]) << 16) | (int(bgr[1]) << 8) | int(bgr[0])
            rgb_packed.append(struct.unpack('f', struct.pack('I', rgb_int))[0])
        rgb_packed = np.array(rgb_packed, dtype=np.float32)

        # Combine into [M, 5] array: x, y, z, rgb, cost
        points_data = np.stack([x_valid, y_valid, z_valid, rgb_packed, costs], axis=-1).astype(np.float32)

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.header.frame_id = 'camera_optical_frame'
        pc_msg.height = 1
        pc_msg.width = points_data.shape[0]
        pc_msg.is_dense = True
        pc_msg.is_bigendian = False

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='cost', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.fields = fields
        pc_msg.point_step = 20  # 5 floats * 4 bytes
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.data = points_data.tobytes()

        return pc_msg

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        # Run YOLO11n-seg inference
        yolo_results = None
        if self.model is not None:
            try:
                yolo_results = self.model.predict(
                    source=cv_image,
                    conf=self.conf_thresh,
                    device=self.device,
                    verbose=False
                )
            except Exception as e:
                self.get_logger().warn(f'YOLO prediction warning: {e}')

        # Compute traversability cost mask & visual overlay
        cost_mask, overlay_img = self.compute_terrain_cost_mask(cv_image, yolo_results)

        # Publish overlay image
        try:
            seg_msg = self.bridge.cv2_to_imgmsg(overlay_img, encoding='bgr8')
            seg_msg.header = msg.header
            self.seg_pub.publish(seg_msg)
        except Exception as e:
            self.get_logger().warn(f'Publish seg image error: {e}')

        # Publish mono cost mask image
        try:
            cost_msg = self.bridge.cv2_to_imgmsg(cost_mask, encoding='mono8')
            cost_msg.header = msg.header
            self.cost_pub.publish(cost_msg)
        except Exception as e:
            self.get_logger().warn(f'Publish cost mask error: {e}')

        # Publish 3D Semantic PointCloud if depth is available
        if self.publish_pc and self.latest_depth is not None:
            try:
                pc_msg = self.generate_semantic_pointcloud(
                    self.latest_depth, cost_mask, cv_image, msg.header
                )
                if pc_msg is not None:
                    self.pc_pub.publish(pc_msg)
            except Exception as e:
                self.get_logger().warn(f'PointCloud generation error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLO11nTraversabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
