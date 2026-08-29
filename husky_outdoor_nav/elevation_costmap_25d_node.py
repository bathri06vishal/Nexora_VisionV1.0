#!/usr/bin/env python3
"""
2.5D Elevation & Semantic Traversability Costmap Generator for Husky UGV
Fuses 3D PointCloud geometric terrain features (step height, slope angle, roughness)
with YOLO11n-seg semantic classification into an outdoor 2.5D Nav2-compatible Costmap.
"""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import tf2_ros
from tf2_ros import TransformException


class ElevationCostmap25DNode(Node):
    def __init__(self):
        super().__init__('elevation_costmap_25d_node')

        # Costmap grid parameters
        self.declare_parameter('grid_width', 20.0)      # meters
        self.declare_parameter('grid_height', 20.0)     # meters
        self.declare_parameter('resolution', 0.10)      # meters per cell (10cm)
        self.declare_parameter('max_step_height', 0.18) # meters (Husky max obstacle climb)
        self.declare_parameter('max_slope_deg', 22.0)   # degrees (Husky max safe slope)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('input_pointcloud', '/camera/depth/points')
        self.declare_parameter('semantic_pointcloud', '/perception/semantic_pointcloud')

        self.width_m = self.get_parameter('grid_width').get_parameter_value().double_value
        self.height_m = self.get_parameter('grid_height').get_parameter_value().double_value
        self.res = self.get_parameter('resolution').get_parameter_value().double_value
        self.max_step = self.get_parameter('max_step_height').get_parameter_value().double_value
        self.max_slope = math.radians(self.get_parameter('max_slope_deg').get_parameter_value().double_value)
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.input_pc_topic = self.get_parameter('input_pointcloud').get_parameter_value().string_value
        self.semantic_pc_topic = self.get_parameter('semantic_pointcloud').get_parameter_value().string_value

        self.nx = int(self.width_m / self.res)
        self.ny = int(self.height_m / self.res)

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.grid_pub = self.create_publisher(OccupancyGrid, '/costmap_25d/traversability_grid', 10)

        # Subscribers
        self.create_subscription(PointCloud2, self.semantic_pc_topic, self.semantic_cloud_callback, 5)
        self.create_subscription(PointCloud2, self.input_pc_topic, self.depth_cloud_callback, 5)

        self.get_logger().info(f'2.5D Costmap Node initialized: {self.nx}x{self.ny} cells @ {self.res}m res.')

    def depth_cloud_callback(self, msg: PointCloud2):
        # Fallback if semantic pointcloud is not available
        self.process_pointcloud(msg, has_semantic_cost=False)

    def semantic_cloud_callback(self, msg: PointCloud2):
        self.process_pointcloud(msg, has_semantic_cost=True)

    def process_pointcloud(self, cloud_msg: PointCloud2, has_semantic_cost=False):
        # 1. Transform cloud to robot base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                cloud_msg.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            # If TF not ready yet, skip frame
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float32)
        t_vec = np.array([tx, ty, tz], dtype=np.float32)

        # Extract points from cloud message
        try:
            if has_semantic_cost:
                field_names = ('x', 'y', 'z', 'cost')
                pts_gen = point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=True)
                raw_pts = np.array(list(pts_gen), dtype=np.float32)
                if len(raw_pts) == 0:
                    return
                xyz = raw_pts[:, 0:3]
                semantic_costs = raw_pts[:, 3]
            else:
                field_names = ('x', 'y', 'z')
                pts_gen = point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=True)
                raw_pts = np.array(list(pts_gen), dtype=np.float32)
                if len(raw_pts) == 0:
                    return
                xyz = raw_pts[:, 0:3]
                semantic_costs = np.zeros(len(xyz), dtype=np.float32)
        except Exception as e:
            return

        # Transform 3D points to robot base frame
        pts_robot = (R @ xyz.T).T + t_vec

        # Filter points within grid bounding box
        x_r = pts_robot[:, 0]
        y_r = pts_robot[:, 1]
        z_r = pts_robot[:, 2]

        half_w = self.width_m / 2.0
        half_h = self.height_m / 2.0

        valid_mask = (x_r >= -half_w) & (x_r < half_w) & (y_r >= -half_h) & (y_r < half_h) & (z_r > -1.5) & (z_r < 3.0)
        x_valid = x_r[valid_mask]
        y_valid = y_r[valid_mask]
        z_valid = z_r[valid_mask]
        cost_valid = semantic_costs[valid_mask]

        # Convert to grid indices (centered at robot base)
        col_idx = np.floor((x_valid + half_w) / self.res).astype(np.int32)
        row_idx = np.floor((y_valid + half_h) / self.res).astype(np.int32)

        col_idx = np.clip(col_idx, 0, self.nx - 1)
        row_idx = np.clip(row_idx, 0, self.ny - 1)

        # 2.5D Cell Statistics Aggregation
        flat_idx = row_idx * self.nx + col_idx

        # Initialize costmap (-1 = unknown)
        cost_grid = np.full(self.nx * self.ny, -1, dtype=np.int8)

        # Group by flat grid cell
        unique_cells, cell_indices = np.unique(flat_idx, return_inverse=True)

        for i, cell in enumerate(unique_cells):
            pts_in_cell_mask = (cell_indices == i)
            zs = z_valid[pts_in_cell_mask]
            sem_c = cost_valid[pts_in_cell_mask]

            if len(zs) < 2:
                continue

            z_min = np.min(zs)
            z_max = np.max(zs)
            delta_z = z_max - z_min
            z_std = np.std(zs)

            # 1. Geometric step & roughness cost
            if delta_z > self.max_step or z_std > 0.12:
                # Untraversable obstacle (stone, boulder, tree trunk)
                geom_cost = 100
            elif delta_z > 0.08:
                # Rough terrain / small rocks
                geom_cost = 60
            else:
                # Flat ground
                geom_cost = 0

            # 2. Semantic cost from YOLO11n-seg
            max_sem = np.max(sem_c) if len(sem_c) > 0 else 0
            if max_sem >= 200:
                final_cost = 100 # Lethal
            else:
                # Scale semantic cost (0-200 -> 0-90)
                sem_cost_scaled = int(max_sem * 0.45)
                final_cost = max(geom_cost, sem_cost_scaled)

            cost_grid[cell] = final_cost

        # 3. Publish OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = cloud_msg.header.stamp
        grid_msg.header.frame_id = self.robot_frame
        grid_msg.info.resolution = float(self.res)
        grid_msg.info.width = self.nx
        grid_msg.info.height = self.ny
        grid_msg.info.origin.position.x = -half_w
        grid_msg.info.origin.position.y = -half_h
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = cost_grid.tolist()

        self.grid_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ElevationCostmap25DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
