#!/usr/bin/env python3
"""
Husky Outdoor Navigation System Verification & Smoke Test
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid


class SystemSmokeTester(Node):
    def __init__(self):
        super().__init__('system_smoke_tester')
        self.received_seg = False
        self.received_cost = False
        self.received_cloud = False

        self.create_subscription(Image, '/perception/segmented_image', self.seg_cb, 10)
        self.create_subscription(Image, '/perception/traversability_cost_image', self.cost_cb, 10)
        self.create_subscription(PointCloud2, '/perception/semantic_pointcloud', self.cloud_cb, 10)

    def seg_cb(self, msg):
        self.received_seg = True

    def cost_cb(self, msg):
        self.received_cost = True

    def cloud_cb(self, msg):
        self.received_cloud = True


def main():
    print("Smoke test node ready.")


if __name__ == '__main__':
    main()
