#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped

class ShowPolygen(Node):
    def __init__(self):
        super().__init__('show_polygen')
        self.pubPolygen = self.create_publisher(PolygonStamped, 'poly', 1)
        self.get_logger().info('show_polygen has started')

def main(args=None):
    rclpy.init(args=args)

    show_polygen = ShowPolygen()

    # [scp_carry_plan-2] [INFO] [1721268521.004962708] [carry_plan]: Agent anchor: (-0.474995, -3.525000).
    # [scp_carry_plan-2] [INFO] [1721268521.004965529] [carry_plan]: Agent anchor: (-0.474995, -3.975000).
    # [scp_carry_plan-2] [INFO] [1721268521.004967261] [carry_plan]: Agent anchor: (-0.024995, -3.975000).
    # [scp_carry_plan-2] [INFO] [1721268521.004969029] [carry_plan]: Agent anchor: (0.375005, -3.900000).
    # [scp_carry_plan-2] [INFO] [1721268521.004970830] [carry_plan]: Agent anchor: (0.375005, -3.600000).
    # [scp_carry_plan-2] [INFO] [1721268521.004972575] [carry_plan]: Agent anchor: (-0.024995, -3.525000).


    poly = PolygonStamped()
    poly.header.frame_id = 'map'
    poly.header.stamp = show_polygen.get_clock().now().to_msg()
    poly.polygon.points = [
        Point32(x=-0.474995, y=-3.525000, z=0.0),
        Point32(x=-0.474995, y=-3.975000, z=0.0),
        Point32(x=-0.024995, y=-3.975000, z=0.0),
        Point32(x=0.375005, y=-3.900000, z=0.0),
        Point32(x=0.375005, y=-3.600000, z=0.0),
        Point32(x=-0.024995, y=-3.525000, z=0.0),
    ]
    show_polygen.pubPolygen.publish(poly)

    rclpy.spin(show_polygen)

    show_polygen.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()