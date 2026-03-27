#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

# We'll use Trigger because it returns a success bool and a message string (which will hold "A", "B", etc.)
from std_srvs.srv import Trigger

class NearestWaypointServiceNode(Node):
    def __init__(self):
        super().__init__('nearest_waypoint_node')

        # Fix Jazzy use_sim_time parameter exception
        if self.has_parameter('use_sim_time'):
            self.get_logger().info('use_sim_time parameter already declared.')
        else:
            self.declare_parameter('use_sim_time', True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.waypoints = {
            'A': (-2.53, -2.2),
            'B': (1.16, 1.23),
            'C': (-2.38, 0.545),
            'D': (-2.3, 4.49),
            'E': (-1.7, -7.0),
            'F': (1.8, -6.6),
        }

        self.amcl_pose = None

        # Subscribe to AMCL pose with TRANSIENT_LOCAL to get the latest saved pose immediately
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_cb,
            qos_profile
        )

        # Start the Service Server
        self.srv = self.create_service(Trigger, '/get_nearest_waypoint', self.get_nearest_callback)
        self.get_logger().info('Nearest Waypoint Service Server started on /get_nearest_waypoint')

    def _pose_cb(self, msg):
        self.amcl_pose = msg.pose.pose

    def get_nearest_callback(self, request, response):
        self.get_logger().info('Received request for nearest waypoint.')
        
        if self.amcl_pose is None:
            self.get_logger().warn('No AMCL pose received yet.')
            response.success = False
            response.message = "No pose available"
            return response

        current_x = self.amcl_pose.position.x
        current_y = self.amcl_pose.position.y

        nearest_wp = None
        min_distance = float('inf')

        for name, coords in self.waypoints.items():
            dist = math.sqrt((coords[0] - current_x)**2 + (coords[1] - current_y)**2)
            if dist < min_distance:
                min_distance = dist
                nearest_wp = name

        self.get_logger().info(f'Calculated nearest: {nearest_wp} ({min_distance:.2f}m)')
        
        response.success = True
        response.message = nearest_wp
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NearestWaypointServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
