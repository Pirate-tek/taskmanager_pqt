#!/usr/bin/env python3

import sys
import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StopNode(Node):
    def __init__(self):
        super().__init__('stop_node')
        
        # This node now provides a service to shutdown everything
        self.srv = self.create_service(Trigger, '/shutdown_taskmanager', self.shutdown_callback)
        self.get_logger().info('Shutdown Service "/shutdown_taskmanager" is ready.')

    def shutdown_callback(self, request, response):
        self.get_logger().warn('SHUTDOWN REQUEST RECEIVED! Cleaning up system...')
        
        # List of process patterns to kill
        processes_to_kill = [
            'mission_executor_node',
            'nearest_waypoint_node',
            'control_node',
            'gzserver',
            'gzclient',
            'robot_state_publisher',
            'nav2_waypoint_follower',
            'nav2_bt_navigator',
            'nav2_map_server',
            'nav2_planner_server',
            'nav2_controller_server',
            'nav2_recoveries_server',
            'nav2_lifecycle_manager',
            'amcl'
        ]

        try:
            for proc in processes_to_kill:
                self.get_logger().info(f'Killing {proc}...')
                # Use pkill with full string matching to avoid accidental kills
                subprocess.run(['pkill', '-f', proc], check=False)
            
            response.success = True
            response.message = "System shutdown initiated successfully."
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
            response.success = False
            response.message = f"Error during shutdown: {str(e)}"

        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = StopNode()
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
