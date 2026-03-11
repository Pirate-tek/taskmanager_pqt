#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped, Twist
from nav2_msgs.action import NavigateToPose
from taskmanager_pqt.action import MissionTask
from std_srvs.srv import Trigger

class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__('mission_executor_node')
        
        # Always run on sim time if needed, typical for Nav2
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Nav2 client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Cmd Vel Publisher (must be TwistStamped for Nav2 config)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._listening = False
        
        # Stop handling
        self._stop_requested = False
        self._nav_goal_handle = None
        self._stop_service = self.create_service(
            Trigger, '/stop_mission', self.stop_mission_callback)
        
        # Create our Action Server
        self._action_server = ActionServer(
            self,
            MissionTask,
            'mission_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Mission Executor Action Server started.')

    # ------------------------------------------------------------------
    # Stop Service
    # ------------------------------------------------------------------
    def stop_mission_callback(self, request, response):
        self.get_logger().warn('--- STOP MISSION SERVICE INVOKED ---')
        self._stop_requested = True
        
        # Immediately halt hardware
        self.cmd_vel_pub.publish(self._make_twist_stamped())

        # If navigating, cancel the goal asynchronously
        if self._nav_goal_handle is not None:
            self.get_logger().info('Cancelling Nav2 goal...')
            # Notice we just trigger the cancel here, the Action
            # loop handles the rest.
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        response.success = True
        response.message = "Robot stopped successfully."
        return response

    # ------------------------------------------------------------------
    # Action Server Callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request to waypoint: {goal_request.waypoint_name}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        wp_name = req.waypoint_name
        self.get_logger().info(f'Executing MissionTask for {wp_name}...')
        self._stop_requested = False

        # If it's a maneuver, handle directly
        if wp_name.startswith('MANEUVER_ROT_'):
            try:
                parts = wp_name.split('_')
                angle_deg = float(parts[2])
                direction = parts[3] # CW or CCW
                
                angle_rad = math.radians(angle_deg)
                speed = 0.5
                if direction == "CW":
                    speed = -0.5 # Clockwise is negative angular velocity
                
                self.rotate(angle_rad, speed=speed)
                goal_handle.succeed()
                return MissionTask.Result(success=True, message=f'Rotation {angle_deg} {direction} complete')
            except Exception as e:
                self.get_logger().error(f'Failed to parse rotation maneuver: {e}')
                goal_handle.abort()
                return MissionTask.Result(success=False, message='Maneuver parsing failed')

        elif wp_name == 'MANEUVER_LISTEN':
            self.listen_cmd_vel(duration=5.0)
            if self._stop_requested:
                goal_handle.abort()
                return MissionTask.Result(success=False, message='Maneuver stopped')
            goal_handle.succeed()
            return MissionTask.Result(success=True, message='Finished listening to cmd_vel')

        # Otherwise, it's a Navigation goal
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            goal_handle.abort()
            return MissionTask.Result(success=False, message='Nav2 unavailable')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = req.target_pose

        self.get_logger().info(f'Sending Nav2 goal for {wp_name}...')
        send_goal_future = await self.nav_client.send_goal_async(nav_goal)
        
        if not send_goal_future.accepted:
            self.get_logger().error('Nav2 rejected goal.')
            goal_handle.abort()
            return MissionTask.Result(success=False, message='Goal rejected by Nav2')

        self._nav_goal_handle = send_goal_future

        get_result_future = await send_goal_future.get_result_async()
        
        self._nav_goal_handle = None

        # Check if we were cancelled during execution
        if self._stop_requested or get_result_future.status == 5: # CANCELED
            goal_handle.canceled()
            return MissionTask.Result(success=False, message='Navigation stopped by user')

        status = get_result_future.status
        if status == 4: # SUCCEEDED
            goal_handle.succeed()
            return MissionTask.Result(success=True, message=f'Successfully navigated to {wp_name}')
        else:
            goal_handle.abort()
            return MissionTask.Result(success=False, message=f'Navigation failed with status {status}')

    # ------------------------------------------------------------------
    # Maneuvers
    # ------------------------------------------------------------------
    def _make_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """Build a TwistStamped message (Nav2 enable_stamped_cmd_vel=true)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def rotate(self, angle, speed):
        dur = abs(angle / speed)
        t0 = time.time()
        while time.time() - t0 < dur:
            if self._stop_requested:
                break
            self.cmd_vel_pub.publish(self._make_twist_stamped(angular_z=speed))
            time.sleep(0.05)
        self.cmd_vel_pub.publish(self._make_twist_stamped())

    def listen_cmd_vel(self, duration=5.0):
        self.get_logger().info(f'Listening to /cmd_vel_external for {duration}s – send commands now.')
        self._listening = True
        sub = self.create_subscription(Twist, '/cmd_vel_external', self._cmd_vel_relay_cb, 10)
        
        # Check stop frequently
        t0 = time.time()
        while time.time() - t0 < duration:
            if self._stop_requested:
                break
            time.sleep(0.1)
            
        self.destroy_subscription(sub)
        self.cmd_vel_pub.publish(self._make_twist_stamped())  # stop
        self._listening = False
        self.get_logger().info('Stopped listening to external cmd_vel.')

    def _cmd_vel_relay_cb(self, msg):
        if self._listening:
            stamped = self._make_twist_stamped(
                linear_x=msg.linear.x, angular_z=msg.angular.z)
            self.cmd_vel_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    
    # Use multithreaded executor since we are calling Nav2 Action inside an Action callback
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
