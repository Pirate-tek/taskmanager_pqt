#not in use for now
#!/usr/bin/env python3

import sys
import csv
import time
import math
import subprocess
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from datetime import datetime


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Parse custom CLI arguments for launch_sim
        self.launch_sim = False
        for arg in sys.argv:
            if arg.lower() == 'launch_sim:=true':
                self.launch_sim = True
            elif arg.lower() == 'launch_sim:=false':
                self.launch_sim = False

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.waypoints = {
            'A': (-2.53, -2.2),
            'B': (1.16, 1.23),
            'C': (-2.38, 0.545),
            'D': (-2.3, 4.49),
        }
        self.mission_logs = []
        self.subprocess_list = []
        self._shutdown_requested = False
        self._listening = False

        # Launch simulation if requested
        if self.launch_sim:
            self.launch_simulation()

    # ------------------------------------------------------------------
    # Simulation management
    # ------------------------------------------------------------------
    def launch_simulation(self):
        self.get_logger().info('Launching simulation (Gazebo + Nav2)...')
        try:
            self.sim_log = open('simulation.log', 'w')
            sim = subprocess.Popen(
                ['ros2', 'launch', 'taskmanager_pqt', 'mission_launch.py',
                 'use_sim_time:=true'],
                stdout=self.sim_log,
                stderr=subprocess.STDOUT,
            )
            self.subprocess_list.append(sim)
            self.get_logger().info(
                'Simulation started (logs → simulation.log). '
                'Waiting 30 s for components…')
            time.sleep(30)
        except Exception as e:
            self.get_logger().error(f'Failed to launch simulation: {e}')
            sys.exit(1)

    # ------------------------------------------------------------------
    # CLI menu
    # ------------------------------------------------------------------
    def display_menu(self):
        print('\n' + '=' * 40)
        print('          ROBOT MISSION MENU')
        print('=' * 40)
        print('Predefined Waypoints:')
        for name, coords in self.waypoints.items():
            print(f'  {name} -> {coords}')
        print('\nOptions:')
        print('  Number (1-4) -> How many waypoints to visit')
        print('  END          -> Terminate and generate report')
        print('=' * 40 + '\n')

    def run_menu_loop(self):
        while rclpy.ok() and not self._shutdown_requested:
            try:
                self.display_menu()
                val = input('Selection (or END): ').strip().upper()

                if val == 'END':
                    self.terminate()
                    break

                try:
                    num = int(val)
                    if num <= 0:
                        print('Please enter a positive number.')
                        continue

                    selected = []
                    for i in range(num):
                        wp = input(f'Target {i+1} (A/B/C/D): ').strip().upper()
                        if wp in self.waypoints:
                            selected.append(wp)
                        else:
                            print(f"Invalid waypoint '{wp}'. Skipping.")

                    for wp_name in selected:
                        if self._shutdown_requested:
                            break
                        self.navigate_to_waypoint(wp_name)

                except ValueError:
                    print("Invalid input. Enter a number or 'END'.")

            except (EOFError, KeyboardInterrupt):
                self.terminate()
                break

    # ------------------------------------------------------------------
    # Navigation (direct Nav2 action client)
    # ------------------------------------------------------------------
    def navigate_to_waypoint(self, wp_name):
        coords = self.waypoints[wp_name]
        self.get_logger().info(
            f'Navigating to {wp_name} ({coords[0]}, {coords[1]})…')

        # Build goal pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coords[0])
        pose.pose.position.y = float(coords[1])
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Wait for the action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.log_mission(wp_name, 'FAILURE', '0.00')
            self.print_result(wp_name, 'FAILURE', 0.0)
            return

        start_time = time.time()

        # Send goal (synchronous-style via threading Event)
        self._nav_done = threading.Event()
        self._nav_status = None

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, wp_name))

        # Block the menu thread until navigation finishes
        self._nav_done.wait()

        duration = time.time() - start_time
        status = self._nav_status or 'FAILURE'

        # Post-arrival maneuvers on success
        if status == 'SUCCESS':
            self.get_logger().info('Navigation succeeded – choose a maneuver.')
            self.perform_post_arrival()
            duration = time.time() - start_time

        self.log_mission(wp_name, status, f'{duration:.2f}')
        self.print_result(wp_name, status, duration)

    def _on_goal_response(self, future, wp_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal for {wp_name} rejected by Nav2.')
            self._nav_status = 'FAILURE'
            self._nav_done.set()
            return

        self.get_logger().info(f'Goal for {wp_name} accepted. Waiting…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self._nav_status = 'SUCCESS'
        else:
            self.get_logger().error(
                f'Navigation failed (status code {result.status}).')
            self._nav_status = 'FAILURE'
        self._nav_done.set()

    # ------------------------------------------------------------------
    # Post-arrival maneuvers
    # ------------------------------------------------------------------
    def perform_post_arrival(self):
        """Show maneuver menu and execute the user's choice."""
        while True:
            print('\n' + '-' * 40)
            print('  POST-ARRIVAL MANEUVERS')
            print('-' * 40)
            print('  1 -> Half Spin   (180°)')
            print('  2 -> Full Spin   (360°)')
            print('  3 -> Listen to /cmd_vel for 5 s')
            print('  4 -> NONE (proceed to next waypoint)')
            print('-' * 40)

            choice = input('Maneuver (1-4): ').strip()

            if choice == '1':
                print('Executing half spin…')
                self.rotate(math.pi, speed=0.5)
            elif choice == '2':
                print('Executing full spin…')
                self.rotate(2 * math.pi, speed=0.5)
            elif choice == '3':
                print('Listening to /cmd_vel for 5 seconds…')
                self.listen_cmd_vel(duration=5.0)
            elif choice == '4':
                break
            else:
                print('Invalid choice. Enter 1-4.')

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
            self.cmd_vel_pub.publish(self._make_twist_stamped(angular_z=speed))
            time.sleep(0.05)
        self.cmd_vel_pub.publish(self._make_twist_stamped())

    def listen_cmd_vel(self, duration=5.0):
        """Subscribe to /cmd_vel_external and relay messages for `duration` seconds."""
        self.get_logger().info(
            f'Listening to /cmd_vel_external for {duration}s – send commands now.')
        self._listening = True

        sub = self.create_subscription(
            Twist, '/cmd_vel_external', self._cmd_vel_relay_cb, 10)

        time.sleep(duration)
        self.destroy_subscription(sub)
        self.cmd_vel_pub.publish(self._make_twist_stamped())  # stop
        self._listening = False
        self.get_logger().info('Stopped listening to external cmd_vel.')

    def _cmd_vel_relay_cb(self, msg):
        if self._listening:
            stamped = self._make_twist_stamped(
                linear_x=msg.linear.x, angular_z=msg.angular.z)
            self.cmd_vel_pub.publish(stamped)

    # ------------------------------------------------------------------
    # Logging & reporting
    # ------------------------------------------------------------------
    def log_mission(self, waypoint, result, duration):
        self.mission_logs.append({
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'waypoint': waypoint,
            'result': result,
            'duration': duration,
        })

    @staticmethod
    def print_result(wp, status, duration):
        print('\n' + '-' * 40)
        print(f'MISSION {wp} COMPLETED')
        print(f'Status:   {status}')
        print(f'Duration: {duration:.2f}s')
        print('-' * 40 + '\n')
        if status == 'FAILURE':
            input('Press Enter to return to menu…')
        else:
            time.sleep(2.0)

    def generate_report(self):
        report = 'mission_report.csv'
        self.get_logger().info(f'Generating report → {report}')
        try:
            with open(report, 'w', newline='') as f:
                w = csv.DictWriter(
                    f, fieldnames=['timestamp', 'waypoint', 'result', 'duration'])
                w.writeheader()
                w.writerows(self.mission_logs)
            self.get_logger().info('Report saved. Mission terminated.')
        except Exception as e:
            self.get_logger().error(f'Failed to save report: {e}')

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def terminate(self):
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.generate_report()
        self.get_logger().info('Shutting down simulation processes…')
        for proc in self.subprocess_list:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_menu_loop()
    except Exception as e:
        print(f'Error in menu loop: {e}')
    finally:
        node.terminate()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
