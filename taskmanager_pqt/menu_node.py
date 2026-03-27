#!/usr/bin/env python3

import sys
import csv
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from datetime import datetime
from taskmanager_pqt.action import MissionTask


class MenuNode(Node):
    def __init__(self):
        super().__init__('menu_node')

        self.launch_sim = False
        for arg in sys.argv:
            if arg.lower() == 'launch_sim:=true':
                self.launch_sim = True

        self.waypoints = {
            'A': (-2.53, -2.2),
            'B': (1.16, 1.23),
            'C': (-2.38, 0.545),
            'D': (-2.3, 4.49),
            'E': (-1.7, -7.0),
            'F': (1.8, -6.6),
        }
        
        self.mission_logs = []
        self.subprocess_list = []
        self._shutdown_requested = False

        self.action_client = ActionClient(self, MissionTask, 'mission_task')

        if self.launch_sim:
            self.launch_simulation()

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
            self.get_logger().info('Simulation started. Waiting 30s...')
            time.sleep(30)
        except Exception as e:
            self.get_logger().error(f'Failed to launch sim: {e}')
            sys.exit(1)

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
                    success, duration = self.send_mission_goal(wp_name, self.waypoints[wp_name])
                    self.log_mission(wp_name, 'SUCCESS' if success else 'FAILURE', f'{duration:.2f}')
                    self.print_result(wp_name, 'SUCCESS' if success else 'FAILURE', duration)
                    
                    if success:
                        self.interactive_maneuvers_loop()

            except ValueError:
                print("Invalid input. Enter a number or 'END'.")
            except (EOFError, KeyboardInterrupt):
                self.terminate()
                break

    def send_mission_goal(self, wp_name, coords):
        self.get_logger().info(f'Sending action request for {wp_name}...')

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MissionTask server not available!')
            return False, 0.0

        goal_msg = MissionTask.Goal()
        goal_msg.waypoint_name = wp_name
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coords[0])
        pose.pose.position.y = float(coords[1])
        pose.pose.orientation.w = 1.0
        goal_msg.target_pose = pose

        start_time = time.time()
        
        # We must use proper asyncio/spinning to wait in a single-threaded architecture,
        # but since we are blocking the input loop, we will temporarily spin until the future is complete.
        send_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False, time.time() - start_time

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        duration = time.time() - start_time
        return result.success, duration

    def interactive_maneuvers_loop(self):
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

            cmd = None
            if choice == '1':
                cmd = 'MANEUVER_HALF_SPIN'
            elif choice == '2':
                cmd = 'MANEUVER_FULL_SPIN'
            elif choice == '3':
                cmd = 'MANEUVER_LISTEN'
            elif choice == '4':
                break
            else:
                print('Invalid choice.')
                continue
                
            if cmd:
                # Dispatch maneuver as an Action to the server
                print(f"Requesting {cmd} from the Executor...")
                goal_msg = MissionTask.Goal()
                goal_msg.waypoint_name = cmd
                
                send_fut = self.action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, send_fut)
                gh = send_fut.result()
                if gh.accepted:
                    res_fut = gh.get_result_async()
                    rclpy.spin_until_future_complete(self, res_fut)
                    print(res_fut.result().result.message)

    def log_mission(self, waypoint, result, duration):
        self.mission_logs.append({
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'waypoint': waypoint,
            'result': result,
            'duration': duration,
        })

    def print_result(self, wp, status, duration):
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
    node = MenuNode()
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
