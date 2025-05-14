#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
import math
import time
import threading

class MoveShelfToShip(Node):
    def __init__(self):
        super().__init__('move_shelf_to_ship')
        
        # Create ActionClient for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publisher for elevator control
        self.elevator_up_pub = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_down_pub = self.create_publisher(String, '/elevator_down', 10)
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        
        # Create clients for footprint parameter updates
        self.local_costmap_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters')
        
        self.global_costmap_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters')
        
        # Define positions - positions are approximate and should be adjusted for real environment
        # Format: {x, y, z, yaw} in map frame
        self.init_position = {'x': 4.713, 'y': -1.907, 'z': 0.0, 'yaw': -3.043}
        self.loading_position = {'x': 5.628, 'y': -0.688, 'z': 0.0, 'yaw': -1.592}
        self.elevator_up_position = {'x': 5.622, 'y': -1.483, 'z': 0.0, 'yaw': -1.584}
        self.shipping_position = {'x': 2.493, 'y': 1.473, 'z': 0.0, 'yaw': 1.585}
        
        # Define robot footprints
        self.normal_footprint = "[[-0.22, -0.22], [-0.22, 0.22], [0.22, 0.22], [0.22, -0.22]]"
        self.shelf_footprint = "[[-0.22, -0.4], [-0.22, 0.4], [0.4, 0.4], [0.4, -0.4]]"
        
        # Flag to check if Nav2 is active
        self.nav2_active = False
        
        self.get_logger().info('MoveShelfToShip node initialized')
        
        # Wait for services before continuing
        self.check_services_and_start()
    
    def check_services_and_start(self):
        """Check if all necessary services are available and then start the mission."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available after 10 seconds. Exiting...')
            return
            
        if not self.local_costmap_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Local costmap service not available after 10 seconds. Exiting...')
            return
            
        if not self.global_costmap_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Global costmap service not available after 10 seconds. Exiting...')
            return
            
        self.get_logger().info('All services are available! Starting mission...')
        
        # Run mission in a separate thread to not block the main ROS executor
        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.daemon = True
        self.mission_thread.start()
    
    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion components."""
        q_x = 0.0
        q_y = 0.0
        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)
        return q_x, q_y, q_z, q_w
    
    def drive_distance(self, distance, speed=0.15):
        """Drive the robot forward a certain distance."""
        self.get_logger().info(f'Driving forward {distance} meters at {speed} m/s...')
        
        # Calculate time required to cover the distance at given speed
        duration = abs(distance) / speed
        
        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = speed if distance > 0 else -speed  # Negative speed for backward motion
        
        # Send velocity commands for the calculated duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)
            
        # Stop the robot
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info('Drive complete')
        return True
    
    def navigate_to_pose(self, position, timeout=60.0):
        """Navigate the robot to a specific pose."""
        self.get_logger().info(f'Navigating to pose: x={position["x"]}, y={position["y"]}, yaw={position["yaw"]}')
        
        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = position['x']
        goal_pose.pose.position.y = position['y']
        goal_pose.pose.position.z = position['z']
        
        # Set orientation from yaw
        q_x, q_y, q_z, q_w = self._yaw_to_quaternion(position['yaw'])
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        
        # Create NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal and get future
        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()
        
        if not goal_handle:
            self.get_logger().error('Goal was rejected!')
            return False
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server')
            return False
            
        self.get_logger().info('Goal accepted by server, waiting for result...')
        
        # Get result future
        result_future = goal_handle.get_result_async()
        
        # Wait for the result with timeout
        start_time = time.time()
        while not result_future.done():
            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(f'Navigation timed out after {timeout} seconds')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
                return False
            
            # Print status every 10 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and elapsed - int(elapsed) < 0.1:
                self.get_logger().info(f'Still navigating... {int(elapsed)}s elapsed')
            
            # Sleep to avoid busy waiting
            time.sleep(0.1)
        
        # Get the result
        result = result_future.result().result
        
        if result.result == 0:  # NAVIGATE_TO_POSE_SUCCEEDED
            self.get_logger().info('Successfully reached the goal pose')
            return True
        else:
            # Error codes defined in nav2_msgs/action/NavigateToPose.action
            error_codes = {
                1: "NAVIGATE_TO_POSE_FAILED",
                2: "NAVIGATION_CANCELED"
            }
            error_msg = error_codes.get(result.result, f"Unknown error code: {result.result}")
            self.get_logger().error(f'Failed to reach the goal pose: {error_msg}')
            return False
    
    def update_footprint(self, with_shelf=False):
        """Update the robot's footprint in the Nav2 costmaps."""
        footprint_str = self.shelf_footprint if with_shelf else self.normal_footprint
        
        self.get_logger().info(f'Updating robot footprint to {"shelf" if with_shelf else "normal"} configuration')
        
        # Create parameter - make sure to use the correct way to set the string value
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_STRING
        param_value.string_value = footprint_str
        
        param = Parameter()
        param.name = 'footprint'
        param.value = param_value
        
        # Update parameters
        request = SetParameters.Request()
        request.parameters = [param]
        
        # Update local costmap
        self.get_logger().info('Updating local costmap footprint...')
        try:
            future_local = self.local_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future_local, timeout_sec=5.0)
            if not future_local.done():
                self.get_logger().error('Timeout waiting for local costmap update')
                return False
            result = future_local.result()
            if not result or not result.results[0].successful:
                self.get_logger().error(f'Failed to update local costmap footprint: {result.results[0].reason if result else "unknown error"}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error updating local costmap footprint: {e}')
            return False
        
        # Update global costmap
        self.get_logger().info('Updating global costmap footprint...')
        try:
            future_global = self.global_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future_global, timeout_sec=5.0)
            if not future_global.done():
                self.get_logger().error('Timeout waiting for global costmap update')
                return False
            result = future_global.result()
            if not result or not result.results[0].successful:
                self.get_logger().error(f'Failed to update global costmap footprint: {result.results[0].reason if result else "unknown error"}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error updating global costmap footprint: {e}')
            return False
        
        self.get_logger().info('Footprint updated successfully')
        return True
    
    def get_under_shelf(self):
        """Precisely move under the shelf."""
        self.get_logger().info('Moving under the shelf...')
        
        # Drive forward slowly to get under the shelf
        return self.drive_distance(0.5, speed=0.15)  # 0.5 meters forward
    
    def get_out_from_shelf(self):
        """Back away from the shelf after lowering it."""
        self.get_logger().info('Backing away from the shelf...')
        
        # Drive backward to get out from under the shelf
        return self.drive_distance(-0.5, speed=0.15)  # 0.5 meters backward
    
    def lift_shelf(self):
        """Send command to lift the shelf."""
        self.get_logger().info('Lifting the shelf...')
        
        # Create empty message (compatible with the elevator_up topic)
        msg = String()
        msg.data = ""
        
        # Send the command multiple times to ensure receipt
        for _ in range(5):
            self.elevator_up_pub.publish(msg)
            time.sleep(0.2)
        
        # Wait for lifting operation to complete
        time.sleep(2.0)
        
        self.get_logger().info('Shelf has been lifted')
        return True
    
    def lower_shelf(self):
        """Send command to lower the shelf."""
        self.get_logger().info('Lowering the shelf...')
        
        # Create empty message (compatible with the elevator_down topic)
        msg = String()
        msg.data = ""
        
        # Send the command multiple times to ensure receipt
        for _ in range(5):
            self.elevator_down_pub.publish(msg)
            time.sleep(0.2)
        
        # Wait for lowering operation to complete
        time.sleep(2.0)
        
        self.get_logger().info('Shelf has been lowered')
        return True
    
    def run_mission(self):
        """Run the complete shelf transportation mission."""
        try:
            # Step 1: Navigate to loading position first
            self.get_logger().info('=== STEP 1: Navigating to loading position ===')
            if not self.navigate_to_pose(self.loading_position, timeout=120.0):
                self.get_logger().error('Failed to reach loading position. Aborting mission.')
                return False
                
            # Step 2: Navigate to elevator position
            self.get_logger().info('=== STEP 2: Navigating to elevator position ===')
            if not self.navigate_to_pose(self.elevator_up_position, timeout=60.0):
                self.get_logger().error('Failed to reach elevator position. Aborting mission.')
                return False
            
            # Step 3: Get under the shelf and lift it
            self.get_logger().info('=== STEP 3: Getting under shelf and lifting it ===')
            if not self.get_under_shelf():
                self.get_logger().error('Failed to get under the shelf. Aborting mission.')
                return False
            
            if not self.lift_shelf():
                self.get_logger().error('Failed to lift the shelf. Aborting mission.')
                return False
            
            # Step 4: Update footprint to account for shelf
            self.get_logger().info('=== STEP 4: Updating footprint for shelf ===')
            if not self.update_footprint(True):
                self.get_logger().error('Failed to update footprint. Proceeding anyway with caution.')
            
            # Step 5: Navigate to shipping position (avoiding cones with keepout mask)
            self.get_logger().info('=== STEP 5: Navigating to shipping position ===')
            if not self.navigate_to_pose(self.shipping_position, timeout=120.0):
                self.get_logger().error('Failed to reach shipping position. Aborting mission.')
                return False
            
            # Step 6: Lower the shelf and back away
            self.get_logger().info('=== STEP 6: Lowering shelf and backing away ===')
            if not self.lower_shelf():
                self.get_logger().error('Failed to lower the shelf. Aborting mission.')
                return False
            
            if not self.get_out_from_shelf():
                self.get_logger().error('Failed to back away from the shelf. Aborting mission.')
                return False
            
            # Step 7: Update footprint back to normal
            self.get_logger().info('=== STEP 7: Updating footprint back to normal ===')
            if not self.update_footprint(False):
                self.get_logger().error('Failed to update footprint back to normal. Proceeding anyway with caution.')
            
            # Step 8: Return to initial position
            self.get_logger().info('=== STEP 8: Returning to initial position ===')
            if not self.navigate_to_pose(self.init_position, timeout=120.0):
                self.get_logger().error('Failed to return to initial position. Mission partially completed.')
                return False
            
            self.get_logger().info('=== MISSION COMPLETED SUCCESSFULLY ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during mission: {e}')
            return False

def main():
    # Initialize ROS
    rclpy.init()
    
    # Create the node
    node = MoveShelfToShip()
    
    # Spin the node to process callbacks
    rclpy.spin(node)
    
    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()