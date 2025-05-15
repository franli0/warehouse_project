#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import math
import time
import threading

class MoveShelfToShip(Node):
    def __init__(self):
        super().__init__('move_shelf_to_ship')
        
        # Create a navigator
        self.navigator = BasicNavigator()
        
        # Create publisher for elevator control
        self.elevator_up_pub = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_down_pub = self.create_publisher(String, '/elevator_down', 10)
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service clients for updating robot footprint
        self.local_costmap_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_costmap_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters')
        
        # Define robot footprints
        self.normal_footprint = "[[-0.22, -0.22], [-0.22, 0.22], [0.22, 0.22], [0.22, -0.22]]"
        self.shelf_footprint = "[[-0.22, -0.4], [-0.22, 0.4], [0.4, 0.4], [0.4, -0.4]]"
        
        # Define positions - simplified with slight adjustments
        self.init_position = {'x': 0.0, 'y': 0.0}
        self.loading_position = {'x': 3.9, 'y': -1.9, 'theta': -1.7}
        self.elevator_up_position = {'x': 3.7, 'y': -2.7, 'theta': -1.69}
        self.shipping_position = {'x': 1.7, 'y': 0.6, 'theta': 1.39}
        
        self.get_logger().info('Move Shelf To Ship node initialized')
    
    def update_footprint(self, with_shelf=False):
        """Update the robot's footprint in Nav2 costmaps."""
        footprint = self.shelf_footprint if with_shelf else self.normal_footprint
        self.get_logger().info(f'Updating robot footprint to {"shelf" if with_shelf else "normal"} size')
        
        # Create parameter
        param = Parameter()
        param.name = 'footprint'
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = footprint
        
        # Create request
        request = SetParameters.Request()
        request.parameters = [param]
        
        # Wait for services
        if not self.local_costmap_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Local costmap service not available')
            return False
            
        if not self.global_costmap_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Global costmap service not available')
            return False
        
        # Update local costmap
        future_local = self.local_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_local, timeout_sec=5.0)
        
        # Update global costmap
        future_global = self.global_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_global, timeout_sec=5.0)
        
        # Give time for costmaps to update
        time.sleep(2.0)
        self.get_logger().info('Footprint updated')
        return True
    
    def stop_robot(self):
        """Send explicit stop commands to ensure the robot is stopped."""
        self.get_logger().info('Stopping robot')
        
        # Create stop command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        # Send stop command multiple times
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.1)
        
        self.get_logger().info('Robot stopped')
    
    def drive_straight(self, distance, speed=0.15, timeout=30.0):
        """Drive the robot in a straight line."""
        self.get_logger().info(f'Driving {distance} meters at {speed} m/s')
        
        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = speed if distance > 0 else -speed
        
        # Publish for set duration
        duration = abs(distance) / speed
        start_time = time.time()
        
        while time.time() - start_time < min(duration, timeout):
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)
        
        # Stop the robot
        self.stop_robot()
        self.get_logger().info('Finished driving')
    
    def lift_shelf(self):
        """Send command to lift the shelf."""
        self.get_logger().info('Lifting shelf')
        msg = String()
        msg.data = ""
        
        # Send message multiple times
        for _ in range(10):
            self.elevator_up_pub.publish(msg)
            time.sleep(0.2)
        
        time.sleep(3.0)
    
    def lower_shelf(self):
        """Send command to lower the shelf."""
        self.get_logger().info('Lowering shelf')
        
        # First ensure the robot is completely stopped
        self.stop_robot()
        
        msg = String()
        msg.data = ""
        
        # Send message multiple times
        for _ in range(10):
            self.elevator_down_pub.publish(msg)
            time.sleep(0.2)
        
        time.sleep(3.0)
    
    def create_pose(self, position):
        """Create a PoseStamped message from position dictionary."""
        pose = PoseStamped()
        pose.header.frame_id = 'robot_map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = position['x']
        pose.pose.position.y = position['y']
        pose.pose.position.z = 0.0
        
        theta = position['theta']
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose
    
    def stabilize_at_pose(self, duration=3.0):
        """Pause to let the robot stabilize at current pose."""
        self.get_logger().info(f'Stabilizing at current pose for {duration} seconds')
        self.stop_robot()
        time.sleep(duration)
        self.get_logger().info('Stabilization complete')
    
    def navigate_with_retries(self, position, max_retries=3, care_about_orientation=True):
        """Navigate to position with retries."""
        self.get_logger().info(f'Navigating to x={position["x"]}, y={position["y"]}')
        
        # Create pose
        goal_pose = self.create_pose(position)
        
        for attempt in range(max_retries):
            self.get_logger().info(f'Navigation attempt {attempt+1}/{max_retries}')
            
            # Clear costmaps before navigation
            self.navigator.clearAllCostmaps()
            time.sleep(2.0)  # Give more time for costmaps to clear
            
            self.navigator.goToPose(goal_pose)
            
            # Wait with feedback
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
                time.sleep(1.0)
            
            # Check result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
                # Make sure robot is fully stopped
                self.stop_robot()
                # Allow it to stabilize
                self.stabilize_at_pose()
                return True
            else:
                self.get_logger().warn(f'Navigation failed: {result}')
                
                # If we're not at the final retry, wait before trying again
                if attempt < max_retries - 1:
                    self.get_logger().info('Waiting before retry...')
                    time.sleep(2.0)
        
        self.get_logger().error('All navigation attempts failed')
        return False
    
    def run_mission(self):
        """Run the shelf transport mission."""
        try:
            # Initialize navigation system
            self.get_logger().info('=== STARTING MISSION ===')
            self.navigator.waitUntilNav2Active()
            
            # Step 1: Navigate to loading position
            self.get_logger().info('=== STEP 1: Navigating to loading position ===')
            if not self.navigate_with_retries(self.loading_position):
                self.get_logger().error('Failed to reach loading position. Aborting mission.')
                return False
            
            # Step 2: Get under shelf and lift it
            self.get_logger().info('=== STEP 2: Loading shelf ===')
            self.drive_straight(1.0)  # Move forward to get under shelf
            self.lift_shelf()
            time.sleep(1.0)
            self.drive_straight(-1.3)
            
            # Step 3: Update footprint to account for shelf size
            self.get_logger().info('=== STEP 3: Updating footprint for shelf ===')
            self.update_footprint(with_shelf=True)

            # Step 4: Navigate to shipping position
            self.get_logger().info('=== STEP 5: Navigating to shipping position ===')
            if not self.navigate_with_retries(self.shipping_position):
                self.get_logger().error('Failed to reach shipping position. Aborting mission.')
                return False
            
            # Step 5: Lower shelf and back away
            self.get_logger().info('=== STEP 5: Unloading shelf ===')
            self.lower_shelf()
            time.sleep(1.0)
            self.drive_straight(-1.5)
            
            # Step 6: Update footprint back to normal
            self.get_logger().info('=== STEP 6: Updating footprint back to normal ===')
            self.update_footprint(with_shelf=False)
            
            # Step 7: Return to initial position
            self.get_logger().info('=== STEP 7: Returning to initial position ===')   
            if not self.navigate_with_retries(self.init_position):
                self.get_logger().error('Failed to return to initial position.')
                return False
            
            self.get_logger().info('=== MISSION COMPLETED SUCCESSFULLY ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during mission: {e}')
            return False

def main():
    rclpy.init()
    
    node = MoveShelfToShip()
    
    # Create executor for callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run executor in thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Run mission
        success = node.run_mission()
        if success:
            print("Mission completed successfully!")
        else:
            print("Mission failed.")
    except KeyboardInterrupt:
        print("Mission interrupted by user")
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()