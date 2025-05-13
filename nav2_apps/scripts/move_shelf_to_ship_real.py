import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
import math
import time
import threading

class MoveShelfToShipReal(Node):
    def __init__(self):
        super().__init__('move_shelf_to_ship_real')
        
        # Create a navigator
        self.navigator = BasicNavigator()
        
        # Create publisher for elevator control
        self.elevator_up_pub = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_down_pub = self.create_publisher(String, '/elevator_down', 10)
        
        # Create publisher for velocity commands - different topic for real robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        # Create clients for footprint parameter updates
        self.local_costmap_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.global_costmap_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters')
        
        # Define positions - must be adjusted for real robot environment
        # Format: {x, y, z, yaw} in map frame
        self.init_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.loading_position = {'x': 1.5, 'y': 0.3, 'z': 0.0, 'yaw': 0.0}
        self.shipping_position = {'x': -1.5, 'y': -0.8, 'z': 0.0, 'yaw': 0.0}
        
        # Define robot footprints
        self.normal_footprint = [[-0.22, -0.22], [-0.22, 0.22], [0.22, 0.22], [0.22, -0.22]]
        self.shelf_footprint = [[-0.22, -0.4], [-0.22, 0.4], [0.4, 0.4], [0.4, -0.4]]
        
        self.get_logger().info('MoveShelfToShipReal node initialized for real robot')
    
    def localize_robot(self):
        """Localize the robot at the initial position."""
        self.get_logger().info('Localizing robot at init position...')
        
        # Create initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'robot_map'  # Changed from 'map' to 'robot_map' for real robot
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Set position
        initial_pose.pose.position.x = self.init_position['x']
        initial_pose.pose.position.y = self.init_position['y']
        initial_pose.pose.position.z = self.init_position['z']
        
        # Set orientation from yaw
        q_x, q_y, q_z, q_w = self._yaw_to_quaternion(self.init_position['yaw'])
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        
        # Set initial pose in the navigator
        self.navigator.setInitialPose(initial_pose)
        
        # Wait for Nav2 to be fully activated
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info('Robot successfully localized')
    
    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion components."""
        q_x = 0.0
        q_y = 0.0
        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)
        return q_x, q_y, q_z, q_w
    
    def navigate_to_pose(self, position, timeout=90.0):  # Increased timeout for real robot
        """Navigate the robot to a specific pose."""
        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'robot_map'  # Changed from 'map' to 'robot_map' for real robot
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
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
        
        # Send goal to navigator
        self.get_logger().info(f'Navigating to pose: x={position["x"]}, y={position["y"]}, yaw={position["yaw"]}')
        self.navigator.goToPose(goal_pose)
        
        # Wait for navigation to complete with timeout
        start_time = time.time()
        i = 0
        while not self.navigator.isTaskComplete():
            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(f'Navigation timed out after {timeout} seconds')
                self.navigator.cancelTask()
                return False
            
            # Get and log feedback periodically
            i += 1
            if i % 10 == 0:  # Only log every 10th iteration
                feedback = self.navigator.getFeedback()
                if feedback:
                    remaining = feedback.distance_remaining
                    self.get_logger().info(f'Distance remaining: {remaining:.2f} meters')
            
            time.sleep(0.1)
        
        # Check result
        result = self.navigator.getResult()
        success = result == BasicNavigator.TaskResult.SUCCEEDED
        
        if success:
            self.get_logger().info('Successfully reached the goal pose')
        else:
            self.get_logger().error(f'Failed to reach the goal pose. Result: {result}')
        
        return success
    
    def update_footprint(self, with_shelf=False):
        """Update the robot's footprint in the Nav2 costmaps."""
        footprint = self.shelf_footprint if with_shelf else self.normal_footprint
        footprint_str = str(footprint)
        
        self.get_logger().info(f'Updating robot footprint to {"shelf" if with_shelf else "normal"} configuration')
        
        # Create parameter
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_STRING
        param_value.string_value = footprint_str
        
        parameter = Parameter()
        parameter.name = 'footprint'
        parameter.value = param_value
        
        # Wait for services
        while not self.local_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for local costmap parameter service...')
        
        while not self.global_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for global costmap parameter service...')
        
        # Update parameters
        request = SetParameters.Request()
        request.parameters = [parameter]
        
        # Update local costmap
        self.get_logger().info('Updating local costmap footprint...')
        future_local = self.local_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_local)
        
        # Update global costmap
        self.get_logger().info('Updating global costmap footprint...')
        future_global = self.global_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_global)
        
        self.get_logger().info('Footprint updated successfully')
    
    def get_under_shelf(self):
        """Precisely move under the shelf."""
        self.get_logger().info('Moving under the shelf...')
        
        # Stop any current navigation
        self.navigator.cancelTask()
        
        # Create velocity command to move forward slowly
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.12  # Slower movement for real robot (0.12 m/s)
        
        # Publish for about 3 seconds to move forward
        for _ in range(30):
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)
        
        # Stop moving
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info('Robot now positioned under the shelf')
    
    def get_out_from_shelf(self):
        """Back away from the shelf after lowering it."""
        self.get_logger().info('Backing away from the shelf...')
        
        # Stop any current navigation
        self.navigator.cancelTask()
        
        # Create velocity command to move backward slowly
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.12  # Slower movement for real robot (0.12 m/s)
        
        # Publish for about 3 seconds to move backward
        for _ in range(30):
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)
        
        # Stop moving
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info('Robot has backed away from the shelf')
    
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
    
    def run_mission(self):
        """Run the complete shelf transportation mission."""
        try:
            # Step 1: Localize the robot
            self.get_logger().info('=== STEP 1: Localizing the robot ===')
            self.localize_robot()
            
            # Step 2: Navigate to loading position
            self.get_logger().info('=== STEP 2: Navigating to loading position ===')
            if not self.navigate_to_pose(self.loading_position):
                self.get_logger().error('Failed to reach loading position. Aborting mission.')
                return False
            
            # Step 3: Get under the shelf and lift it
            self.get_logger().info('=== STEP 3: Getting under shelf and lifting it ===')
            self.get_under_shelf()
            self.lift_shelf()
            
            # Step 4: Update footprint to account for shelf
            self.get_logger().info('=== STEP 4: Updating footprint for shelf ===')
            self.update_footprint(True)
            
            # Step 5: Navigate to shipping position (avoiding cones with keepout mask)
            self.get_logger().info('=== STEP 5: Navigating to shipping position ===')
            if not self.navigate_to_pose(self.shipping_position):
                self.get_logger().error('Failed to reach shipping position. Aborting mission.')
                return False
            
            # Step 6: Lower the shelf and back away
            self.get_logger().info('=== STEP 6: Lowering shelf and backing away ===')
            self.lower_shelf()
            self.get_out_from_shelf()
            
            # Step 7: Update footprint back to normal
            self.get_logger().info('=== STEP 7: Updating footprint back to normal ===')
            self.update_footprint(False)
            
            # Step 8: Return to initial position
            self.get_logger().info('=== STEP 8: Returning to initial position ===')
            if not self.navigate_to_pose(self.init_position):
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
    node = MoveShelfToShipReal()
    
    # Create executor for handling callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Run the mission
        success = node.run_mission()
        if success:
            print("Mission completed successfully!")
        else:
            print("Mission failed or was incomplete.")
    except Exception as e:
        node.get_logger().error(f'Mission failed with exception: {e}')
    finally:
        # Cleanup
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()