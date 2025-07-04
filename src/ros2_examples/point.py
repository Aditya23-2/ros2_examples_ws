#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import math
import sys


class PandaRobotController(Node):
    """
    Custom Python node to control Panda robot arm using MoveIt2
    """
    
    def __init__(self):
        super().__init__('panda_robot_controller')
        
        # Initialize action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Robot configuration
        self.group_name = "panda_arm"
        self.end_effector_link = "panda_link8"
        self.planning_frame = "panda_link0"
        
        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        self.get_logger().info("Panda Robot Controller initialized successfully!")
        
        # Predefined poses for testing
        self.predefined_poses = {
            'home': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            'ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            'extended': [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785],
            'transport': [0.0, -0.5599, 0.0, -2.97, 0.0, 0.0, 0.785]
        }
    
    def move_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Move the robot end effector to a specific pose
        
        Args:
            x, y, z: Target position in meters
            roll, pitch, yaw: Target orientation in radians
        """
        self.get_logger().info(f"Moving to pose: x={x}, y={y}, z={z}")
        
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.planning_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Convert RPY to quaternion
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        target_pose.pose.orientation = quat
        
        # Create and send motion plan request
        return self._plan_and_execute_pose(target_pose)
    
    def move_to_joint_state(self, joint_positions):
        """
        Move the robot to specific joint positions
        
        Args:
            joint_positions: List of 7 joint angles in radians
        """
        if len(joint_positions) != 7:
            self.get_logger().error("Joint positions must contain exactly 7 values")
            return False
        
        self.get_logger().info(f"Moving to joint state: {joint_positions}")
        
        # Create motion plan request for joint space planning
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group_name
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        # Set joint constraints
        joint_names = [f"panda_joint{i}" for i in range(1, 8)]
        
        for i, (joint_name, position) in enumerate(zip(joint_names, joint_positions)):
            joint_constraint = Constraints()
            joint_constraint.name = f"joint_constraint_{i}"
            joint_constraint.joint_constraints.append(
                self._create_joint_constraint(joint_name, position)
            )
            motion_plan_request.goal_constraints.append(joint_constraint)
        
        # Create and send goal
        return self._execute_motion_plan(motion_plan_request)
    
    def move_to_predefined_pose(self, pose_name):
        """
        Move to a predefined pose
        
        Args:
            pose_name: Name of predefined pose ('home', 'ready', 'extended', 'transport')
        """
        if pose_name not in self.predefined_poses:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            self.get_logger().info(f"Available poses: {list(self.predefined_poses.keys())}")
            return False
        
        joint_positions = self.predefined_poses[pose_name]
        return self.move_to_joint_state(joint_positions)
    
    def move_relative(self, dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0):
        """
        Move the robot relative to its current position
        
        Args:
            dx, dy, dz: Relative position change in meters
            droll, dpitch, dyaw: Relative orientation change in radians
        """
        self.get_logger().info(f"Moving relative: dx={dx}, dy={dy}, dz={dz}")
        
        # Note: For a complete implementation, you would need to:
        # 1. Get current pose from /tf or robot state
        # 2. Add relative values to current pose
        # 3. Call move_to_pose with new target
        
        # For now, this is a placeholder - you can extend this
        self.get_logger().warn("Relative movement not fully implemented. Use absolute coordinates.")
        return False
    
    def _plan_and_execute_pose(self, target_pose):
        """
        Plan and execute motion to reach target pose
        """
        # Create motion plan request
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group_name
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Create bounding box for position (small tolerance)
        constraint_region = SolidPrimitive()
        constraint_region.type = SolidPrimitive.BOX
        constraint_region.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance
        
        position_constraint.constraint_region.primitives.append(constraint_region)
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        position_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        goal_constraint = Constraints()
        goal_constraint.position_constraints.append(position_constraint)
        goal_constraint.orientation_constraints.append(orientation_constraint)
        motion_plan_request.goal_constraints.append(goal_constraint)
        
        return self._execute_motion_plan(motion_plan_request)
    
    def _execute_motion_plan(self, motion_plan_request):
        """
        Execute the motion plan request
        """
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = motion_plan_request
        
        # Set planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 1.0
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        
        # Send goal
        self.get_logger().info("Sending motion plan request...")
        send_goal_future = self.move_group_client.send_goal_async(goal)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion completed successfully!")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {result.result.error_code.val}")
            return False
    
    def _create_joint_constraint(self, joint_name, target_position, tolerance=0.01):
        """
        Create a joint constraint for motion planning
        """
        from moveit_msgs.msg import JointConstraint
        
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = target_position
        joint_constraint.tolerance_above = tolerance
        joint_constraint.tolerance_below = tolerance
        joint_constraint.weight = 1.0
        
        return joint_constraint
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q


def main(args=None):
    """
    Main function to demonstrate robot control
    """
    rclpy.init(args=args)
    
    # Create controller node
    controller = PandaRobotController()
    
    try:
        # Example usage - uncomment the commands you want to test
        
        # 1. Move to predefined pose
        print("Moving to 'ready' pose...")
        controller.move_to_predefined_pose('ready')
        
        # 2. Move to specific Cartesian position
        # print("Moving to specific position...")
        # controller.move_to_pose(0.4, 0.0, 0.4)  # x=0.4m, y=0.0m, z=0.4m
        
        # 3. Move to custom joint configuration
        # print("Moving to custom joint configuration...")
        # custom_joints = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785]
        # controller.move_to_joint_state(custom_joints)
        
        # Keep node alive
        controller.get_logger().info("Robot controller ready! Press Ctrl+C to exit.")
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down robot controller...")
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
