#!/usr/bin/env python3
"""Stop-and-pick coordinator using MoveGroup and the existing gripper action."""

import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool


class PickPlaceCoordinator(Node):
    ARM_JOINTS = [
        'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
        'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange',
    ]

    def __init__(self):
        super().__init__('pick_place_coordinator')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('end_effector_link', 'link6_flange')
        self.declare_parameter('approach_height', 0.12)
        self.declare_parameter('grasp_height', 0.08)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('conveyor_length', 0.5)
        self.declare_parameter('conveyor_width', 0.16)
        self.declare_parameter('conveyor_height', 0.08)
        self.declare_parameter('conveyor_center_x', 0.32)
        self.declare_parameter('conveyor_center_y', 0.14)
        self.declare_parameter('collection_box_x', 0.15)
        self.declare_parameter('collection_box_y', -0.16)
        self.declare_parameter('collection_box_length', 0.16)
        self.declare_parameter('collection_box_width', 0.16)
        self.declare_parameter('collection_box_height', 0.10)
        self.latest_target = None
        self.busy = False
        self.retry_after = 0.0
        self.lock = threading.Lock()
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_action_controller/gripper_cmd')
        self.conveyor_client = self.create_client(SetBool, '/conveyor/set_running')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/perception/target', self.on_target, 10)
        self.get_logger().info('Coordinator waiting for a detected target')

    @staticmethod
    def wait_future(future, timeout, description):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f'{description} timed out')
        return future.result()

    def call_conveyor(self, running):
        if not self.conveyor_client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('conveyor set_running service is unavailable')
        request = SetBool.Request()
        request.data = running
        response = self.wait_future(
            self.conveyor_client.call_async(request), 8.0, 'conveyor service')
        if not response.success:
            raise RuntimeError(response.message)

    def on_target(self, message):
        if message.header.frame_id != self.get_parameter('planning_frame').value:
            self.get_logger().warning(
                f'Ignoring target in unexpected frame {message.header.frame_id!r}')
            return
        self.latest_target = message
        with self.lock:
            if self.busy or time.monotonic() < self.retry_after:
                return
            self.busy = True
        threading.Thread(target=self.run_cycle, daemon=True).start()

    def publish_environment(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        frame = self.get_parameter('planning_frame').value
        # Dimensions and poses mirror the static SDF environment.
        length = float(self.get_parameter('conveyor_length').value)
        width = float(self.get_parameter('conveyor_width').value)
        height = float(self.get_parameter('conveyor_height').value)
        center_x = float(self.get_parameter('conveyor_center_x').value)
        center_y = float(self.get_parameter('conveyor_center_y').value)
        box_x = float(self.get_parameter('collection_box_x').value)
        box_y = float(self.get_parameter('collection_box_y').value)
        box_length = float(self.get_parameter('collection_box_length').value)
        box_width = float(self.get_parameter('collection_box_width').value)
        box_height = float(self.get_parameter('collection_box_height').value)
        wall = 0.02
        obstacles = [
            ('conveyor_belt', (center_x, center_y, height), (length, width, 0.08)),
            ('collection_floor', (box_x, box_y, 0.01), (box_length, box_width, 0.02)),
            ('collection_x_plus', (box_x + box_length / 2 - wall / 2,
                                   box_y, box_height / 2),
             (wall, box_width, box_height)),
            ('collection_x_minus', (box_x - box_length / 2 + wall / 2,
                                    box_y, box_height / 2),
             (wall, box_width, box_height)),
            ('collection_y_plus', (box_x, box_y + box_width / 2 - wall / 2,
                                   box_height / 2),
             (box_length, wall, box_height)),
            ('collection_y_minus', (box_x, box_y - box_width / 2 + wall / 2,
                                    box_height / 2),
             (box_length, wall, box_height)),
        ]
        for name, xyz, dimensions in obstacles:
            obj = CollisionObject()
            obj.header.frame_id = frame
            obj.id = name
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(dimensions)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.w = 1.0
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)

    def pose_goal(self, pose_stamped):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'arm'
        request.allowed_planning_time = float(self.get_parameter('planning_time').value)
        request.num_planning_attempts = 8
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        position = PositionConstraint()
        position.header = pose_stamped.header
        position.link_name = self.get_parameter('end_effector_link').value
        volume = position.constraint_region
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]
        volume.primitives = [sphere]
        volume.primitive_poses = [pose_stamped.pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = pose_stamped.header
        orientation.link_name = position.link_name
        orientation.orientation = pose_stamped.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.35
        orientation.absolute_y_axis_tolerance = 0.35
        orientation.absolute_z_axis_tolerance = 3.14
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def move_to_pose(self, xyz):
        target = PoseStamped()
        target.header.frame_id = self.get_parameter('planning_frame').value
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        # Tool Z points down to approach the belt and the collection box.
        target.pose.orientation.x = 1.0
        self.get_logger().info(
            f'Planning flange pose at ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m')
        goal_handle = self.wait_future(
            self.move_client.send_goal_async(self.pose_goal(target)),
            10.0, 'MoveGroup goal acceptance')
        if not goal_handle.accepted:
            raise RuntimeError('MoveGroup rejected the pose goal')
        result = self.wait_future(goal_handle.get_result_async(), 90.0, 'MoveGroup execution')
        if result.result.error_code.val != result.result.error_code.SUCCESS:
            raise RuntimeError(f'MoveIt failed with code {result.result.error_code.val}')

    def move_home(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = float(self.get_parameter('planning_time').value)
        goal.request.num_planning_attempts = 8
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        constraints = Constraints()
        for name in self.ARM_JOINTS:
            from moveit_msgs.msg import JointConstraint
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = 0.0
            joint.tolerance_above = 0.04
            joint.tolerance_below = 0.04
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        handle = self.wait_future(
            self.move_client.send_goal_async(goal), 10.0, 'home goal acceptance')
        if not handle.accepted:
            raise RuntimeError('MoveGroup rejected the home goal')
        result = self.wait_future(handle.get_result_async(), 90.0, 'home execution')
        if result.result.error_code.val != result.result.error_code.SUCCESS:
            raise RuntimeError(f'Home motion failed with code {result.result.error_code.val}')

    def command_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 100.0
        handle = self.wait_future(
            self.gripper_client.send_goal_async(goal), 10.0, 'gripper goal acceptance')
        if not handle.accepted:
            raise RuntimeError('Gripper controller rejected the goal')
        result = self.wait_future(handle.get_result_async(), 15.0, 'gripper action')
        if not result.result.reached_goal and position > 0.0:
            raise RuntimeError('Gripper failed to reach its open position')

    def run_cycle(self):
        success = False
        try:
            if not self.move_client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError('MoveGroup action server is unavailable')
            if not self.gripper_client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError('Gripper action server is unavailable')
            self.call_conveyor(False)
            time.sleep(0.5)
            target = self.latest_target
            if target is None:
                raise RuntimeError('Target disappeared while stopping conveyor')
            x = target.pose.position.x
            y = target.pose.position.y
            z = target.pose.position.z
            approach = float(self.get_parameter('approach_height').value)
            grasp_height = float(self.get_parameter('grasp_height').value)

            self.publish_environment()
            time.sleep(0.2)
            self.command_gripper(0.15)
            self.move_to_pose((x, y, z + approach))
            # Keep tracking while the tool is moving to the pre-grasp pose.
            # Use the freshest depth-backed estimate for the final descent.
            latest_target = self.latest_target
            if latest_target is not None:
                x = latest_target.pose.position.x
                y = latest_target.pose.position.y
                z = latest_target.pose.position.z
                self.move_to_pose((x, y, z + approach))
            self.move_to_pose((x, y, z + grasp_height))
            self.command_gripper(-0.7)
            self.move_to_pose((x, y, z + approach))

            # Place over the open collection bin; objects are released by gravity.
            box_x = float(self.get_parameter('collection_box_x').value)
            box_y = float(self.get_parameter('collection_box_y').value)
            box_height = float(self.get_parameter('collection_box_height').value)
            approach_z = box_height + 0.10
            place_z = box_height + 0.06
            self.move_to_pose((box_x, box_y, approach_z))
            self.move_to_pose((box_x, box_y, place_z))
            self.command_gripper(0.15)
            self.move_home()
            success = True
            self.get_logger().info('Pick/place cycle completed')
        except Exception as exc:
            self.get_logger().error(f'Pick/place cycle stopped: {exc}')
            try:
                self.command_gripper(0.15)
            except Exception:
                pass
        finally:
            try:
                self.call_conveyor(True)
            except Exception as exc:
                self.get_logger().error(f'Could not resume conveyor: {exc}')
            if not success:
                self.retry_after = time.monotonic() + 4.0
                self.get_logger().warning('Waiting for a new target before retrying')
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceCoordinator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
