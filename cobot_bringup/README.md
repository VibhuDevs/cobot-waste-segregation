# Cobot 280 conveyor pick and place

## Start

From this workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cobot_bringup conveyor_pick_place.launch.py
```

The integrated launch starts Gazebo Sim 8 with `conveyor_pick_place.sdf`, the
Cobot model and ros2_control, MoveIt 2, the RGB-D tracker, the conveyor manager,
the pick/place coordinator, and the MoveIt RViz panel. Set
`enable_autonomy:=false` to inspect the simulation without motion. Set
`use_rviz:=false` for headless simulation.

The default belt and collection box are placed in the Cobot 280's reachable
workspace. To view the RGB and depth feeds in a separate RViz window while the
integrated simulation is running, use another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cobot_bringup camera_view.launch.py
```

The conveyor tracker uses the RGB-D camera mounted on the side stand to segment
red blocks and localize them with aligned depth. The overhead RGB-D and thermal
feeds remain available for inspection.

## ROS interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/camera_head/color/image_raw` | `sensor_msgs/Image` | RGB image from the overhead RGB-D camera |
| `/camera_head/depth/image_rect_raw` | `sensor_msgs/Image` | Aligned depth image in metres or millimetres |
| `/camera_head/depth/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics used for deprojection |
| `/camera_ir/image` | `sensor_msgs/Image` | Overhead thermal preview |
| `/camera_yolo/color/image_raw` | `sensor_msgs/Image` | Side-stand RGB image used to detect red blocks |
| `/camera_yolo/depth/image_rect_raw` | `sensor_msgs/Image` | Aligned depth used to localize detected blocks |
| `/camera_yolo/depth/camera_info` | `sensor_msgs/CameraInfo` | Side camera calibration |
| `/perception/objects` | `geometry_msgs/PoseArray` | Detected red object positions in `world` |
| `/perception/target` | `geometry_msgs/PoseStamped` | Tracked target predicted to the grasp horizon |
| `/conveyor/cmd_vel` | `std_msgs/Float64` | Conveyor velocity in m/s, positive along +X |
| `/conveyor/set_running` | `std_srvs/SetBool` | Stop or resume the conveyor |
| `/move_action` | `moveit_msgs/MoveGroup` action | Collision-aware arm planning and execution |
| `/gripper_action_controller/gripper_cmd` | `control_msgs/GripperCommand` action | Adaptive gripper open and close |

The conveyor manager parameters are in
`cobot_gazebo/config/conveyor.yaml`: belt speed, object spawn interval, object
dimensions, mass, maximum count, and spawn pose. Belt and collection box
dimensions and poses are launch parameters. The tracker segments red pixels
and uses the aligned side-camera depth for 3D localization.

## Current validation and limits

- The workspace builds; the conveyor SDF parses; MoveIt loads the Cobot model,
  KDL solver, OMPL planner, and arm/gripper controller definitions.
- The full launch was attempted here, but this execution sandbox denies Gazebo
  writes to `~/.gz` and denies ROS/Gazebo transport sockets. Gazebo exits before
  physics or camera topics can be checked, so object motion, RGB-D output,
  IK success, trajectory execution, and physical grasp retention are not yet
  verified.
- Perception thresholds red pixels in the stand camera's RGB feed and uses
  aligned depth. It does not estimate object orientation.
- The coordinator uses stop-and-pick and plans approach, grasp, lift, placement,
  and home motions with MoveIt. It does not yet verify object retention from
  contact feedback or attach the object collision body to the MoveIt scene.
- The MoveIt scene includes static belt and bin collision geometry. A live
  depth-derived octomap is not configured.
- Grasp verification and object attachment are not implemented; object
  retention depends on the simulated gripper's physical contact with the block.
