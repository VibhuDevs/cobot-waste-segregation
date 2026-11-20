#!/usr/bin/env python3
"""Run the Gazebo track and periodically spawn physical test objects."""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class ConveyorManager(Node):
    def __init__(self):
        super().__init__('conveyor_manager')
        self.declare_parameter('velocity', 0.12)
        self.declare_parameter('spawn_interval', 6.0)
        self.declare_parameter('object_dimensions', [0.045, 0.045, 0.05])
        self.declare_parameter('object_mass', 0.08)
        self.declare_parameter('max_objects', 8)
        self.declare_parameter('spawn_x', 0.185)
        self.declare_parameter('spawn_y', 0.19)
        self.declare_parameter('spawn_z', 0.145)
        self.running = True
        self.spawn_count = 0
        self.pending = False
        self.cmd_pub = self.create_publisher(Float64, '/conveyor/cmd_vel', 10)
        self.spawn_client = self.create_client(SpawnEntity, '/world/default/create')
        self.toggle_service = self.create_service(SetBool, '/conveyor/set_running', self.set_running)
        self.timer = self.create_timer(0.25, self.tick)
        self.spawn_timer = self.create_timer(
            max(0.5, float(self.get_parameter('spawn_interval').value)), self.spawn_object)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info('Conveyor manager ready; use /conveyor/set_running to pause or resume')

    def on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'velocity' and parameter.value < 0.0:
                from rcl_interfaces.msg import SetParametersResult
                return SetParametersResult(successful=False, reason='velocity must be nonnegative')
            if parameter.name == 'spawn_interval' and parameter.value < 0.5:
                from rcl_interfaces.msg import SetParametersResult
                return SetParametersResult(
                    successful=False, reason='spawn_interval must be at least 0.5 seconds')
            if parameter.name == 'object_dimensions' and (
                    len(parameter.value) != 3 or min(parameter.value) <= 0.0):
                from rcl_interfaces.msg import SetParametersResult
                return SetParametersResult(
                    successful=False, reason='object_dimensions must contain three positive values')
        interval = next((p.value for p in parameters if p.name == 'spawn_interval'), None)
        if interval is not None:
            self.spawn_timer.cancel()
            self.destroy_timer(self.spawn_timer)
            self.spawn_timer = self.create_timer(float(interval), self.spawn_object)
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def set_running(self, request, response):
        self.running = request.data
        self.publish_velocity()
        response.success = True
        response.message = 'conveyor running' if self.running else 'conveyor stopped'
        return response

    def publish_velocity(self):
        msg = Float64()
        msg.data = float(self.get_parameter('velocity').value) if self.running else 0.0
        self.cmd_pub.publish(msg)

    def tick(self):
        self.publish_velocity()

    def object_sdf(self):
        dx, dy, dz = [float(value) for value in self.get_parameter('object_dimensions').value]
        mass = float(self.get_parameter('object_mass').value)
        ixx = mass * (dy * dy + dz * dz) / 12.0
        iyy = mass * (dx * dx + dz * dz) / 12.0
        izz = mass * (dx * dx + dy * dy) / 12.0
        return f'''<sdf version="1.7"><model name="conveyor_object"><link name="body">
          <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
          <collision name="collision"><geometry><box><size>{dx} {dy} {dz}</size></box></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
          <visual name="visual"><geometry><box><size>{dx} {dy} {dz}</size></box></geometry><material><ambient>0.9 0.04 0.03 1</ambient><diffuse>0.9 0.04 0.03 1</diffuse></material><plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>315</temperature></plugin></visual>
        </link></model></sdf>'''

    def spawn_object(self):
        if self.pending or self.spawn_count >= int(self.get_parameter('max_objects').value):
            return
        if not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service(timeout_sec=0.0)
            return
        self.spawn_count += 1
        name = f'red_object_{self.spawn_count:03d}'
        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.sdf = self.object_sdf()
        request.entity_factory.allow_renaming = False
        request.entity_factory.pose = Pose()
        request.entity_factory.pose.position.x = float(self.get_parameter('spawn_x').value)
        request.entity_factory.pose.position.y = float(self.get_parameter('spawn_y').value)
        request.entity_factory.pose.position.z = float(self.get_parameter('spawn_z').value)
        request.entity_factory.pose.orientation.w = 1.0
        self.pending = True
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda result: self.spawn_done(result, name))

    def spawn_done(self, future, name):
        self.pending = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Spawned {name}')
            else:
                self.spawn_count -= 1
                self.get_logger().warning(f'Gazebo rejected spawn for {name}')
        except Exception as exc:
            self.spawn_count -= 1
            self.get_logger().warning(f'Could not spawn {name}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
