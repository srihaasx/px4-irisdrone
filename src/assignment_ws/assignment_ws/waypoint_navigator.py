import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from std_msgs.msg import String

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_setpoint = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',  10)
        self.pub_command  = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',      10)

        self.create_subscription(VehicleStatus,        '/fmu/out/vehicle_status',        self.on_status,    PX4_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_position,  PX4_QOS)
        self.create_subscription(String,               '/perception/detection',           self.on_detection, 10)

        self._counter       = 0
        self._armed         = False
        self._offboard      = False
        self._detect_count  = 0       # how many times detected
        self._box_found     = False   # confirmed after 5 detections
        self._above_box     = False
        self._box_x         = 0.0
        self._box_y         = 0.0
        self._cur_x         = 0.0
        self._cur_y         = 0.0
        self._land_z        = -3.0

        self.create_timer(0.1, self.tick)
        self.get_logger().info('Navigator started')

    def on_position(self, msg):
        self._cur_x = msg.x
        self._cur_y = msg.y

    def on_status(self, msg):
        self._armed    = (msg.arming_state == 2)
        self._offboard = (msg.nav_state == 14)

    def on_detection(self, msg):
        if msg.data == 'DETECTED' and not self._box_found:
            self._detect_count += 1
            # Keep updating box position — gets more accurate each detection
            self._box_x = self._cur_x
            self._box_y = self._cur_y
            self.get_logger().info(
                f'Detection {self._detect_count}/5 at x={self._box_x:.1f} y={self._box_y:.1f}'
            )
            # Only confirm after 5 detections — more centered position
            if self._detect_count >= 5:
                self._box_found = True
                self.get_logger().warn(
                    f'Box confirmed — returning to land at x={self._box_x:.1f} y={self._box_y:.1f}'
                )

    def tick(self):
        self._counter += 1
        self._send_offboard_mode()

        if self._counter == 10:
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self._counter == 15:
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if not self._armed or not self._offboard:
            self._send_position(0.0, 0.0, -3.0)
            return

        # Phase 1: fly forward until box confirmed
        if not self._box_found:
            self._send_position(20.0, 0.0, -3.0)
            return

        # Phase 2: return above box
        if not self._above_box:
            dx = abs(self._cur_x - self._box_x)
            dy = abs(self._cur_y - self._box_y)
            if dx < 0.5 and dy < 0.5:
                self._above_box = True
                self._land_z    = -3.0
                self.get_logger().warn('Above box — descending...')
            else:
                self._send_position(self._box_x, self._box_y, -3.0)
            return

        # Phase 3: descend slowly
        self._land_z += 0.02
        if self._land_z >= 0.0:
            self._land_z = 0.0
            if self._counter % 50 == 0:
                self.get_logger().info('Landed on box')
        self._send_position(self._box_x, self._box_y, self._land_z)

    def _send_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position  = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.pub_offboard.publish(msg)

    def _send_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position  = [x, y, z]
        msg.yaw       = 0.0
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.pub_setpoint.publish(msg)

    def _send_command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command=cmd; msg.param1=p1; msg.param2=p2
        msg.target_system=1; msg.target_component=1
        msg.source_system=1; msg.source_component=1
        msg.from_external=True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.pub_command.publish(msg)

def main():
    rclpy.init()
    node = WaypointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
