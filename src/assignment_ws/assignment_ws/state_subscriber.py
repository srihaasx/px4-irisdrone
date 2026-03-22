#!/usr/bin/env python3
"""
Subscribes to PX4 vehicle state topics and logs them to console.
Topics read:
  - VehicleLocalPosition  (x, y, z, vx, vy, vz)
  - VehicleAttitude       (quaternion)
  - VehicleStatus         (armed state, nav mode)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class StateSubscriber(Node):
    def __init__(self):
        super().__init__('drone_state_monitor')
        self.get_logger().info('State monitor started — waiting for PX4 topics...')

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_position,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.on_attitude,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.on_status,
            PX4_QOS,
        )

        self._armed = False
        self._nav_state = -1

    def on_position(self, msg: VehicleLocalPosition):
        self.get_logger().info(
            f'POS  x={msg.x:+.2f}  y={msg.y:+.2f}  z={msg.z:+.2f} m  '
            f'vx={msg.vx:+.2f}  vy={msg.vy:+.2f}  vz={msg.vz:+.2f} m/s'
        )

    def on_attitude(self, msg: VehicleAttitude):
        q = msg.q
        self.get_logger().info(
            f'ATT  qw={q[0]:+.3f}  qx={q[1]:+.3f}  qy={q[2]:+.3f}  qz={q[3]:+.3f}'
        )

    def on_status(self, msg: VehicleStatus):
        armed = (msg.arming_state == 2)
        nav   = msg.nav_state
        if armed != self._armed or nav != self._nav_state:
            self._armed    = armed
            self._nav_state = nav
            self.get_logger().warn(
                f'STATUS CHANGE  armed={armed}  nav_state={nav}'
            )


def main():
    rclpy.init()
    node = StateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
