"""
Nodo de ROS 2 (Humble): puntos 3D del cuerpo -> angulos de las articulaciones.

Entrada:  /human/keypoints (geometry_msgs/PoseArray, marco world, orden de
          kinematics.KEYPOINT_NAMES, NaN = no visto).
Salidas:  /joint_states (sensor_msgs/JointState): 4 de base + 18 del cuerpo,
          para robot_state_publisher / RViz.
          /human/cmd/<articulacion> (std_msgs/Float64): una por articulacion,
          que ros_gz_bridge lleva a los controladores de Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from human_shadow.kinematics import (
    ALL_JOINTS, forward_kinematics, KEYPOINT_NAMES, load_body_config,
    ShadowSolver)
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def default_body_config():
    try:
        return os.path.join(get_package_share_directory('human_shadow'),
                            'config', 'body.yaml')
    except Exception:  # paquete no instalado (p. ej. en pruebas)
        return ''


class ShadowNode(Node):

    def __init__(self):
        super().__init__('shadow_node')
        self.declare_parameter('body_config', default_body_config())
        # Publicar los comandos para Gazebo (/human/cmd/<articulacion>)
        self.declare_parameter('publish_gazebo_cmds', True)
        # Pegar los pies al piso: corrige base_z para que el tobillo mas bajo
        # quede a ankle_height. Evita que la sombra flote o se hunda.
        self.declare_parameter('ground_feet', True)
        self.declare_parameter('ankle_height', 0.08)

        body, limits = load_body_config(self.get_parameter('body_config').value)
        self.body = body
        self.solver = ShadowSolver(body, limits)
        self.ground = self.get_parameter('ground_feet').value
        self.ankle_h = self.get_parameter('ankle_height').value

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_pubs = {}
        if self.get_parameter('publish_gazebo_cmds').value:
            self.cmd_pubs = {j: self.create_publisher(Float64, f'human/cmd/{j}', 10)
                             for j in ALL_JOINTS}
        self.create_subscription(PoseArray, 'human/keypoints', self.kp_cb, 10)

        self.latencies = []
        self.create_timer(5.0, self.report_cb)
        self.get_logger().info('Nodo de sombra listo. Esperando /human/keypoints...')

    def kp_cb(self, msg: PoseArray):
        if len(msg.poses) != len(KEYPOINT_NAMES):
            self.get_logger().warn(
                f'Llegaron {len(msg.poses)} puntos, esperaba {len(KEYPOINT_NAMES)}',
                throttle_duration_sec=5.0)
            return
        kp = {}
        for name, pose in zip(KEYPOINT_NAMES, msg.poses):
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            if np.all(np.isfinite(p)):
                kp[name] = p

        q = self.solver.solve(kp)
        if not self.solver.has_base:
            return
        if self.ground:
            pts = forward_kinematics(q, self.body)
            low = min(pts['l_ankle'][2], pts['r_ankle'][2])
            q['base_z'] += self.ankle_h - low

        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = list(ALL_JOINTS)
        js.position = [q[j] for j in ALL_JOINTS]
        self.js_pub.publish(js)
        for j, pub in self.cmd_pubs.items():
            pub.publish(Float64(data=q[j]))

        # Latencia: desde que se tomo la imagen hasta que salen los angulos
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.latencies.append(now - stamp)

    def report_cb(self):
        if not self.latencies:
            return
        lat = np.array(self.latencies) * 1000.0
        self.get_logger().info(
            f'Angulos publicados: {len(lat) / 5.0:.1f} Hz | latencia imagen->angulos '
            f'media {lat.mean():.0f} ms, p95 {np.percentile(lat, 95):.0f} ms')
        self.latencies = []


def main(args=None):
    rclpy.init(args=args)
    node = ShadowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
