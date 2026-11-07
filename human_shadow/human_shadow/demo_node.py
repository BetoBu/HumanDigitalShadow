"""
Nodo de demostracion: una persona virtual que publica /human/keypoints.

Camina en circulo, saluda y se agacha. Sirve para probar el modelo, los
angulos y Gazebo SIN camara.

Como los puntos salen de la cinematica directa con angulos conocidos,
tambien publica esos angulos en /human/ground_truth (sensor_msgs/JointState)
para comparar contra lo que calcula shadow_node.
"""

import math

from geometry_msgs.msg import Point, Pose, PoseArray
from human_shadow.kinematics import (
    ALL_JOINTS, forward_kinematics, KEYPOINT_NAMES, load_body_config)
from human_shadow.shadow_node import default_body_config
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def demo_pose(t):
    """Angulos de la persona virtual en el tiempo t (segundos)."""
    walk = 0.6 * math.sin(2.0 * t)
    squat = max(0.0, math.sin(0.25 * t)) ** 4          # se agacha de vez en cuando
    wave = 0.5 + 0.5 * math.sin(4.0 * t)
    yaw = 0.15 * t
    return {
        'base_x': 2.5 + 0.6 * math.cos(yaw), 'base_y': 0.6 * math.sin(yaw),
        'base_z': 0.93 - 0.35 * squat, 'base_yaw': yaw + math.pi / 2,
        'torso_flex': 0.1 + 0.5 * squat, 'neck_flex': 0.15 * math.sin(t),
        'l_shoulder_flex': -0.4 * walk, 'l_shoulder_abd': 0.1,
        'l_elbow_flex': 0.4,
        'r_shoulder_flex': 0.3, 'r_shoulder_abd': 1.3, 'r_shoulder_rot': 1.2,
        'r_elbow_flex': 0.6 + 0.8 * wave,                 # saluda con la derecha
        'l_hip_flex': walk + 1.4 * squat, 'r_hip_flex': -walk + 1.4 * squat,
        'l_hip_abd': 0.05, 'r_hip_abd': 0.05,
        'l_knee_flex': 0.2 + 0.4 * max(0.0, walk) + 2.0 * squat,
        'r_knee_flex': 0.2 + 0.4 * max(0.0, -walk) + 2.0 * squat,
    }


class DemoNode(Node):

    def __init__(self):
        super().__init__('demo_node')
        self.declare_parameter('body_config', default_body_config())
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('world_frame', 'world')
        self.body, _ = load_body_config(self.get_parameter('body_config').value)
        self.frame = self.get_parameter('world_frame').value
        self.kp_pub = self.create_publisher(PoseArray, 'human/keypoints', 10)
        self.gt_pub = self.create_publisher(JointState, 'human/ground_truth', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info('Demo: persona virtual publicando en /human/keypoints')

    def tick(self):
        now = self.get_clock().now()
        q = demo_pose((now - self.t0).nanoseconds * 1e-9)
        pts = forward_kinematics(q, self.body)

        msg = PoseArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame
        for name in KEYPOINT_NAMES:
            p = pts[name]
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            msg.poses.append(pose)
        self.kp_pub.publish(msg)

        gt = JointState()
        gt.header.stamp = msg.header.stamp
        gt.name = list(ALL_JOINTS)
        gt.position = [float(q.get(j, 0.0)) for j in ALL_JOINTS]
        self.gt_pub.publish(gt)


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
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
