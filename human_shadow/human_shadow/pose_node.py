"""
Nodo de ROS 2 (Humble) para el human digital shadow.

Lee la imagen a color y la profundidad de la Orbbec Astra Pro,
detecta el cuerpo con MediaPipe (Pose Landmarker) y publica:

* /human/keypoints (geometry_msgs/PoseArray): los puntos 3D del cuerpo en el
  marco "world" (piso), en el orden de kinematics.KEYPOINT_NAMES. Un punto que
  no se ve llega con posicion NaN. El stamp es el de la imagen, para medir latencia.
* /human_skeleton (visualization_msgs/MarkerArray): un MANIQUI 3D
  (cilindros, esferas y cabeza) que se mueve como la persona, para RViz2.

Marco "world": origen en el piso debajo de la camara, x hacia adelante,
y a la izquierda, z hacia arriba. Se define con camera_height y camera_pitch_deg.
"""

import math
import os
import time

from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from human_shadow.kinematics import camera_to_world, KEYPOINTS
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# Puntos del cuerpo que usamos: indice de MediaPipe Pose -> nombre
JOINTS = {idx: name for name, idx in KEYPOINTS}

# Extremidades del maniqui: (punto A, punto B, radio en metros)
LIMBS = [
    (11, 13, 0.045), (13, 15, 0.038),   # brazo izquierdo
    (12, 14, 0.045), (14, 16, 0.038),   # brazo derecho
    (23, 25, 0.065), (25, 27, 0.050),   # pierna izquierda
    (24, 26, 0.065), (26, 28, 0.050),   # pierna derecha
    (11, 12, 0.050),                    # hombros
    (23, 24, 0.070),                    # cadera
]

# Tamano de la esfera en cada articulacion (diametro en metros)
JOINT_SIZE = {
    11: 0.11, 12: 0.11,     # hombros
    13: 0.08, 14: 0.08,     # codos
    15: 0.07, 16: 0.07,     # munecas / manos
    23: 0.12, 24: 0.12,     # caderas
    25: 0.11, 26: 0.11,     # rodillas
    27: 0.09, 28: 0.09,     # tobillos / pies
}

BODY_COLOR = ColorRGBA(r=0.92, g=0.92, b=0.92, a=1.0)   # blanco tipo maniqui
JOINT_COLOR = ColorRGBA(r=0.80, g=0.80, b=0.82, a=1.0)  # gris claro


def quat_from_z_to(v):
    """Cuaternion que gira el eje Z para que apunte en la direccion v."""
    n = np.linalg.norm(v)
    if n < 1e-6:
        return Quaternion(w=1.0)
    v = v / n
    dot = float(np.clip(v[2], -1.0, 1.0))      # z . v
    if dot > 0.999999:
        return Quaternion(w=1.0)
    if dot < -0.999999:
        return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    axis = np.array([-v[1], v[0], 0.0])        # z x v
    axis /= np.linalg.norm(axis)
    ang = math.acos(dot)
    s = math.sin(ang / 2.0)
    return Quaternion(x=axis[0] * s, y=axis[1] * s, z=axis[2] * s,
                      w=math.cos(ang / 2.0))


class HumanPoseNode(Node):

    def __init__(self):
        super().__init__('human_pose_node')

        # Parametros (se pueden cambiar con --ros-args -p nombre:=valor)
        self.declare_parameter(
            'model_path', os.path.expanduser('~/models/pose_landmarker_full.task'))
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        self.declare_parameter('min_visibility', 0.3)
        # Suavizado de la deteccion: 0 = sin suavizar, 0.9 = muy suave (con retraso)
        self.declare_parameter('smoothing', 0.3)
        # Que tan rapido el maniqui "persigue" la ultima deteccion en cada redibujado
        # (0.1 = lento y muy suave, 1.0 = salta directo)
        self.declare_parameter('follow_speed', 0.35)
        # La camara mide la superficie del cuerpo; empujamos el maniqui
        # unos cm hacia atras para que quede "dentro" de la persona.
        self.declare_parameter('body_offset', 0.06)
        # Segundos que el maniqui "recuerda" una parte del cuerpo que dejo de ver
        self.declare_parameter('hold_time', 1.5)
        # Datos de la lente (se usan si la camara los manda en cero / sin calibrar).
        # Valores aproximados de la camara a color de la Astra Pro a 640x480.
        self.declare_parameter('fx', 554.0)
        self.declare_parameter('fy', 554.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        # Donde esta la camara respecto al piso (marco "world").
        # Mide la altura del lente al piso con el flexometro; pitch > 0 = mira hacia abajo.
        self.declare_parameter('camera_height', 1.0)
        self.declare_parameter('camera_pitch_deg', 0.0)
        self.declare_parameter('world_frame', 'world')
        # Veces por segundo que se redibuja el maniqui
        self.declare_parameter('publish_rate', 30.0)

        model_path = self.get_parameter('model_path').value
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'No encuentro el modelo de MediaPipe en {model_path}. '
                'Descargalo primero (ver instrucciones).')
            raise FileNotFoundError(model_path)

        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
        )
        self.landmarker = vision.PoseLandmarker.create_from_options(options)

        self.bridge = CvBridge()
        self.depth = None          # ultima imagen de profundidad
        self.K = None              # parametros de la camara (fx, fy, cx, cy)
        self.frame_id = None       # se fija al recibir camera_info
        self.world_frame = self.get_parameter('world_frame').value
        self.R, self.t = camera_to_world(
            self.get_parameter('camera_height').value,
            math.radians(self.get_parameter('camera_pitch_deg').value))
        self.last_ts = 0
        self.min_vis = self.get_parameter('min_visibility').value
        self.alpha = self.get_parameter('smoothing').value
        self.offset = self.get_parameter('body_offset').value
        self.smoothed = {}         # posiciones suavizadas de cada articulacion
        self.warned_info = False
        self.last_seen = {}        # ultima vez (s) que se vio cada articulacion
        self.hold = self.get_parameter('hold_time').value
        self.follow = self.get_parameter('follow_speed').value
        self.display = {}          # posiciones que se dibujan (se mueven suave)
        self.frames = 0
        self.detections = 0
        self.detect_time = 0.0

        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self.color_cb, qos_profile_sensor_data)

        self.pub = self.create_publisher(MarkerArray, 'human_skeleton', 10)
        self.kp_pub = self.create_publisher(PoseArray, 'human/keypoints', 10)
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.timer_cb)
        self.create_timer(5.0, self.report_cb)
        self.get_logger().info('Nodo de pose listo. Esperando imagenes de la camara...')

    # ---------- callbacks ----------

    def info_cb(self, msg: CameraInfo):
        k = msg.k
        if k[0] > 0 and k[4] > 0:
            self.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy
        else:
            # Camara sin calibrar: usamos valores tipicos
            if not self.warned_info:
                self.get_logger().warn(
                    'La camara manda datos de lente en cero; uso valores por defecto '
                    '(parametros fx, fy, cx, cy).')
                self.warned_info = True
            self.K = (self.get_parameter('fx').value, self.get_parameter('fy').value,
                      self.get_parameter('cx').value, self.get_parameter('cy').value)
        self.frame_id = self.world_frame

    def depth_cb(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_cb(self, msg: Image):
        if self.depth is None or self.K is None:
            return

        rgb = np.ascontiguousarray(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # MediaPipe en modo VIDEO necesita tiempos (ms) siempre crecientes
        ts = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6)
        if ts <= self.last_ts:
            ts = self.last_ts + 1
        self.last_ts = ts

        self.frames += 1
        t0 = time.monotonic()
        result = self.landmarker.detect_for_video(mp_image, ts)
        self.detect_time += time.monotonic() - t0
        if not result.pose_landmarks:
            return
        self.detections += 1

        landmarks = result.pose_landmarks[0]
        h, w = rgb.shape[:2]
        dh, dw = self.depth.shape[:2]
        fx, fy, cx, cy = self.K

        # 1) Puntos que MediaPipe ve en la imagen, con su profundidad (si la hay)
        seen = {}
        for idx in JOINTS:
            lm = landmarks[idx]
            if lm.visibility is not None and lm.visibility < self.min_vis:
                continue
            u, v = lm.x * w, lm.y * h          # pixel en la imagen a color
            if not (0 <= u < w and 0 <= v < h):
                continue
            seen[idx] = (u, v, self.depth_at(u * dw / w, v * dh / h))
        if not seen:
            return

        # 2) Profundidad "del cuerpo": mediana de los puntos con profundidad valida.
        #    Sirve para rellenar puntos sin profundidad y descartar los que
        #    cayeron en la pared o el fondo.
        zs = [z for (_, _, z) in seen.values() if z is not None]
        if not zs:
            zs = [p[2] - self.offset for p in self.smoothed.values()]
        if not zs:
            return
        body_z = float(np.median(zs))

        now = time.monotonic()
        for idx, (u, v, z) in seen.items():
            if z is None or abs(z - body_z) > 0.5:
                # Sin profundidad o fuera del cuerpo: usamos la anterior o la del cuerpo
                prev = self.smoothed.get(idx)
                z = (prev[2] - self.offset) if prev is not None else body_z
            z += self.offset
            # Convertir pixel + profundidad a coordenadas 3D (metros)
            p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
            if not np.all(np.isfinite(p)):
                continue
            # Suavizado para que el maniqui no tiemble
            if idx in self.smoothed and now - self.last_seen.get(idx, 0) < self.hold:
                p = self.alpha * self.smoothed[idx] + (1 - self.alpha) * p
            self.smoothed[idx] = p
            self.last_seen[idx] = now

        self.publish_keypoints(msg.header.stamp, now)

    def timer_cb(self):
        """Redibuja el maniqui a ritmo fijo con las ultimas posiciones conocidas."""
        if self.frame_id is None:
            return
        now = time.monotonic()
        pts = {}
        for i, target in self.smoothed.items():
            if now - self.last_seen.get(i, 0) >= self.hold:
                self.display.pop(i, None)
                continue
            cur = self.display.get(i)
            if cur is None:
                cur = target.copy()
            else:
                # Avanza un poco hacia la ultima deteccion: movimiento fluido
                cur = cur + self.follow * (target - cur)
            self.display[i] = cur
            pts[i] = self.to_world(cur)
        self.publish_mannequin(pts)

    def report_cb(self):
        if self.frames:
            self.get_logger().info(
                f'Procesadas: {self.frames / 5.0:.1f} img/s | '
                f'MediaPipe tarda {1000.0 * self.detect_time / self.frames:.0f} ms por imagen | '
                f'persona detectada en {100.0 * self.detections / self.frames:.0f}%')
        else:
            self.get_logger().warn('No llegan imagenes de la camara.')
        self.frames = 0
        self.detections = 0
        self.detect_time = 0.0

    # ---------- utilidades ----------

    def to_world(self, p):
        """Punto del marco optico de la camara al marco world (piso)."""
        return self.R @ p + self.t

    def publish_keypoints(self, stamp, now):
        """Publica los puntos 3D (marco world). Los que no se ven van en NaN."""
        out = PoseArray()
        out.header.stamp = stamp
        out.header.frame_id = self.world_frame
        nan = float('nan')
        for _, idx in KEYPOINTS:
            pose = Pose()
            pose.orientation.w = 1.0
            p = self.smoothed.get(idx)
            if p is not None and now - self.last_seen.get(idx, 0) < self.hold:
                w = self.to_world(p)
                pose.position = Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
            else:
                pose.position = Point(x=nan, y=nan, z=nan)
            out.poses.append(pose)
        self.kp_pub.publish(out)

    def depth_at(self, u, v):
        """Profundidad en metros alrededor de un pixel (mediana de un cuadrito)."""
        ui, vi = int(u), int(v)
        patch = self.depth[max(vi - 3, 0):vi + 4, max(ui - 3, 0):ui + 4].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        if self.depth.dtype == np.uint16:   # la Astra da milimetros
            z /= 1000.0
        if z < 0.2 or z > 6.0:
            return None
        return z

    def new_marker(self, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.frame_id   # sin hora: RViz usa la posicion mas reciente
        m.ns = 'maniqui'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.color = color
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=0, nanosec=500_000_000)
        return m

    def cylinder(self, mid, a, b, radius, color=BODY_COLOR):
        """Cilindro que va del punto a al punto b."""
        d = b - a
        length = float(np.linalg.norm(d))
        if length < 0.02:
            return None
        m = self.new_marker(mid, Marker.CYLINDER, color)
        mid_pt = (a + b) / 2.0
        m.pose.position = Point(x=mid_pt[0], y=mid_pt[1], z=mid_pt[2])
        m.pose.orientation = quat_from_z_to(d)
        m.scale.x = m.scale.y = radius * 2.0
        m.scale.z = length
        return m

    def sphere(self, mid, p, diameter, color=JOINT_COLOR, sx=None, sy=None, sz=None):
        m = self.new_marker(mid, Marker.SPHERE, color)
        m.pose.position = Point(x=p[0], y=p[1], z=p[2])
        m.scale.x = sx or diameter
        m.scale.y = sy or diameter
        m.scale.z = sz or diameter
        return m

    def publish_mannequin(self, pts):
        markers = []
        mid = 0

        # Extremidades
        for a, b, r in LIMBS:
            mid += 1
            if a in pts and b in pts:
                m = self.cylinder(mid, pts[a], pts[b], r)
                if m:
                    markers.append(m)

        # Articulaciones
        for idx, size in JOINT_SIZE.items():
            mid += 1
            if idx in pts:
                markers.append(self.sphere(mid, pts[idx], size))

        have_shoulders = 11 in pts and 12 in pts
        have_hips = 23 in pts and 24 in pts

        # Torso: cilindro grueso del centro de hombros al centro de cadera
        mid += 1
        if have_shoulders and have_hips:
            sh = (pts[11] + pts[12]) / 2.0
            hp = (pts[23] + pts[24]) / 2.0
            width = float(np.linalg.norm(pts[11] - pts[12]))
            m = self.cylinder(mid, sh, hp, max(width * 0.45, 0.10))
            if m:
                markers.append(m)

        # Cuello y cabeza
        mid += 1
        neck_id = mid
        mid += 1
        head_id = mid
        if have_shoulders:
            sh = (pts[11] + pts[12]) / 2.0
            width = float(np.linalg.norm(pts[11] - pts[12]))
            head_d = float(np.clip(width * 0.65, 0.17, 0.26))
            if 0 in pts:
                head = pts[0].copy()
            else:
                # Si no vemos la cara, ponemos la cabeza arriba de los hombros
                up = np.array([0.0, 0.0, 1.0])    # en el marco world, "arriba" es +Z
                if have_hips:
                    up = sh - (pts[23] + pts[24]) / 2.0
                    up /= max(np.linalg.norm(up), 1e-6)
                head = sh + up * head_d * 1.2
            neck = self.cylinder(neck_id, sh, sh + (head - sh) * 0.6, 0.035)
            if neck:
                markers.append(neck)
            markers.append(self.sphere(head_id, head, head_d, BODY_COLOR,
                                       sz=head_d * 1.0, sy=head_d * 1.2))
        elif 0 in pts:
            markers.append(self.sphere(head_id, pts[0], 0.22, BODY_COLOR))

        if markers:
            self.pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = HumanPoseNode()
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
