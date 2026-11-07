"""
Cinematica del modelo humano de 18 articulaciones.

No depende de ROS: solo numpy. Asi se puede probar con pytest y reutilizar
en los scripts de validacion.

Convenciones
------------
* Marco "world": origen en el piso, justo debajo de la camara.
  x = hacia adelante (alejandose de la camara), y = izquierda, z = arriba.
* Marco del cuerpo (pelvis): x = hacia donde mira la persona, y = su
  izquierda, z = arriba. Con todos los angulos en cero la persona esta de pie,
  brazos colgando y piernas rectas.
* s = +1 para el lado izquierdo, -1 para el derecho.

Articulaciones (18):
  torso_flex, neck_flex                         -> eje y   (+ = hacia adelante)
  {l,r}_shoulder_flex                           -> eje -y  (+ = brazo adelante)
  {l,r}_shoulder_abd                            -> eje s*x (+ = brazo afuera)
  {l,r}_shoulder_rot                            -> eje z del brazo
  {l,r}_elbow_flex                              -> eje -y  (+ = doblar)
  {l,r}_hip_flex                                -> eje -y  (+ = muslo adelante)
  {l,r}_hip_abd                                 -> eje s*x (+ = pierna afuera)
  {l,r}_hip_rot                                 -> eje z del muslo
  {l,r}_knee_flex                               -> eje y   (+ = doblar)
Ademas hay 4 articulaciones "virtuales" que colocan al cuerpo en el piso:
  base_x, base_y, base_z (prismaticas) y base_yaw (giro sobre z).
"""

import math
import os

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# Puntos que publica pose_node, en este orden, en /human/keypoints.
# (nombre, indice de MediaPipe Pose)
KEYPOINTS = [
    ('nose', 0),
    ('l_ear', 7), ('r_ear', 8),
    ('l_shoulder', 11), ('r_shoulder', 12),
    ('l_elbow', 13), ('r_elbow', 14),
    ('l_wrist', 15), ('r_wrist', 16),
    ('l_hip', 23), ('r_hip', 24),
    ('l_knee', 25), ('r_knee', 26),
    ('l_ankle', 27), ('r_ankle', 28),
]
KEYPOINT_NAMES = [n for n, _ in KEYPOINTS]

BASE_JOINTS = ['base_x', 'base_y', 'base_z', 'base_yaw']
BODY_JOINTS = [
    'torso_flex', 'neck_flex',
    'l_shoulder_flex', 'l_shoulder_abd', 'l_shoulder_rot', 'l_elbow_flex',
    'r_shoulder_flex', 'r_shoulder_abd', 'r_shoulder_rot', 'r_elbow_flex',
    'l_hip_flex', 'l_hip_abd', 'l_hip_rot', 'l_knee_flex',
    'r_hip_flex', 'r_hip_abd', 'r_hip_rot', 'r_knee_flex',
]
ALL_JOINTS = BASE_JOINTS + BODY_JOINTS
SIDES = (('l', 1.0), ('r', -1.0))

DEFAULT_BODY = {
    'hip_half_width': 0.09,
    'torso_length': 0.50,
    'shoulder_half_width': 0.18,
    'neck_length': 0.20,
    'upper_arm_length': 0.30,
    'forearm_length': 0.27,
    'thigh_length': 0.43,
    'shank_length': 0.42,
    'head_radius': 0.11,
}

DEFAULT_LIMITS = {
    'torso_flex': (-0.6, 1.6),
    'neck_flex': (-0.9, 0.9),
    'shoulder_flex': (-1.2, 3.14),
    'shoulder_abd': (-0.8, 1.6),
    'shoulder_rot': (-3.14, 3.14),
    'elbow_flex': (0.0, 2.7),
    'hip_flex': (-0.6, 2.4),
    'hip_abd': (-0.6, 1.2),
    'hip_rot': (-1.2, 1.2),
    'knee_flex': (0.0, 2.6),
}


def load_body_config(path=None):
    """Lee medidas y limites de body.yaml. Si no hay archivo, usa los de arriba."""
    body = dict(DEFAULT_BODY)
    limits = dict(DEFAULT_LIMITS)
    if path and os.path.exists(path) and yaml is not None:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        body.update(data.get('body', {}))
        limits.update({k: tuple(v) for k, v in data.get('limits', {}).items()})
    return body, limits


def joint_limits(joint, limits):
    """Limite de una articulacion por su nombre completo (p. ej. l_elbow_flex)."""
    key = joint[2:] if joint[:2] in ('l_', 'r_') else joint
    return limits.get(key)


# ---------- rotaciones basicas ----------

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


DOWN = np.array([0.0, 0.0, -1.0])
UP = np.array([0.0, 0.0, 1.0])


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else None


def _closest_angle(a, ref):
    """Mismo angulo que a, +/- 2*pi, lo mas cercano posible a ref (evita saltos)."""
    if ref is None:
        return a
    return a + 2.0 * math.pi * round((ref - a) / (2.0 * math.pi))


def camera_to_world(height, pitch):
    """
    Rotacion R y traslacion t tales que p_world = R @ p_optico + t.

    Marco optico de la camara: x derecha, y abajo, z hacia adelante.
    La camara esta a `height` metros del piso, inclinada `pitch` rad hacia abajo.
    """
    # optico -> camara estilo ROS (x adelante, y izquierda, z arriba)
    opt_to_body = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])
    c, s = math.cos(pitch), math.sin(pitch)
    pitch_rot = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return pitch_rot @ opt_to_body, np.array([0.0, 0.0, height])


# ---------- cinematica directa ----------

def forward_kinematics(q, body=None):
    """
    Calcula la posicion 3D (marco world) de cada punto a partir de los angulos.

    Devuelve los puntos de KEYPOINT_NAMES mas 'head' (centro de la cabeza) y 'pelvis'.
    q es un dict con los nombres de ALL_JOINTS (los que falten valen 0).
    """
    b = dict(DEFAULT_BODY, **(body or {}))
    g = lambda n: float(q.get(n, 0.0))  # noqa: E731

    pts = {}
    pelvis = np.array([g('base_x'), g('base_y'), g('base_z')])
    r0 = rot_z(g('base_yaw'))
    pts['pelvis'] = pelvis

    # Torso, cuello y cabeza
    rt = r0 @ rot_y(g('torso_flex'))
    sh_mid = pelvis + rt @ np.array([0, 0, b['torso_length']])
    rn = rt @ rot_y(g('neck_flex'))
    head = sh_mid + rn @ np.array([0, 0, b['neck_length']])
    pts['head'] = head
    pts['l_ear'] = head + rn @ np.array([0, 0.07, 0])
    pts['r_ear'] = head + rn @ np.array([0, -0.07, 0])
    pts['nose'] = head + rn @ np.array([0.10, 0, -0.02])

    for side, s in SIDES:
        # Brazo
        sh = sh_mid + rt @ np.array([0, s * b['shoulder_half_width'], 0])
        ra = (rt @ rot_y(-g(f'{side}_shoulder_flex'))
              @ rot_x(s * g(f'{side}_shoulder_abd'))
              @ rot_z(g(f'{side}_shoulder_rot')))
        el = sh + ra @ np.array([0, 0, -b['upper_arm_length']])
        rf = ra @ rot_y(-g(f'{side}_elbow_flex'))
        wr = el + rf @ np.array([0, 0, -b['forearm_length']])
        pts[f'{side}_shoulder'], pts[f'{side}_elbow'], pts[f'{side}_wrist'] = sh, el, wr

        # Pierna
        hp = pelvis + r0 @ np.array([0, s * b['hip_half_width'], 0])
        rh = (r0 @ rot_y(-g(f'{side}_hip_flex'))
              @ rot_x(s * g(f'{side}_hip_abd'))
              @ rot_z(g(f'{side}_hip_rot')))
        kn = hp + rh @ np.array([0, 0, -b['thigh_length']])
        rk = rh @ rot_y(g(f'{side}_knee_flex'))
        an = kn + rk @ np.array([0, 0, -b['shank_length']])
        pts[f'{side}_hip'], pts[f'{side}_knee'], pts[f'{side}_ankle'] = hp, kn, an

    return pts


# ---------- cinematica inversa (puntos -> angulos) ----------

class ShadowSolver:
    """
    Convierte los puntos 3D de la persona en los 22 valores de articulacion.

    Son 4 de base + 18 del cuerpo.

    Guarda la ultima solucion: si una parte del cuerpo no se ve, sus
    articulaciones conservan el valor anterior, y los giros que no se pueden
    observar (p. ej. rotacion del brazo con el codo estirado) tampoco saltan.
    """

    # Debajo de este angulo de codo/rodilla (rad) no se puede saber la rotacion
    # del brazo/muslo, asi que se conserva la anterior.
    MIN_BEND_FOR_ROT = 0.35

    def __init__(self, body=None, limits=None):
        self.body = dict(DEFAULT_BODY, **(body or {}))
        self.limits = dict(DEFAULT_LIMITS, **(limits or {}))
        self.q = dict.fromkeys(ALL_JOINTS, 0.0)
        self.has_base = False

    def _set(self, name, value, unwrap=False):
        if value is None or not math.isfinite(value):
            return
        if unwrap:
            value = _closest_angle(value, self.q.get(name))
        lim = joint_limits(name, self.limits)
        if lim is not None:
            value = min(max(value, lim[0]), lim[1])
        self.q[name] = float(value)

    def solve(self, kp):
        """
        Resuelve los angulos de un cuadro.

        kp: dict nombre -> np.array(3) en el marco world (solo los puntos vistos).
        Devuelve un dict con los valores de ALL_JOINTS.
        """
        P = lambda n: kp.get(n)  # noqa: E731

        # 1) Base: centro y orientacion de la cadera
        lh, rh = P('l_hip'), P('r_hip')
        if lh is not None and rh is not None:
            pelvis = (lh + rh) / 2.0
            left = lh - rh
            left[2] = 0.0
            left = _unit(left)
            if left is not None:
                fwd = np.cross(left, UP)
                self._set('base_x', pelvis[0])
                self._set('base_y', pelvis[1])
                self._set('base_z', pelvis[2])
                self._set('base_yaw', math.atan2(fwd[1], fwd[0]), unwrap=True)
                self.has_base = True
        if not self.has_base:
            return dict(self.q)

        pelvis = np.array([self.q['base_x'], self.q['base_y'], self.q['base_z']])
        r0 = rot_z(self.q['base_yaw'])

        # 2) Torso
        ls, rs = P('l_shoulder'), P('r_shoulder')
        sh_mid = None
        if ls is not None and rs is not None:
            sh_mid = (ls + rs) / 2.0
            v = r0.T @ (sh_mid - pelvis)
            self._set('torso_flex', math.atan2(v[0], v[2]))
        rt = r0 @ rot_y(self.q['torso_flex'])

        # 3) Cuello: centro de la cabeza = punto medio de las orejas (o la nariz)
        if sh_mid is not None:
            le, re = P('l_ear'), P('r_ear')
            head = (le + re) / 2.0 if le is not None and re is not None else P('nose')
            if head is not None:
                v = rt.T @ (head - sh_mid)
                self._set('neck_flex', math.atan2(v[0], v[2]))

        # 4) Brazos (relativos al torso) y 5) piernas (relativas a la pelvis)
        for side, s in SIDES:
            self._limb(side, s, rt, P(f'{side}_shoulder'), P(f'{side}_elbow'),
                       P(f'{side}_wrist'), 'shoulder', 'elbow', knee=False)
            self._limb(side, s, r0, P(f'{side}_hip'), P(f'{side}_knee'),
                       P(f'{side}_ankle'), 'hip', 'knee', knee=True)

        return dict(self.q)

    def _limb(self, side, s, r_parent, a, b, c, root, mid, knee):
        """Resuelve hombro+codo o cadera+rodilla a partir de 3 puntos a-b-c."""
        if a is None or b is None:
            return
        u = _unit(r_parent.T @ (b - a))
        if u is None:
            return
        flex, abd, rot = f'{side}_{root}_flex', f'{side}_{root}_abd', f'{side}_{root}_rot'
        bend = f'{side}_{mid}_flex'

        # u = Ry(-flex) Rx(s*abd) (0,0,-1) = (cos(abd) sin(flex), s sin(abd), -cos(abd) cos(flex))
        self._set(abd, math.asin(float(np.clip(s * u[1], -1.0, 1.0))))
        self._set(flex, math.atan2(u[0], -u[2]), unwrap=True)

        if c is None:
            return
        r1 = r_parent @ rot_y(-self.q[flex]) @ rot_x(s * self.q[abd])
        f = _unit(r1.T @ (c - b))
        if f is None:
            return
        bend_angle = math.acos(float(np.clip(-f[2], -1.0, 1.0)))
        self._set(bend, bend_angle)
        if bend_angle > self.MIN_BEND_FOR_ROT:
            if knee:
                # rodilla dobla hacia atras: f = Rz(rot) (-sin k, 0, -cos k)
                r = math.atan2(-f[1], -f[0])
            else:
                # codo dobla hacia adelante: f = Rz(rot) (sin e, 0, -cos e)
                r = math.atan2(f[1], f[0])
            self._set(rot, r, unwrap=True)
