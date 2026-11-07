"""Pruebas de ida y vuelta: angulos -> puntos 3D -> angulos."""

import math
import os

from human_shadow.kinematics import (
    ALL_JOINTS, forward_kinematics, KEYPOINT_NAMES, load_body_config,
    ShadowSolver)
import numpy as np
import pytest

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'body.yaml')


def random_pose(rng):
    """Postura aleatoria lejos de las singularidades del calculo."""
    q = {
        'base_x': rng.uniform(1.0, 3.5), 'base_y': rng.uniform(-1.0, 1.0),
        'base_z': rng.uniform(0.8, 1.0), 'base_yaw': rng.uniform(-3.0, 3.0),
        'torso_flex': rng.uniform(-0.3, 1.0), 'neck_flex': rng.uniform(-0.5, 0.5),
    }
    for side in 'lr':
        q.update({
            f'{side}_shoulder_flex': rng.uniform(-0.8, 2.0),
            f'{side}_shoulder_abd': rng.uniform(-0.5, 1.2),
            f'{side}_shoulder_rot': rng.uniform(-1.5, 1.5),
            f'{side}_elbow_flex': rng.uniform(0.5, 2.4),
            f'{side}_hip_flex': rng.uniform(-0.4, 1.8),
            f'{side}_hip_abd': rng.uniform(-0.4, 0.9),
            f'{side}_hip_rot': rng.uniform(-0.9, 0.9),
            f'{side}_knee_flex': rng.uniform(0.5, 2.2),
        })
    return q


def keypoints(q, body):
    pts = forward_kinematics(q, body)
    return {n: pts[n] for n in KEYPOINT_NAMES}


@pytest.mark.parametrize('seed', range(200))
def test_round_trip(seed):
    body, limits = load_body_config(CONFIG)
    rng = np.random.default_rng(seed)
    q = random_pose(rng)
    solved = ShadowSolver(body, limits).solve(keypoints(q, body))
    for j in ALL_JOINTS:
        assert solved[j] == pytest.approx(q[j], abs=1e-6), j


def test_standing_is_zero():
    body, limits = load_body_config(CONFIG)
    q = {'base_x': 2.0, 'base_z': 0.9}
    solved = ShadowSolver(body, limits).solve(keypoints(q, body))
    for j in ALL_JOINTS:
        assert solved[j] == pytest.approx(q.get(j, 0.0), abs=1e-6), j


def test_missing_limb_keeps_previous():
    body, limits = load_body_config(CONFIG)
    rng = np.random.default_rng(1)
    q = random_pose(rng)
    solver = ShadowSolver(body, limits)
    solver.solve(keypoints(q, body))
    kp = keypoints(q, body)
    del kp['l_elbow'], kp['l_wrist']
    out = solver.solve(kp)
    assert out['l_elbow_flex'] == pytest.approx(q['l_elbow_flex'], abs=1e-6)


def test_straight_elbow_holds_rotation():
    body, limits = load_body_config(CONFIG)
    solver = ShadowSolver(body, limits)
    q = {'base_x': 2.0, 'base_z': 0.9, 'l_shoulder_rot': 0.8, 'l_elbow_flex': 1.5}
    solver.solve(keypoints(q, body))
    q['l_elbow_flex'] = 0.0          # codo estirado: la rotacion no se observa
    out = solver.solve(keypoints(q, body))
    assert out['l_shoulder_rot'] == pytest.approx(0.8, abs=1e-6)


def test_limits_are_applied():
    body, limits = load_body_config(CONFIG)
    q = {'base_x': 2.0, 'base_z': 0.9, 'torso_flex': 2.5}
    out = ShadowSolver(body, limits).solve(keypoints(q, body))
    assert out['torso_flex'] <= limits['torso_flex'][1] + 1e-9
    assert math.isfinite(out['neck_flex'])


def test_camera_to_world():
    from human_shadow.kinematics import camera_to_world
    # Camara a 1.2 m, horizontal: un punto 2 m al frente queda en x=2, z=1.2
    R, t = camera_to_world(1.2, 0.0)
    assert R @ np.array([0, 0, 2.0]) + t == pytest.approx([2.0, 0, 1.2])
    # Un punto a la derecha de la imagen (x optico +) queda a la derecha (y -)
    assert (R @ np.array([0.5, 0, 2.0]) + t)[1] == pytest.approx(-0.5)
    # Un punto abajo en la imagen (y optico +) queda mas bajo (z menor)
    assert (R @ np.array([0, 0.3, 2.0]) + t)[2] == pytest.approx(0.9)
    # Camara inclinada 30 grados hacia abajo: el eje optico baja
    R, t = camera_to_world(1.5, math.radians(30))
    p = R @ np.array([0, 0, 1.0]) + t
    assert p == pytest.approx([math.cos(math.radians(30)), 0, 1.5 - 0.5])
