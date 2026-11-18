# Human Digital Shadow

Sombra digital humana en tiempo real: una cámara RGB-D de bajo costo (Orbbec Astra Pro)
detecta a una persona con MediaPipe, ROS 2 Humble calcula los ángulos de sus articulaciones y
un modelo humano articulado de **18 articulaciones** en **Gazebo Fortress** reproduce su
posición y postura a 30 Hz, para interacción humano-robot.

```
Astra Pro ──► pose_node ──► /human/keypoints ──► shadow_node ──► /joint_states ──► RViz
 (color +     (MediaPipe +    (15 puntos 3D,       (ángulos)    └► /human/cmd/* ──► Gazebo
 profundidad)  profundidad)    marco del piso)
```

| Plataforma probada | Versión |
|---|---|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Gazebo | Fortress (ignition-gazebo 6, vía `ros_gz`) |
| Python | 3.10, MediaPipe 0.10.21, numpy < 2 |

## Instalación (Ubuntu 22.04)

```bash
git clone https://github.com/BetoBu/HumanDigitalShadow.git
cd HumanDigitalShadow
bash scripts/setup_ubuntu2204.sh
```

El script instala ROS 2 Humble (si no está), `ros_gz` (Gazebo Fortress), el driver
[`ros2_astra_camera`](https://github.com/orbbec/ros2_astra_camera) (con `libuvc` y reglas udev),
MediaPipe, descarga el modelo `pose_landmarker_full.task` a `~/models/` y compila el
workspace en `~/hds_ws`. Se puede correr otra vez sin problema.

- ¿Ya tienes ROS 2 Humble? `SKIP_ROS=1 bash scripts/setup_ubuntu2204.sh`
- ¿Solo quieres la demo sin cámara? `SKIP_CAMERA=1 bash scripts/setup_ubuntu2204.sh`
- Otra carpeta de workspace: `WS=~/mi_ws bash scripts/setup_ubuntu2204.sh`

Después de instalar, **desconecta y vuelve a conectar la cámara** (para que tomen efecto las
reglas udev) y abre una terminal nueva.

## Uso

### 1. Demo sin cámara (para comprobar que todo está bien)

```bash
ros2 launch human_shadow demo.launch.py
```

Una persona virtual camina en círculo, saluda y se agacha. Se ve en RViz y en Gazebo.
La demo también publica los ángulos reales en `/human/ground_truth`, para comparar
contra lo que calcula `shadow_node`.

### 2. Con la Astra Pro

1. Coloca la cámara en un tripié mirando hacia la zona de trabajo (unos 3 × 2 m, empezando
   como a 1 m de la cámara).
2. Mide con el flexómetro la **altura del lente al piso** y la **inclinación hacia abajo**
   (0° si está horizontal).
3. Lanza:

```bash
ros2 launch human_shadow shadow.launch.py camera_height:=1.10 camera_pitch_deg:=10
```

Si tienes una **Astra Pro Plus**, agrega `camera_launch:=astro_pro_plus.launch.xml`.

Opciones del launch:

| Argumento | Default | Qué hace |
|---|---|---|
| `source` | `camera` | `camera` = Astra Pro, `demo` = persona virtual |
| `gazebo` | `true` | Abrir Gazebo con el modelo |
| `headless` | `false` | Gazebo sin ventana (solo servidor) |
| `rviz` | `true` | Abrir RViz |
| `camera_height` | `1.0` | Altura del lente al piso (m) |
| `camera_pitch_deg` | `0.0` | Inclinación de la cámara hacia abajo (°) |
| `model_path` | `~/models/pose_landmarker_full.task` | Modelo de MediaPipe |
| `body_config` | `config/body.yaml` | Medidas del cuerpo y límites de articulaciones |

`pose_node` y `shadow_node` reportan cada 5 s las imágenes por segundo, el tiempo de
MediaPipe, el % de detección y la latencia imagen → ángulos.

## Marco de referencia

`world` está en el **piso, justo debajo de la cámara**: x hacia adelante (alejándose de la
cámara), y a la izquierda, z hacia arriba. Así las posiciones se pueden comparar directamente
con marcas de cinta en el piso.

## Modelo de 18 articulaciones

| Grupo | Articulaciones | Cantidad |
|---|---|---|
| Tronco | `torso_flex`, `neck_flex` | 2 |
| Hombros | `{l,r}_shoulder_flex`, `{l,r}_shoulder_abd`, `{l,r}_shoulder_rot` | 6 |
| Codos | `{l,r}_elbow_flex` | 2 |
| Caderas | `{l,r}_hip_flex`, `{l,r}_hip_abd`, `{l,r}_hip_rot` | 6 |
| Rodillas | `{l,r}_knee_flex` | 2 |
| **Total** | | **18** |

Además hay 4 articulaciones virtuales (`base_x`, `base_y`, `base_z`, `base_yaw`) que colocan
al cuerpo en el piso. Las medidas y límites están en
[`human_shadow/config/body.yaml`](human_shadow/config/body.yaml); los usan tanto el URDF como el
cálculo de ángulos, así que siempre coinciden. Los ejes y sentidos de cada ángulo están
documentados en [`kinematics.py`](human_shadow/human_shadow/kinematics.py).

## Tópicos

| Tópico | Tipo | Quién lo publica |
|---|---|---|
| `/camera/color/image_raw`, `/camera/depth/image_raw` | `sensor_msgs/Image` | driver de la Astra (profundidad alineada al color) |
| `/human/keypoints` | `geometry_msgs/PoseArray` | `pose_node` / `demo_node`: 15 puntos 3D en `world`, orden de `KEYPOINT_NAMES`, NaN = no visto |
| `/human_skeleton` | `visualization_msgs/MarkerArray` | `pose_node`: maniquí para RViz |
| `/joint_states` | `sensor_msgs/JointState` | `shadow_node`: 22 valores (4 base + 18 cuerpo) |
| `/human/cmd/<articulación>` | `std_msgs/Float64` | `shadow_node` → `ros_gz_bridge` → Gazebo |
| `/human/gz_joint_states` | `sensor_msgs/JointState` | Gazebo: ángulos que alcanzó el modelo |
| `/human/ground_truth` | `sensor_msgs/JointState` | `demo_node`: ángulos reales de la persona virtual |

## Estructura

```
human_shadow/
├── human_shadow/
│   ├── pose_node.py      cámara + MediaPipe -> puntos 3D (marco del piso)
│   ├── shadow_node.py    puntos 3D -> ángulos -> RViz y Gazebo
│   ├── kinematics.py     cinemática directa/inversa (solo numpy, con pruebas)
│   └── demo_node.py      persona virtual para probar sin cámara
├── urdf/human.urdf.xacro modelo de 18 articulaciones + controladores de Gazebo
├── config/body.yaml      medidas y límites
├── worlds/shadow_world.sdf
├── launch/               shadow, demo y camera
├── rviz/shadow.rviz
└── test/                 pruebas de cinemática + linters de ROS
scripts/setup_ubuntu2204.sh
```

## Pruebas

```bash
cd ~/hds_ws
colcon test --packages-select human_shadow && colcon test-result --verbose
```

`test_kinematics.py` genera 200 posturas aleatorias, calcula los puntos 3D con la cinemática
directa y verifica que el cálculo de ángulos recupere exactamente los mismos ángulos.

## Problemas comunes

- **`No llegan imagenes de la camara`**: revisa `ros2 topic hz /camera/color/image_raw`. Si no
  hay nada, desconecta y conecta la cámara (reglas udev) o prueba `lsusb | grep 2bc5`.
- **El maniquí sale desfasado de la persona**: la profundidad no está alineada al color. El
  launch ya pide `depth_registration:=true`; verifica que el driver no muestre errores.
- **La sombra flota o está inclinada**: corrige `camera_height` y `camera_pitch_deg`.
- **Error de numpy al importar `cv_bridge`**: se instaló numpy 2. Corre
  `python3 -m pip install --user "numpy<2"`.

## Flujo de trabajo

Cada etapa se trabaja en su propia branch y entra a `main` por pull request.

## Licencia

Apache-2.0
