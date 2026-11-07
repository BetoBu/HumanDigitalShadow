#!/usr/bin/env bash
# Instala todo lo necesario para el Human Digital Shadow en Ubuntu 22.04:
#   ROS 2 Humble, Gazebo Fortress (ros_gz), driver de la Orbbec Astra Pro,
#   MediaPipe y el modelo de pose, y compila el workspace.
#
# Uso (desde la raiz del repo):
#   bash scripts/setup_ubuntu2204.sh
#
# Variables opcionales:
#   WS=~/hds_ws            carpeta del workspace de ROS 2
#   SKIP_ROS=1             no instalar ROS 2 (si ya lo tienes)
#   SKIP_CAMERA=1          no instalar el driver de la camara (solo demo)
#
# Se puede volver a correr sin problema: se salta lo que ya esta instalado.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${WS:-$HOME/hds_ws}"
MODEL_DIR="$HOME/models"
MODEL_URL="https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"

step() { echo -e "\n\033[1;34m==> $*\033[0m"; }

# ---------- 0. Verificaciones ----------
. /etc/os-release
if [[ "${VERSION_ID}" != "22.04" ]]; then
  echo "Este script es para Ubuntu 22.04 (tienes ${PRETTY_NAME})."; exit 1
fi

# ---------- 1. ROS 2 Humble ----------
if [[ -z "${SKIP_ROS:-}" && ! -f /opt/ros/humble/setup.bash ]]; then
  step "Instalando ROS 2 Humble"
  sudo apt update && sudo apt install -y locales curl software-properties-common
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
  # Importante en 22.04: actualizar systemd/udev antes de instalar ROS
  sudo apt upgrade -y
  sudo apt install -y ros-humble-desktop ros-dev-tools
else
  step "ROS 2 Humble ya instalado (o SKIP_ROS); sigo"
fi

# ---------- 2. Paquetes de ROS del proyecto (Gazebo Fortress incluido) ----------
step "Instalando ros_gz (Gazebo Fortress), xacro y herramientas"
sudo apt install -y \
  ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-rviz2 \
  python3-pip python3-yaml python3-rosdep git cmake build-essential

if ! [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# ---------- 3. Driver de la Orbbec Astra Pro ----------
mkdir -p "$WS/src"
if [[ -z "${SKIP_CAMERA:-}" ]]; then
  step "Dependencias del driver ros2_astra_camera"
  sudo apt install -y libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
    ros-humble-image-geometry ros-humble-camera-info-manager \
    ros-humble-image-transport ros-humble-image-publisher

  if ! pkg-config --exists libuvc; then
    step "Compilando libuvc"
    rm -rf /tmp/libuvc && git clone --depth 1 https://github.com/libuvc/libuvc.git /tmp/libuvc
    cmake -S /tmp/libuvc -B /tmp/libuvc/build -DCMAKE_BUILD_TYPE=Release
    cmake --build /tmp/libuvc/build -j"$(nproc)"
    sudo cmake --install /tmp/libuvc/build
    sudo ldconfig
  fi

  if [[ ! -d "$WS/src/ros2_astra_camera" ]]; then
    step "Descargando ros2_astra_camera"
    git clone --depth 1 https://github.com/orbbec/ros2_astra_camera.git "$WS/src/ros2_astra_camera"
  fi
  step "Instalando reglas udev de la camara (para usarla sin sudo)"
  sudo cp "$WS/src/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# ---------- 4. Nuestro paquete ----------
if [[ ! -e "$WS/src/human_shadow" ]]; then
  ln -s "$REPO_DIR/human_shadow" "$WS/src/human_shadow"
fi

# ---------- 5. MediaPipe y modelo de pose ----------
step "Instalando MediaPipe (versiones probadas con ROS 2 Humble)"
# numpy<2: cv_bridge de Humble esta compilado con numpy 1.x
python3 -m pip install --user -r "$REPO_DIR/requirements.txt"

mkdir -p "$MODEL_DIR"
if [[ ! -f "$MODEL_DIR/pose_landmarker_full.task" ]]; then
  step "Descargando modelo de MediaPipe Pose"
  curl -L -o "$MODEL_DIR/pose_landmarker_full.task" "$MODEL_URL"
fi

# ---------- 6. Compilar ----------
step "Compilando el workspace $WS"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if ! grep -qF "$WS/install/setup.bash" "$HOME/.bashrc"; then
  {
    echo ""
    echo "# Human Digital Shadow"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $WS/install/setup.bash"
  } >> "$HOME/.bashrc"
fi

step "Listo. Abre una terminal nueva y prueba:"
echo "  ros2 launch human_shadow demo.launch.py        # sin camara"
echo "  ros2 launch human_shadow shadow.launch.py      # con la Astra Pro"
