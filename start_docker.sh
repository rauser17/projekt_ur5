#!/bin/bash
IMAGE_NAME="ros2_projekt_ur5"

echo "=== 1. Konfiguracja GUI (X11) ==="

echo "=== 2. Budowanie Obrazu Docker ==="
docker build -t $IMAGE_NAME .

echo "=== 3. Uruchamianie Projektu ==="
docker run -it --rm \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    $IMAGE_NAME \
    /bin/bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch sterowanie_kamera ur5_projekt.launch.py"
