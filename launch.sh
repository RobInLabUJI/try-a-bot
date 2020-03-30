#!/bin/sh

xhost +local:root

docker run --rm -it \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/home/jovyan/work:rw" \
    -p 1234:1234 \
    -p 8888:8888 \
    try-a-bot

xhost -local:root

