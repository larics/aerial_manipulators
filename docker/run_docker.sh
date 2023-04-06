#!/bin/bash

CONTAINER_NAME=$1
[ -z "$CONTAINER_NAME" ] && CONTAINER_NAME=uam_ros_cont

IMAGE_NAME=$2
[ -z "$IMAGE_NAME" ] && IMAGE_NAME=uam_ros_img:latest

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run \
  -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --volume $HOME/bags:/root/bags \
  --volume $HOME/rosbag_default:/root/rosbag_default \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env display=$display \
  --env TERM=xterm-256color \
  --name $CONTAINER_NAME \
  $IMAGE_NAME \
  /bin/bash
