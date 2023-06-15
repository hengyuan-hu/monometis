#!/usr/bin/env bash
CONDA_PREFIX=${CONDA_PREFIX:-"$(dirname $(which conda))/../"} \
LD_LIBRARY_PATH=${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH} \
launch_gripper.py \
--config-path /home/iliad/hengyuan/launch_robot/conf \
--config-name gripper_launch.yaml
