#!/usr/bin/env bash
export CONDA_PREFIX=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
# export LIBRARY_PATH=${CONDA_PREFIX}/lib
export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib
launch_robot.py \
--config-path ${PWD}/conf \
--config-name robot_launch.yaml \
timeout=15
