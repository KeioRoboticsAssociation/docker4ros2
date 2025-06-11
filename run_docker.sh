#!/bin/bash
# Launch a ROS2 Humble container with the current repo mounted.
docker run -it --rm \
  -v "$(pwd)":/workspace \
  -w /workspace \
  osrf/ros:humble \
  bash
