#!/bin/bash
set -e
echo "Starting Basic ROS2 entrypoint..." --
. $HOME/.profile --
echo "Finished ROS2 entrypoint." --
exec "$@"