#!/bin/bash

echo ''
echo "Run container\`s shell. You can exit by command - 'exit' or run commands in container (see README.md)."
docker exec -it $ros_container_name bash
