# Preparation before running the startup script

## Install Docker with Compose and Buildx plugins
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

### Add your user to docker group
```
sudo usermod -aG docker $USER
```

### Reboot system
```
reboot
```

### (optional) Also you can install Portainer CE (Docker GUI) for ease of Docker use
https://docs.portainer.io/start/install-ce/server/docker/linux


# (optional) Next chapters are optional (use if you want to utilize GPU in container - needed for simulation performance) 

## Nvidia video card (if your case) {#nvidia-video-card}

### Install Nvidia driver (if not installed before)
```
sudo apt install nvidia-driver-535
```

#### Disable other video drivers
```
echo "blacklist nouveau" | sudo tee --append /etc/modprobe.d/blacklist.conf
echo "blacklist intel" | sudo tee --append /etc/modprobe.d/blacklist.conf
```

##### (optional) Enable other video drivers (only if want to enable them for some reason but they will occupy OpenGL)
```
sudo sed -i '/blacklist nouveau/d' /etc/modprobe.d/blacklist.conf
sudo sed -i '/blacklist intel/d' /etc/modprobe.d/blacklist.conf
```

#### Reboot system
```
reboot
```

### Set Nvidia as default video card
```
sudo prime-select nvidia
```

#### Check Nvidia used for OpenGL
```
glxinfo | grep OpenGL # must return text with Nvidia words (not Mesa)
```
If you see Mesa (instead of Nvidia) one of reason is you did not blacklist some video driver that used insted of Nvidia for OpenGL. See what used with OpenGL and blacklist it. Also check "prime-select query" (must be nvidia).

## Install nvidia-container-toolkit
[nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)

### Restart docker
```
sudo systemctl daemon-reload
sudo systemctl restart docker
```

### Troubleshooting
#
## Error - "OpenGL context creation failed" or "Bad Value" or any other graphical problem.
Follow instruction about [Nvidia video card](#nvidia-video-card) OR just install video driver in container same as in host (not recomended).
```
sudo apt update && sudo apt install nvidia-driver-<replace_with_driver_version> # f.e. 535.
```
## "Requesting list of world names"
Press "Ctrl + c" and restart ROS2 launcher.
## After run Gazebo you dont see model at field
Check you have binded collisions.

