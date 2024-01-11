# ROS 2 Installation Steps:

All the installation methods can be found in the official site of [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html). Keep in mind that for this tutorial (learning_ros2) we will use native Ubuntu 22.04. So prefer that installation for our purposes, the steps are also mentioned below. 

1. Make sure you have set locale:

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

2. Configure sources:

    sudo apt install software-properties-common 
    sudo add-apt-repository universe # Check ubuntu universe in enabled

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg # Get ROS2 GPG Key

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null # Add repository to source lists

3. Install ROS2 (Desktop recommended):

    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop

4. Set up the environment:

    source /opt/ros/humble/setup.bash # First time
    echo "/opt/ros/humble/setup.bash" >> ~./bashrc # Add configuration permanently

# Troubleshooting:

If you have any troubles, or find a solution for a problem, you can recommend it to be added here. In the future some of the most common ones will be added.