# Robot description:

Here we are going to explore the aspects needed to generate a robot description that can be used for visualization and simulation. We will cover topics related with **tf2**, **urdf**, **sdf**, and other related.

# TF2

A transform....


Some general dependencies you will need are related with the proper *tf2* packages, you can run the command:

    sudo apt-get install ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim

Now, let's move to the practice part, we will begin with Python, this practice is heavily based on the code provided by the package **turtle-tf2-py**, you can check the original info by installing the next package (do not forget to source the package):

    sudo apt-get install ros2-humble-turtle-tf2-py

## TF2 with Python:

    ros2 pkg create --build-type ament_python m03_tf_with_py

