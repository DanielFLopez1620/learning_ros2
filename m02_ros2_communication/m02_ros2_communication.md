# ROS2 Communications

In this module, you will learn more about packages and its usage for communication with the tools you have already watched in the previous module (**topics**, **services** and **actions**). This implementations can be made with the ROS Client Libraries for Python (**rclpy**) and for C++ (**rclcpp**). You will also learn about compilation of the packages, the usage of **colcon** and the basics of package configuration with **Python** and **CMake**.

## Colcon for your workspace:

Colcon is the tool oriented for compilation of your packages, which is an iteration of the catkin tools from ROS (1). If you made the complete installation, you must have access to *colcon*, but you can check it or install it (if missing) with:

    sudo apt install python3-colcon-common-extensions

The key with the usage of colcon is that you need to follow the conventions for the directories:

- **build:** Where temporary files are stored, the packages are represented with subdirectories. This folder appears after the first run of the colcon configuration.

- **install:** The directory where the packages are installed to, by default in seperate subdirectories.

- **log:** A directory you will need to see when sonmething goes wrong, it contains the logging information about colcon calls.

- **src:** The space of the package, where you develop packages for future executables, configurations...

If this is your first time, you will need to build everything in your ROS2 space, so you need to use the option *colcon build --symlink-install* which allows all the installed files to be changed. If you want to build a single package or a group of them, you may prefer the option *colcon build*.

As always, do not forget to source the **setup.bash** file, otherwise you won't be able to run the executables of any packages.

    source /opt/ros/humble/setup.bash

After this introduction to colcon, we need a *workspace*, the place where your robotics development and magic will come to life. So, the basic for creating a package is that you need a folder (with the name of your preference), and a nested *src* directory.

    mkdir -p ~/<your_workspace>/src

In the *src* directory you can clone packages from Github, Gitlab or you can aslo create your own packages. This packages would need some **dependencies** to be able to run/execute something, in these cases, you can run:

    cd ~/<your_workspace>
    rosdep install -i --from-path src --rosdistro humble -y

Each time you add/create a package you will need to build them and adds the executables to the list, for this you run:

    cd ~/<your_workspace>
    colcon build
    source install/local_setup.bash

## Packages are the key:

Once you have set up your workspace, you could have asked... it seems boring... and yes... you do not have something to execute there, or even something saved there. But what should be in the workspace? Well, to be more precise, what should be in the *src* directory of our workspace, the answer here is: **Packages**.

A package is a collection of directories and files that defines an organizatio unit for your ROS2 code. It also allows you to share it with others. The packages can work with two different aments, which are *CMake* or *Python*, and this implies different package structures.

On one hand, the case of **CMake**, the minimum required content refers to:

- **CMakeLists.txt**: Which describes how the code will be build (mostly related with C/C++).
- **include/<package_name>**: Is the directory that contains the headers of the package.
- **package.xml**: Meta description of the package.
- **src/**: Directory for source code.

On the other hand, we have **Python**, and the minimum structure refers to:

- **package.xml**: Meta description of the package.
- **resource/<package_name>**: Marker files for the package
- **setup.cfg**: Required for setting up executables.
- **setup.py**: File with instructions to install the package.
- **<package_name>/**: Containts the __init__.py for finding your package.

The packages in your workspace doesn't need to be of the same ament, and you can have as many as your computer and memory allows it.

Now, let's create one package:



# Troubleshooting:

- If you get a warn equal or related to: "SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn (...)". It means that the package 'setuptools' isn't in the proper version for ros2, you can resolve (according to [ros.answer](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)) with the next command (only ROS2 Humble):

    pip install setuptools==58.2.0