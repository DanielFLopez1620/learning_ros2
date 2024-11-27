# Docker guide

This is a markdown file intended for learning and practicing about Docker, it is based on the Docker Coobook of Coachrane, Jeeva S. Chelladhurai and K Kahre.

# Introduction

Why Docker? Well... let's start explaining how applications are deployed.

First, applications were deployed directly on phsycal hardwared over Operating Systems (OS). It was stable, hardware-centric and with a long maintenance cycle, but it came with less flexibilty for developers and the resources were underutilized most of the time. 

Second, virtualization was invented (KVM, XEN, ESX, HyperV) where the focus was to use better the system resources . The mission here was to emulate the hardware for Virtual Machines (VMs) which could have their own guest OS, then they allowed to isolate applications, but this came with the cost of complexity and redundancy.

After that, the focus was changed to application-centric IT, where the hypervisor layer was removed in order to reduce complexity. Also, the term container appeared with OpenVZ, Solaris Zones and LXC. A container is less flexible and less secure than a VM (they do not allow to run Linux on a Windows Host and if they crashes the host system may get vulnerable).


Finally, this changed when Docker arrived and got implemented in big companies like Google, Microsoft, Red Hat, IBM and others. Docker appeared in March 2013 under the Apache 2.0 license, its mission was to develop a standard way to manage containers. In this case, it uses the OS underlying kernel features for containerization in order to guarantee platform independence and specificity by working with control groups, namespaces, capability layers and relateds.

![arquitecture_it](/ma01_docker_and_ros/resources/arquitectures_it.png)

Before moving on to the installation, let's make clear some keywords and names:

- **Namespace:** Building blocks of a container, they isolate applications from others.

- **PID Namespaces:** Allow each container to have its own process numbering (even with an own hierachy), which allow to have the smae program multiple times in different isolated enviornments.

- **Net namespace:** If we have many PID namespaces, how do we now which port should they be listening to? This is the work of the net namespace, to have different network interfaces on each container (even with its own routing table and firewall rules).

- **IPC namespace:** Refers to inter-process communication, which provides semaphores, message queue and shared memory segments for the containers.

- **mnt namespace:** Allow for own sets of mounted filesystem and root directories.

- **UTS namespace:** For hostname resolution in different containers.

- **User namespace:** Allow mapping of users and groups IDs per namespaces.

- **Cgroups:** Refers to Control groups which are in charge of resource limitation, prioritization, control and accounting for containers.

- **Union Filesystem:** Allow for files and directories of separate filesystems (in layers) to be transparently overlaid to create a new virtual filesystem.

- **Container format:** Combination of the namespaces, control groups and UnionFS into a wrapper called a container format.

- **OCI:** Which comes from Open Container Initiative, it is a lightweight, open-governance structure that is intended to create open industry standards around container formats and runtimes, which have two formats: The Runtime Specification and the Image Specification.

- **Images:** Read-only templates that allow to produces containers during runtime, which can be created from multiple lyers.

- **Registries:** Holds Docker images, which can be public or private, so you can access and download/upload images. The public registry for Docker is called *Docker Hub*.

- **Index:** Manages user accounts, permission, searches, tagging and so on on the public web interface of a Docker registry.

- **Container:** They run images and contain everything that is required to run a certain application.

- **Repository:** A collection of images tracked by a control system (GUIDs).

# Installation

As this repository is intended to work with Debian/Ubuntu for ROS 2 usage, we will focus on the installation with only these systems. For the official installation, I encourage you to check the [Docker Documentation Installation for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

Make sure you have 64-bit architecture and you have a kernel aboute 3.8 on Debian systems, you can check this with:

~~~bash
uname -i # Architecture
uname -r # Kernel
~~~

Also, you muss have an appropiate storage backend (by default it is the *device-mapper* in Ubuntu) and support for *cgroups* and *namespaces*, check it with the following commands:

~~~bash
grep device-mapper /proc/devices # Storage backend for Ubuntu
grep -i namespaces /boot/config-6.9.3-76060903-generic # Check namespaces for my kernel version
grep -i cgroups /boot/config-6.9.3-76060903-generic # Check cgroups for my kernel version
~~~

We will show the commands for installation as November 2024, so keep in mind this can change for future Docker releases:

1. Uninstall previous Docker packages that may get conflicts:

~~~bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
~~~

2. Add Docker repositories for apt installation

~~~bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
~~~

3. Install with apt:

~~~bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
~~~

4. Create the docker group and add user:

~~~bash
sudo groupadd docker
sudo usermod -aG docker $USER
~~~

5. Test Docker with a Hello World:

~~~bash
docker run hello-world
~~~

If you get errors, I encourage you to check the [post-installation step](https://docs.docker.com/engine/install/linux-postinstall/) for Linux. 

Also, you can use the [Get Docker Script](https://get.docker.com/) for the Docker Engine installation for systems like CentOS, Fedora, Debian, Ubuntu and Raspbian. You can do it with:

~~~bash
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
~~~

We will start soon to learn more about Docker, in case you have questions, a good start can be to check the Docker manual:

~~~bash
man docker
man docker-ps
~~~

And of course, do not forget to check the Docker Documentation for the [CLI references](https://docs.docker.com/reference/cli/docker/), with this said, let's begin our Docker journey.

# Docker containers and CLI tools:

Docker objective is to run containers, so our first steps are going to be related in how to start, stop, list, delete... them, which will be related with applications like CI/CD (Continious Integration / Continious Deployment), PaaS (Platform as a Service) and others.

- **Cheking version:** Here you do not use --version, prefer to use:

~~~bash
docker version
~~~

- **Listing and searching images:** As mentioned earlier, the images come from [Docker Hub](https://hub.docker.com/) mainly, then you can search for them only. However, you can run a search in the terminal. Keep in mind that when listing the images, the convention may be ```<user>/<name>```

~~~bash
# docker search [options] <keyword>
docker search --limit 10 ros
~~~

![docker_search_ros](/ma01_docker_and_ros/resources/docker_search_ros.png)

~~~bash
# If you need help, you can use --help in all the docker commands.
docker search --help
~~~

- **Pulling an image:** If you found an image that may be useful for you, then the next step is to bring it locally. This process (like in git) is called *pull*. Just make sure you are connected to internet through the Docker Client.

~~~bash
# docker pull [options] NAME[:TAG|@DIGEST]
docker pull ros:latest
~~~

- **Listing images:** To print what images you have locally installed or that were created manually.

~~~bash
# docker images [options] <name>:<tag>
docker images
~~~

- **Running images:** With the image present in your system, you can use them to start containers and see what happens with the logging. This process will merge the layers, allocate a unique ID to the container, allocate a filesystem (mounting a read/write lyer for the container), allocate a bridge network and assign a IP.

~~~bash
# docker run [options] <image> [command] [args...]
# Options:
#     -i : Interactive (STDIN mode)
#     -t : Allocate pseudo-tty and attaches it to the standard input
docker container run -i -t --name con_ubuntu ubuntu /bin/bash
~~~

![docker_run_ubuntu](/ma01_docker_and_ros/resources/docker_run_ubuntu.png)

~~~bash
# To exit of a contair use Ctrl + D or type exit
$ exit

# To deatch the container you can press Ctrl + P + Q
~~~


