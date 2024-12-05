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

- **Listing images:** To print what images you have locally installed or that were created manually. A expanded note can be found on [docker image ls | Docker Docs](https://docs.docker.com/engine/reference/commandline/image_ls/)

~~~bash
# docker image ls [options] <name>:<tag>
# docker images [options] <name>:<tag>
docker image list
~~~

- **Running images:** With the image present in your system, you can use them to start containers and see what happens with the logging. This process will merge the layers, allocate a unique ID to the container, allocate a filesystem (mounting a read/write lyer for the container), allocate a bridge network and assign a IP. For info on the flags and options, check the [docker container run | Docker Docs](https://docs.docker.com/reference/cli/docker/container/run/).

~~~bash
# docker container run [options] <image> [command] [args...]
# docker run [options] <image> [command] [args...]
# Options:
#     -i : Interactive (STDIN mode)
#     -t : Allocate pseudo-tty and attaches it to the standard input
#     -rm : Remove container after it exits
docker container run -i -t --rm --name con_ubuntu ubuntu /bin/bash
~~~

![docker_run_ubuntu](/ma01_docker_and_ros/resources/docker_run_ubuntu.png)

~~~bash
# To exit of a contair use Ctrl + D or type exit
$ exit

# To deatch the container you can press Ctrl + P + Q
~~~

**NOTE:** A container can be referred by three ways: Name, short ID or container ID.

- **Listing containers:** To check running/available containers. You can check more of the command listed below on [docker container ls | Docker Docs](https://docs.docker.com/reference/cli/docker/container/ls/)

~~~bash
# docker container ls [options]
# docker ps [options]
# Options:
#     -a : List running and stopped coniners
#     -q : Obtain only containers ID
#     -l : Last created
docker container ls -a
~~~

- **Logging containers:** To look at the logs, you can specify the logs of a certain the container (STDOUT/STDERR).Check more info on [docker container log | Docker Docs](https://docs.docker.com/engine/reference/commandline/container_logs/). You can use the commando shown below or navigate to the file **```/var/lib/docker/containers/<container_id>/...```**

~~~bash
# docker container log [options] <container>
docker container log con_ubuntu
~~~

- **Stopping a container:** At this point you may be wondering how to stop a container you accidentally exit, check the  next command for this, and do not forget to learn more about it in [docker container stop | Docker Docs](https://docs.docker.com/engine/reference/commandline/container_stop/).

~~~bash
# docker container stop [options] <container> <containers...>
# docker stop [options] <container> <containers...>
docker container stop con_ubuntu # Make sure it is running
~~~

- **Stopping all the containers:** For those moments of desesperations, you can use the following command:

~~~bash
docker container stop $(docker container list -q)
~~~

- **Delete containers:** Once you stopped a container, thre may be a possibilty yo do not need it more, then you can use the *rm* alike from Docker Cli, more info on [docker container rm | Docker Guides](https://docs.docker.com/engine/reference/commandline/container_rm/). Keep in mind that the container have to be stopped before deleting.

~~~bash
# docker container rm [options] <container> 
# docker rm [options] <container>
# Options:
#       -f : To forcefully remove a container without stopping it first
docker container rm con_ubuntu
~~~

- **Forcing deletion of containers:** If you encounter that you need to stop a container before deleting it, and you have to do it for all the containers in your machine, you can run:

~~~bash
docker container stop $(docker container ls -q)
docker container rm $(docker container ls -aq)
~~~

- **Removing stopped containers:** You may have heard of it, it what we call a prune. And yes, Docker does have it... for more info do not forget to check about [docker container prune | Docker docs](https://docs.docker.com/engine/reference/commandline/container_prune/)

~~~bash
# docker container prune [options]
# Options:
#       -f : force prune
docker container prune
~~~

- **Restart policies:** It aims for automation during Docker management in case it fails. You can set policies on *restart*, so it does something when it reboots/fails. If you want to learn more about policies and restarting containers, you can consider the next resoureces: [Running Containers | Docker Docs](https://docs.docker.com/engine/containers/run/#restart-policies-restart) & [Start containers automatically](https://docs.docker.com/engine/containers/start-containers-automatically/).

~~~bash
# docker container run --restart=<policy> [OPTIONS] <image>:<tags> [Command] [Args...]
# Restart options:
#       --restart=no : If it dies, it does nothing.
#       --restart=on-failure : It restarts if the exit returned a nonzero value
#       --restart=always : Always restart no matter the error code
#       --restart=on-failure:2 : It restarts two times if it receives a nonzero value when exitting.
docker container run --restart=always -d -i -i --name con_ubuntu_2 ubuntu /bin/bash
~~~

- **Getting privileged access in containers:** You may be familiar with superusers (sudo and related commands), for example, when doing installations from *apt* in Ubuntu. However, by default, Dockers start with limited capabilities and you may need privileged access to be able to generate some actions. Of course, this relates with security issues that you will have to consider. Start by checking the [runtime privilege and linux capabilities | Docker Docs](https://docs.docker.com/engine/containers/run/#runtime-privilege-and-linux-capabilities).

~~~bash
# docker container run --privileged [options] <image> [commands] [args...]
docker container run --privileged -i -t --name con_ubuntu_3 ubuntu /bin/bash

# docker container run --cap-drop=<consideration> [options] <image> [command] [args...]
#     --cap-drop=CHOWN : Prevent usage of chown
docker container run --cap-drop=CHOWN -i -t --name con_ubuntu_4 ubuntu /bin/bash
~~~

- **Accessing host device inside a container:** 

Docker allow the option to run a container with access to the device (host), for this process the flag ```--device``` exists.  Just make sure to provide the access to the desired path and avoid to give permission to confidential information or important part of your disks.

~~~bash
# docker container run --device=<HostDevice>:<DeviceMapping>:<Permissions> [options] <image> [command] [args...]
docker container run --device=/dev/sdc:/dev/xvdc -i -t --name con_ubuntu_5 ubuntu /bin/bash
~~~

- **Injecting a new process into a running container:**
Curiosity about the container calls, and you may need to see inside it. In this cases, you can use ```exec``` for including process in your running containers. More info on [docker container exec](https://docs.docker.com/engine/reference/commandline/container_exec/)

~~~bash
# docker exec [options] <container> <command> [args...]
docker container exec -it con_ubuntu_4  /bin/bash
~~~

- **Reading container metadata:**
This process is called inspection, and as you may suppose, the command for this is ```inspect```. For more info, go and check [docker container inspect](https://docs.docker.com/engine/reference/commandline/container_inspect/).

~~~bash
#docker container inspect [options] <container> [containers....]
# Options:
#     --f='{{.NetworkSettings.IPAdress}}' : Get ip of container
docker container inspect con_ubuntu_5
~~~

- **Labeling and filtering containers:**
When you have a lot of images and containers, adding a label may be useful to keep track of them easily, as they can be used for filtering/selection purposes. It is only made by adding the option ```--label``` when running the container. This can be added with inspections too.

~~~bash
# docker contianer run --label <label> [options] <image> [command]
docker run --label ubuntu.current=jellyfish --name con_ubuntu_6 ubuntu

# Then filter with label and check
docker containeer ls -a --filter label=ubuntu.current=jellyfish
~~~

- **Reaping a zombie inside a container:**
This is not about an apocalypsis, rather when a process states in a transient state whie it is dead but the entry in the process table is kept until the parent process reads this and exits it. This removal process is called *reaping*. For this, you may have heard of ```systemd``` and ```init```. In Docker, this also happens, and you have to take attention to them. Consider the next example:

~~~bash
# docker container run --init [options] <images> [command] [args...]
docker container run --name con_ubuntu_7 --rm --init ubuntu pstree -p
~~~

# Working with Docker Images:

We have explored about containers, however we will change the focus on the Docker images as they are the essentila building blocks of the containerization paradigm (and are the base for creating containers). And if you do not find a image with the requierements you are searching, you can create your own.

### Image based on a container:

When we start a container, it mounts a read/write layer which is destroyed if we do not save it. Then, you can use ```docker container commit``` to save the layer and create a new image based on a running or stopped container. The command is shown below:

~~~bash
# docker container commit [options] <container> [repository[:tag]]
docker container commit --author "Dan" --message "mod-ubuntu" con_ubuntu_5 committed_ubuntu
~~~

You can even track changes (like when using Git) for the images considered. You can do it with the ```docker container diff``` command:

~~~bash
# docker container diff
docker diff commited_ubuntu
~~~

For more information you can check:

- [docker container commit | Docker Docs](https://docs.docker.com/engine/reference/commandline/container_commit/)

### Logging for Docker Image Registry

There would be times you will publish your images to the public or share them in a private way, for both cases you will need a Docker registry (if you do not want to depend on sending all the files each time some friend calls your for a copy). To push an image to a public repository, you must log in to a Docker registry and you must have the ownership of the repostitory. In this cases we use ```docker login``` and ```docker logout```:

~~~bash
docker login
# It should ask for your username (DockerID) and password, or sent you to a web page to validate your credentials.

docker logout
# If you want to sign out
~~~

You can use your Docker ID from [DockerHub](https://hub.docker.com/) or use a registry from [Gitlab](https://about.gitlab.com/)

For more information, you can check:

- [docker container login | Docker Docs](https://docs.docker.com/engine/reference/commandline/login/)
- [docker container logout | Docker Docs](https://docs.docker.com/engine/reference/commandline/logout/)
- [Docker credential helpers | Github](https://github.com/docker/docker-credential-helpers)

### Publishing an image to a registry:

When you have an image, you can push (like in Github) to DockerHub (or other Docker image registry service) with the command ```docker image push```.

~~~bash
# docker image push [options] <name>[:<tag>]
# docker push [options] <name>[:<tag>]
~~~

After this, you can go to your account and check the changes, where you should find your new image pushed. Also, you can try to publish your images to a locally hosted registry if you provide the URL (with the proper port) to it.

You can find more information on:

- [docker image push | Docker Docs](https://docs.docker.com/engine/reference/commandline/image_push/)

### Reviewing the history of an image:

The command ```docker image history``` is an analogy to the ```git log``` and ```git diff``` commands, so let's check it out:

~~~bash
# docker image history [options] <image>
docker image history ubuntu
~~~

If you are searching for commits message, you can use our old friend ```docker image inspect``` with filters to achieve it:

~~~bash
docker image inspect --format='{{.Comment}} ubuntu_con_5
~~~

For more information you can check:

- [docker image history | Docker Docs](https://docs.docker.com/engine/reference/commandline/image_history/)

## Removing an image:

You may know about ```rm``` command and his dangerous ```rm -rf```... Well, Docker also have options to remove images when providing the short ID, long ID, image digests or image name (along with tag or just the latest will be deleted), with the command ```docker image rm``` and it also have a force option with the flag ```-f```.

~~~bash
# docker image rm [options] <image> [images...]
docker image rm ubuntu
~~~

Be careful when using ```-f``` because if you delete a image with a container spawned, you will end up with dangling images. If you get in the need of deleting every container and image, please stop them first and you can try:

~~~bash
docker container stop $(docker container ls -q)
docker container rm $(docker container ls -a -q)
docker image rm $(docker image ls -q)
~~~

For more information, you can check:

- [docker image rm | Docker Docs](https://docs.docker.com/engine/reference/commandline/image_rm/)