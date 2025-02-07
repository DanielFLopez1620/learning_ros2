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

## Working with CLI Tools:

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

### Removing an image:

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

### Exporting an image:

You can export images with tarballs to pass them to other people that may not have permission to use public images or images hosted in a Docker registry. For this exits the command ```docker image save```, so let's check it out:

~~~bash
# docker iamge save [-output=<tarfile>] <image> [images...]
docker image save --output=saved_ubuntu.tar con_ubuntu_6
~~~

You can export the container filesystem by providing the next command:

~~~bash
docker container export --output=saved_ubuntu.tar <container_id>
~~~

For more information you can find: 

- [docker image save | Docker Docs]()

### Importing an image:

A colleague passed you an tarball image, you aren't going to uncompress it with ```tar``` command (Here we do not do that). Rather we will use ```docker image import```, the command info is shown below:

~~~bash
# docker image import [options] <file/url> [repository:[tag]]
docker image import saved_ubuntu.tar ubuntu:imported
~~~

For more information you can check:

- [docker image import | Docker Docs](https://docs.docker.com/engine/reference/commandline/image_import/)

### Configuring a private registry

You do not need to have your Dockerfiles in Dockerhub if you think they are confidential or they aren't ready. Here we will create our own private registry too.

Let's build our example number 2 (more info in the section about Dockerfiles that you can find some lessons below) and run it:

~~~bash
# Make sure to be in the 02_ros_docker_example directory
docker image build -t ros_2_humble_ex .
docker container run -i -t -d -p 5000:5000 ros_2_humble_ex
~~~

The next step is to add a proper tag and push it to the local registry, which can be accessed at *localhost* or *127.0.0.1*.

~~~bash
docker tag ros_2_humble_ex localhost:5000/ros_2_humble_ex
~~~

Then, you can make the push to the registry, by using:

~~~bash
docker image push localhost:5000/ros_2_humble_ex
~~~

For more information, you can check on:

- [Docker Registry | Github](https://github.com/docker-archive/docker-registry)


### Automated builds with Github

Yeah, you can automate processes with Docker and containers, but let's start by configuring Docker with your Github Account.

TODO: Add automated builds

### Creating a custom base image

It is better to use the Docker tools to create a image that reflects your requirements. However, there would be cases were you may select to use a custom-build image, one of the options is to use ```debootstrap``` as it can create any Debian-based system. You can install it with:

~~~bash
sudo apt install debootstrap
~~~

For the demostration, we will create a Xenial Ubuntu (18.04) version. So, let's consider the next script:

~~~bash
# Dir for the container
mkdir xenial

# Install xenial
sudo deboostrap xenial ./xenial

# Check the installation
ls ./xenial

# Expor as an image
sudo tar -C xenial/ -c . docker image import - xenial

# Search for the xenial image
docker image ls
~~~

You can do this process with other debian images that are available in. For more information, check:

- [Debootstrap | Debian](https://wiki.debian.org/Debootstrap)

- [Base Images | Docker Docs](https://docs.docker.com/articles/baseimages/)

### Minimal image using scratch base image:

The image presented in the previous lesson is pretty big and , unless you are working with full capacities of the OS, it is not recommended as containers aims for modularization and installing just the required items for an application to run. For this you can select the binaries of a image or use Docker's reserved image (scratch image).

For this you need that the Docker daemon is running and it has access to the *gcc* and *scratch image*. Prepare the next files:

-  A *demo.c* C file:

~~~C
#include <stdio.h>

void main()
{
    printf("Hello World from a Docker!\n");
}
~~~

- A Dockerfile that interact with the *scratch* image:

~~~Dockerfile
FROM scratch
ADD hello_world /
CMD ["/hello_world"]
~~~

Then you will need to move to the proper folder (in this case the [04_docker_from_scratch](/ma01_docker_and_ros/docker_examples/04_docker_from_scratch/) example) and follow the next steps:

1, Build the executable using a gcc:7.2 runtime container:

~~~bash
docker container run --rm -v ${PWD}:/src -w /src gcc:7.2 gcc -static -o hello_world src/hello_world.c
~~~

2. Verify the binary creation

~~~bash
ls -al
# Check for a hello_world executable
file -b hello_world
~~~

3. Build the scratch image based on the Dockerfile:

~~~bash
docker image build -t hello_world_scratch .
~~~

4. Run the container and watch the message:

~~~bash
docker container run --rm hello_world_scratch
~~~

This is how you can manage Dockers without additional layers from the base image. Just keep in mind that you will need to add all the binaries or aspect required for the app to work from the *scratch.*

For more information you can check:

- [Scratch | Docker Hub](https://hub.docker.com/_/scratch/)
- [Base Images | Docker Docs](https://docs.docker.com/articles/baseimages/)

### Building images in multiple stages

As we did in the previous lesson, we had two processes one for compiling with *gcc* and the other for running the image. This can be replace with multistage build which allows to orchestrate complex building stages in a single Dockerfile with intermediate stages.

Let's consider the [05_docker_multistage](/ma01_docker_and_ros/docker_examples/05_docker_multistage/) which contian a C file and a Dockerfile with the next contents:

~~~C
#include <stdio.h>

void main()
{
    printf("Hello from a multistaged Docker\n");
}
~~~

~~~Dockerfile
# Stage 1
FROM gcc:7.2 AS compiler
COPY src /src
RUN gcc -static -o /src/hello_multistaged /src/hello_mutistaged.c && strip -R .comment -s /src/hello_multistaged

# Stage 2
FROM scratch
COPY --from=compiler /src/hello_multistaged .
CMD ["./hello_multistaged"]
~~~

As you may notice,each stage is divided by a From, and to consider other stages info, for example, during ```COPY``` instruction you hsoul dspecify the ```FROM``` origin of the file/biary/dir to consider.

For running the exmaple, make sure you are in the example dir, and run:

1. Build the image

~~~bash
docker image build -t hello_multistaged .
~~~

2. After a succesful build, run the container:

~~~bash
docker container run -rm hello_multistaged
~~~

For more information, you can check at Docker Hub: 

- [Scratch images | Docker Hub](https://hub.docker.com/_/scratch/)

### Visualizing hierarchy of images:

Sometimes you need a graph to understand what is going on, that is the reason **[Graphviz](https://graphviz.org/)** and **[nate/dockviz](https://hub.docker.com/r/nate/dockviz/)** appeared. Before making anything, confirm you have it installed:

~~~bash
sudo apt install graphviz
~~~

Then, you can use the **nate/dockviz** to visualize the hierarchy:

~~~bash
docker run -it --rm -v /var/run/docker.sock:/var/run/docker.sock nate/dockviz images -t
~~~

You should see somthing like the next phto (if you have run the previous two examples):

![docker_hierarchy](/ma01_docker_and_ros/resources/docker_hierarchy_view.png)

For more information, check:

- [DockerViz | Github](https://github.com/justone/dockviz)

## Considering newtwork and data management:

Some applications require to work from outside the docker, share data, communicate, and so on. So the focus will be to understand the network management of a Docker container and the data uses with Docker.

### Introduction to Docker network

When a Docker starts it creates a virtual Ethernet bridge with the name **docker0**, which can be checked using your net tools on the machine, for example:

~~~bash
ifconfig
~~~

But you can get direct info on it by considering:

~~~bash
ip addr show docker0
~~~

The IP and network data (domain and ranges) are chosen pseudo-randomly based on the RFC1018. This process allow the communication between containers and local machine (if set up properly). Then you can check the ip on a Docker with:

~~~bash
docker container run --rm -it ubuntu

# And then inside the container
apt update
apt install net-tools iproute2
ip a
ifconfig 
~~~

In the ```ip a``` results you should see the ```eth0@if<n>``` where the ```<n>``` is the interface index to identify the host. Also, you should be able to see that the ip is based on **172.17.0.0/16** which is the subdomain it is able to cover.


Another important thing to consider is the pair creation of the virtual ethernet (veth), where it ties one to the end of the **veth** to the Docker0 and the other pair to the new created eth0 interface. This host end of the **veth** interface usually receives a name based on ```veth + <hex_of_7_digits> + @ + <interface_id>```, which can be checked with ```ip addr``` too.

Now, let's experiment more with the Docker networks. This time taking advantage of the Linux Ethernet bridge administration command ```brctl```:

~~~bash
docker container run --rm -it --network=host ubuntu

# And then inside the container
apt update
apt install bridge-utils

# Check interfaces 
brctl show

~~~

This command should list the avaialble interfaces and the bridge present in your container. But also, as you can see next, it creates **iptables** with they proper rules in the Docker host:

~~~bash
# Run in your machine}
sudo iptables -t nat -L -n
~~~

Focus your attetion in the **POSTROUTING** section, where the rule is set up for data exchange on the **docker0** apparently, which also allows for connections with the external world.

You can experiment with **ping** and **traceroute** to check connectivity and redirections with a Docker container, for example, the adress 8.8.8.8.

For more info you can check:

- [RFC1928 | IETF](https://datatracker.ietf.org/doc/html/rfc1918)
- [Network | Docker Docs](https://docs.docker.com/network/)

### External access to the container:

Docker aims to enable microservice architecture in a very lightweight. This means that Docker is able to exchange data, but (by default) there is no path to connect with the external world and no info specified to be forwarded outside. However, there is a solution to this problem, that can be used with ```docker container run``` command.

The flag ```-p``` (or publish) allows to publish all exposed ports to random ports. Or you can specify a pair, for example, ```8080:8080```. But let's go through a practical view:

1. Create a container an expose the port 80

~~~bash
# Terminal 1
# docker container run -p <host_port>:<container_port>
docker container run -i -t -p 80:80 ubuntu
~~~

2. Review the port mapping with the command ```docker container port```

~~~bash
# Terminal 2
# docker container port <id>
docker container port <ubuntu_container_id>
~~~

![docker_container-ports](/ma01_docker_and_ros/resources/docker_container_ports.png)

3. According to the selected port you can try to configure actions and manage the usage of them. Keep in mind to use the ports properly, and consider security issues in these cases.

Remember that with ```iptables``` you can check the NAT rules for the Docker, here you should be able to watch the mapping of the ports:

~~~bash
sudo iptables -t nat -L -n
~~~

![iptables_docker_ports](/ma01_docker_and_ros/resources/docker_container_iptables_ports.png)

Before moving on, let's clarify additional aspects of the ```-p``` flag of the ```docker container run``` command, as it has other options additional to the ```<host_port>:<container-port>``` option we described previously:

- ```<container-port>```: This allow the Docker Engine to select the Docker host port which can be in the range of 32768 to 61000.

- ```<ip>:<host_port>:<container_port>``` : Add a particular IP interface of the Docker host.

- ```<ip>::<container_port>``` Allow Docker Engine selection of the port while providing a particual IP interface.

For more information you can check:

- [Networking | Docker Docs](https://docs.docker.com/engine/userguide/networking/)

- [Binding | Docker Docs](https://docs.docker.com/engine/userguide/networking/default_network/binding/)

### Attaching containers to a host network: 

To make this happen, you just need to add the argument ```--net=host``` when running a container, as will this attach the Docker host's network stack:

~~~bash
docker container run -it --rm --net=host ubuntu 

# Once inside the Docker
apt update
apt install iproute2
ip address

# Check and compare with the address of your machine
~~~

As shown next the machine (on the left) share network data with the container (on the right:)

![docker_net_host_comparison](/ma01_docker_and_ros/resources/docker_net_host_comp.png)

For more information you can check the ```docker run ``` command and network info on containers:

- [docker container run | Docker Docs](https://docs.docker.com/reference/cli/docker/container/run/)

- [Networking | Docker Docs](https://docs.docker.com/engine/userguide/networking/)

### Containers without networks

Let's start by listing the available network configurations on Docker:

~~~bash
docker network ls
~~~

![docker_network_ls](/ma01_docker_and_ros/resources/docker_network_ls.png)

In this case, our interest goes with ```none``` as this option only creates the loopback interface for the container and nothing more. This is useful when requiring isolation of the network.

Just make sure you do not really need a connection to the network, consider the next example:

~~~bash
docker container run -it --rm --net=none ubuntu

# If you try updating, you will not be able 
apt update
~~~

If you check the interface of a container with **none** net configuration, you shoul only see the loopback.

### IPs and multiple containers:

There would be cases when you need to share data between the containers or divide the processes/service to accomplish tasks, even this may require sharing the same IP adress.

For these cases, Docker allow the option to inherit the IP address to the services and other containers.

1. Run a Ubuntu container and prepare the network commands to check ip:

~~~bash
docker container run -itd --name=network_tester ubuntu
docker container exec network_tester apt update
docker container exec network_tester apt install -y iproute2
~~~

2. Run another ubuntu container that inherits from the first one, install *iproute2* to check network set up.

~~~bash
docker container run -itd --net container:network_tester --name=network_comp ubuntu
docker container exec network_comp apt update
docker container exec network_comp apt install -y iproute2
~~~

3. Compare networks between the containers:

~~~bash
# Terminal 1
docker container exec network_tester ip addr
# Terminal 2
docker container exec network_comp ip addr
~~~

![docker_network_comp](/ma01_docker_and_ros/resources/docker_network_comp.png)

As you can check, the containers share the *eth0* configuration. This trick (also known as a default bridge) is used when you want to orchestrate containers by sharing the IP adress, for example, with [Kubernetes](http://kubernetes.io/).


When using **exec** ensure thatthe container is running, otherwise you may not execute the command properly.

### User-defined bridge network

The bridge presented in the previous subtitle has a problem, it allow connections by using IP but not the containers' names.

As the IP is assigned on the container start up, it may present problems when you require orchestration. So Docker introduced capabilities related with user-defined netwokrs to solve these situations.

The features it includes are service discovery through an embedded DNS server, DNS-based load balancing, subnet configurations for the bridge and the option to manually assign IP adresses to the containers in the bridge subnet.

Let's present the step an develop an example:

1. Create a bridge using the ```docker network create```:

~~~bash
# docker network create <name>
docker network create custom_net
~~~

2. You can inspect the network, and take your attention to the *IP* and the subnet:

~~~bash
docker network inpect custom_net

~~~

![docker_inspect_custom_net](/ma01_docker_and_ros/resources/docker_inspect_custom_net.png)

3. Once you have identified the subnet, you can check the Docker host interface and **iptable**

~~~bash
ifconfig # Or 'ip addr' if you prefer so

sudo iptables -t nat -L -n
~~~

In the case of the PC where I try this, the results were:

![docker_network_ip_comm](/ma01_docker_and_ros/resources/docker_network_ip_comm.png)

In the **iptables** note that the NAT **Postrouting** rule has been added for the subnet.

If you want to use a certain IP when configuring your own network, you can use:

~~~bash
# docker network create <name> --subnet <ip>/<domain>
docker network create sixteen --subnet 16.16.1.1
~~~

### Discover and load balance in containers

You can generate various type of typologies with containers, even taking advantages of the network configurations by the default and user-defined bridges. Other useful tools for these purposes can be the service discovery throug an embedded DNS server and a DNS-based load balancing.

Let's say you have two Docker Containers, called *srv1* and *srv2*, that use the same network alias:

~~~bash
# docker container run [options] <image>
#    --network-alias : Helps to group multiple containers and load balance using the embedded DNS, which provides a round-robin load balancing.
#    --net : Allows to use a certain Docker network configuration (can be user-defined, must exists beforehand)
docker container run -itd --name srv1 --network-alias common_alias --net custom_net ubuntu
docker container run -itd --name srv2 --network-alias common_alias --net custom_net ubuntu
~~~

Then, you can proceed to checkout the IP addresses:

~~~bash
# Here we use a filter based on the custom user defined interface we made previously. Do not forget the '--format'

docker container inspect --format '{{ .NetworkSettings.Networks.custom_net.IPAddress }}' srv1

docker container inspect --format '{{ .NetworkSettings.Networks.custom_net.IPAddress }}' srv2
~~~

Which returns the next results in the case of the containers running in my PC:

![docker_load_balance_ips](/ma01_docker_and_ros/resources/docker_load_balance_ips.png)

Now, it is time to run another container, which is going to be transient and will allow us to understand the service discovery:

~~~bash
# In this case, we make the try with alpine and try to make ping to the given container by providing the name
docker container run --rm --net custom_net alpine ping -c1 srv1
~~~

![docker_load_balance_ip_ping](/ma01_docker_and_ros/resources/docker_load_balance_name_ping.png)

As you can see, it is possible to communicate only with the container's names, which can be used for orchestration. Also, you can even, ping the network alias you created previously, the result is that all the containers in this network configuration will interact with the ping call:

~~~bash
# Run this command multiple times
docker container run -rm --net custom_net ping -c1 common_alias
~~~

![docker_net_load_balance_alias](/ma01_docker_and_ros/resources/docker_net_load_balance_alias.png)

For more information, check:

- [docker netowrk commands | Docker Docs](https://docs.docker.com/reference/cli/docker/network/)
- [Dig Command in Linux | Geeks for Geeks](https://www.geeksforgeeks.org/dig-command-in-linux-with-examples/)


### Persisting data with volumes:

Until this point, we have mostly depend on a read-write layer that is temporary (once we remove the container it is destroyed), so you may be wondering how to persist data as you may need it beyond the life of a container. It can be achieved with volumes or bind mounts, which aims to persist data outside the container's filesystem.

For now, let's talk about volumes. They are a special directory in the Docker host that are created and managed by Docker, they can be anonymous (randomly generated) or named (specified).

It is simple to create a volume, you can run:

~~~bash
# docker volume create <name>
docker volume create backup
~~~

To check if it was created, you can list the available volumes:

~~~bash
# docker volume ls
docker volume ls
~~~

Now you can use the volume with a container, for the specification you will need to pair the volume with a dir in the container:

~~~bash
# docker container run -v <volume>:<pair_location> <image>
docker container run -it --rm -v backup:/for_backup ubuntu
~~~

Then, you can add files and explore the filesystem:

~~~bash
mkdir /for_backup/my_dir
echo "In the backup" > /for_backup/my_dir/saved.txt
~~~

Close the container, and run another, search for the file that we created, it should be right there.

~~~bash
# docker container run -v <volume>:<pair_location> <image>
docker container run -it --rm -v backup:/for_backup ubuntu

# From inside the container run
ls /for_backup
~~~

Keep in mind that the volumes (in the case of Unix system) tend to be located under the **/var/lib/docker/volumes/** (Try to use **tree** command to see your workflow or file flow in a better perspective). If you want to inspect a volume, you can use the command:

~~~bash
docker volume inspect backup
~~~

![docker_volume_inspect](/ma01_docker_and_ros/resources/docker_volume_inspect.png)

Notice that you can use a volume with multiple containers, therefore you can exchange data in a effective way between containers.

For more information, you can check:

- [Volumes | Docker Docs](https://docs.docker.com/engine/admin/volumes/volumes/)
- [Volume creation | Docker Docs](https://docs.docker.com/engine/reference/commandline/volume_create/)
- [Volume ls | Docker Docs](https://docs.docker.com/engine/reference/commandline/volume_ls/)
- [Volume inspect | Docker Docs](https://docs.docker.com/engine/reference/commandline/volume_inspect/)

### Sharing data between the machine and the containers:

Another option to persist data is to use bind mountings, which is to mount a docker host directory to a container and then share the data using a mount point.

To showcase this, you need a workspace you are require to share:

~~~bash
mkdir $HOME/to_share
echo "Share this text" > $HOME/to_share/sharing.txt
~~~

Now, run a container with a bind mount to the file, you can use a command to display the content of the file:

~~~bash
# docker container run -v [path_to_mount]:[pair_to_mount]
docker container run --rm -v $HOME/to_share:/info ubuntu cat /info/sharing.txt
~~~

![docker_bind_mount_ex](/ma01_docker_and_ros/resources/docker_bind_mount_ex.png)

If you only need a file, you can do it if you provide the proper path to it and a valid pair map to the container. Just keep in mind that they are passed in read-write mode, then to prevent modifications you can use the flag **ro** (read only), for example:

~~~bash
# docker container run -v [path_to_mount]:[pair_to_mount]
docker container run --rm -v $HOME/to_share/sharing.txt:/info/sharing.txt:ro ubuntu cat /info/sharing.txt
~~~

For more information, you can check:

- [Volumes | Docker Docs](https://docs.docker.com/engine/admin/volumes/volumes/)

## Working with a Dockerfile:

**Dockerfiles** are text-based build instruction that eneable the definition of the conent of the Docker image and automate the image creation. After the images are created with a **Dockerfile**, the are considered to be immutable.


### Building an image with a Dockerfile:

The format of a *Dockerfile* consist of a instruction with its arguments, where the instruction is generally in uppercase (but it is not case sensitive):

~~~
INSTRUCTION arguments
~~~

The instructions are evaluated in order, from top to botton, to form the layers of the container. Some of the instructions are:

- **FROM:** The first instruction as it is set for considering a base image, if the image is provided without tag. Then the *latest* is considered.

~~~Dockerfile
# Some options for FROM instruction
FROM <image>
FROM <image>:<tag>
FROM [registry_hostname[:port]/[user/](respository_name:version)]
~~~

- **RUN:** Refers to run a command in the shell or an executable.

~~~Dockerfile
# Like the sh -c option
RUN <command> [params...]

# Executable
RUN <exectuable> [params...]
~~~

- **LABEL:** Give a label to an image

- **CMD:** Provide a default executable while starting a container. But only CMD instruction will be honored/considered in a Dockerfile, so use it wisely.

~~~Dockerfile
CMD <executable> [params...]
CMD [params...]
CMD <command> [params...]
~~~

- **ENTRYPOINT:** Allow to configure the container as an executable, and can only be used once per Dockerfile.

~~~Dockerfile
ENTRYPOINT <executable> [params...]
ENTRYPOINT <command> [params...]
~~~

- **EXPOSE:** Exposes network port on the contaier.

~~~Dockerfile
EXPOSE <port> [<port>] [...]
~~~

- **ENV:** It habdles environment variables, by creting a ```key``` linked to a ```value```. It will persist while the container is running.

~~~Dockerfile
ENV <key> <value>
~~~

- **ADD:** Copies and then passes a file from a source (base OS) to the destintation (inside Docker container). The source can be a URL.

~~~Dockerfile
ADD <source> <destination>
ADD ["<source>" ... "<destination">] # If there are white spaces present
~~~

- **COPY:** Similar to ```ADD``` but focused only in files.

~~~Dockerfile
COPY ["<source>" ... "<destination>"]
~~~

- **VOLUME:** Creates a mount point with the given name and flag for mounting a external (outside container) volumen.

~~~Dockerfile
VOLUME ["<path/to/desired/volume>"]
~~~

- **USER:** Set a user for the instructions that are present below this one. So, do not forget to consider the proper permissions and workflow after this decision.

~~~Dockerfile
USER <username>/<UID>
~~~

- **WORKDIR:** Set a working directory for a ```RUN```, ```CMD``` or ```ENTRYPOINT``` command.

~~~Dockerfile
WORKDIR <path>
~~~

- **ONBUILD:** Adds trigger instructions to the image that will be executed later.

~~~Dockerfile
ONBUILD [instructions]
~~~

Now we are ready to give it a try...

First, let's create a directory to work:

~~~bash
mkdir docker_examples
cd docker_examples
~~~

Second, create a file and open it for writing:

~~~bash
mkdir 01_first_dockerfile
cd 01_first_dockerfile
code Dockerfile # If you are not using VS Code, use nano or vim
~~~

Third, let's add the content to the file:

~~~Dockerfile
# Base image
FROM ubuntu

# Adding personal info
LABEL maintainer="DanielFLopez1620"

# Add a command
CMD ["echo",  "Hello from inside the container"]
~~~

Fourth, you can build the image, just make sure you are in the directory of the Dockerfile.

~~~bash
docker image build .
~~~

Finally, you can add it to your repositories (remeber to be loggued in your current session) with:

~~~bash
docker image build -t 01_first_dockerfile_try .
~~~

For more information you can check:

- [Builder | Docker Docs](https://docs.docker.com/engine/reference/builder/)

### Usage case with ROS

We are in a repository to learn about robotics... so why do not give it a try with Docker...? It can be useful when you need to work with a different distro or you may need an isolated set up that is easily replicable for your robotic project... no matter the option, you can try to create your own Dockerfile for robotics.


For this, we will create a custom container for ROS 2 Humble, that can work with a brief ROS2 Comms example, in this case a simple floating numbers publisher:


~~~Dockerfile
# Use ROS 2 Humble base image
FROM ros:humble

# Set up the working directory in the container
WORKDIR /workspace

# Install dependencies for ROS 2
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS 2 workspace into the container
COPY ./ros_ex_ws /workspace

# Install the ROS 2 workspace dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Source the ROS 2 setup script and the workspace setup script
# Set the entrypoint to source the workspace and run the ROS 2 node
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run brief_rclcpp float_pub"]

# Expose necessary ports for ROS 2
EXPOSE 11311 

# Bash call
CMD ["/bin/bash"]
~~~

The example is present in the [02_ros_docker_example_dir](/ma01_docker_and_ros/docker_examples/02_ros_docker_example/), so make sure to be in that location and run:

~~~bash
docker image build -t ros_2_humble_ex .
docker container run -i -t ros_2_humble_ex
~~~

After you run it, you should see in the terminal log messages of random numbers being published. If you check in a terminal of your local PC, you can subscribe to the topic.

~~~bash
ros2 topic list # /num_flt64 must appear
ros2 topic echo /num_flt64
~~~

This example was simple but validates the idea that we can use ROS 2 with Docker and we can manage and connect them over the network.

# Docker considerations:

## Performance:

When using containers, we do not only aim for ease of use because performance is also another important aspect. Some of the factors that may be present are:

- **Volumes:** Depending on how are you managing your data, the write/read/copy processes may be different, for example, you should aboid using the primary/root filesystem to store data, rather you should use the facility to attach/mount external storage through volumes (*volume* and *volumes-from* options).

- **Storage drives:** They have different priorities if nothing is chose, so do not forget to give it a check, for instance, at the writing moment the priority is considered to be *overlay2* (Linux union file system that allows multiple layers to be combined, where only the last and new layer is writable, the rest is read-only), *fuse-overlayfs* (user-space filesystem instaed of using kernel space), *btrfs* (B tree file system, that integrates compression, snapshots and roolbacks for  highly scalable and fault-tolerant system), *zfs* (Zettabyte File System, similar to brtfs but more oriented to data integrity and advanced management features) and *vfs* (Virtual File System, abstraction lyer that allows the kernel to interface with different type of file systems).

- **--net=host:** Docker creates a bridge and associates IPs from it to the contaienrs as a default. However, when using this options it exposes the host networking stack to the contaier which can give a better performance to the defualt bridge (just do not forget to consider the ports constraints).

- **cgroups:** Which are exposed by the defaul execution driver, they can be used to fine-tune performance in the container for example, with **CPU shares** (*-c* or *--cpu-shares* flag options), **CPU sets** (*--cpuset* flag) and **memory limits** (*-m* flag).

- **Sysctl and ulimit settings:** The first one deals with the kernel and system parameters, and the second one deals with user-level resource limits. Then, some changes in these can affect the containers workflow, for example, the number of open files. They can be set up with the option **--ulimit** and the proper suboption (for example, **data=**, **nofile**, **nproc**, **core**). Additionally, you can modify the */etc/docker/daemon.json* file for easy modifications with containers.

For instance, to check the optimization and performance you have to implement benchmarks in order to run a similar workload on a different environment with different performance stats and parameters. 

### Benchmarking CPU performance:

You can compare two systems, one bare-metal (your machine) and then a container to chekcc the differences, for this we will use a Ubuntu container and compare our machine with it. Let's follow the next steps:

1. Install **sysbench** in your system.

~~~bash
sudo apt update
sudo apt install sysbench
~~~

2. Run the test in your machine (bare-metal), consider the next arguments: ```--threads``` to specify how many threads you want to use (you can set it according the number of CPU cores available), ```--test=cpu``` relates to the test specification and ```--cpu-max-prime=<n>``` set the maximum prime number to calculate (complexity of the workload).

~~~bash
sysbench --test=cpu --cpu-max-prime=20000 --threads=4 run
~~~

![cpu_benchmark_bare_metal](/ma01_docker_and_ros/resources/cpu_benchmark_bare_metal.png)

3. Set up the container to test, in this case we are running *Ubuntu 22.04* on bare metal, and we will run *Ubuntu 22.04* too on the container, adapt this to the version you want to compare.

~~~bash
docker run -it --rm ubuntu:22.04 /bin/bash
~~~

4. Install sysbench in the container (do not forget to also update the system):

~~~bash
# Inside the container
apt update
apt install sysbench
~~~

5. Run the same test inside the container:

~~~bash
sysbench --test=cpu --cpu-max-prime=20000 --threads=4 run
~~~

![cpu_benchmark_container](/ma01_docker_and_ros/resources/cpu_benchmark_container.png)

6. Compare the results, prepare actions to adjuts the resources according your conclussions and test again. Remember that you can allocate or limit resources to a container with the flags ```--cpus``` and ```--cpu-shares```.

### Benchmarking Disk Performance:

This can help you to understand the overhead and potential differences in disk I/O performance between two setups, in this case bare-metal (your OS in your machine) and containerized environments (your Docker container). Some of the aspects to consider are read/write speeds, latency and trhoughput. This can be achieved with **[FIO](https://github.com/axboe/fio)** tool (comes from Flexible I/O tester)

1. Let's begin with installing **FIO** in the machine:

~~~bash
sudo apt-get install fio
~~~

2. Then, run the *bare-metal* benchmark, here we are goint to perform a simple sequential read/write so we will consider the next arguments: ```--ioengine=sync``` for I/O engine to use (sync refers to basic block I/O), ```--rw=read``` stands for type of operation, ```--base=4k``` refers to the size block of each operation, ```--numbjobs=1``` relates the number of threads, ```--size=1G``` specifies the sieze of the data to be read or written, ```--runtime=5m``` set up the time of the test and ```--direct=1``` refers to a direct I/O bypassing the  filesystem cache.

~~~bash
fio --name=benchmark --ioengine=sync --rw=read --bs=4k --numjobs=1 --size=1G --runtime=5m --time_based --direct=1 --name=fio_benchmark
~~~


3. Start up the container, in this case we will use a Ubuntu 22.04 image to correlate the results.

~~~bash
docker run -it --rm ubuntu:22.04 /bin/bash
~~~

4. Inside the container, install **FIO**.

~~~bash
apt update
apt install fio
~~~

5. Run the same benchmark for the container, then compare the results:

~~~bash
fio --name=benchmark --ioengine=sync --rw=read --bs=4k --numjobs=1 --size=1G --runtime=5m --time_based --direct=1 --name=fio_benchmark
~~~

After this tests you may compare them, and later change the I/O patterns used to check also for differences. Your attention in the results should consider the **lops** (I/O operations per second, the higher it is better), the **bandhwidth** (bw, rate of data transfer, bit rate or throughput) and **latency** (time between kernel submission and signal of I/O completed).

The results obtained for bare-metal and container are shown (respectively) below:

![disk_performance_bare_metal](/ma01_docker_and_ros/resources/disk_benchmark_bare_metal.png)
![disk_performance_container](/ma01_docker_and_ros/resources/disk_benchmark_container.png)

### Benchmarking network performance:

There are also tools to measure netowrk performance, in this case we are going to use **iperf3** that is a common tool to test TCP, UDP bandwidth performance.

Consider the next steps to do it:

1. Install **iperf3** on your machine:

~~~bash
sudo apt update
sudo apt install iperf3
~~~

2. Perform the benchmark in bare-metal, the ```-s``` allows the server mode option:

~~~bash
iperf3 -s
~~~

3. Start your Docker contaier, we will be still using an Ubuntu image:

~~~bash
docker run -it --rm ubuntu:22.04 /bin/bash
~~~

4. Install **perf3** inside the container:

~~~bash
apt update
apt install iperf3
~~~

5. You can now make connection test between the machine and the container, we will use the ```-c``` option to act as the client:

~~~bash
# Inside the container
iperf3 -c <machine_ip>
~~~

The results are shown below:

![network_benchmark](/ma01_docker_and_ros/resources/network_benchmark_ex.png)

### Checking resource usage:

Docker provides the ```docker stats``` command to chekc the resource used by a container. run a container, consider its name or ID and execute:

~~~bash
# docker stats [option] <container> [containers]
docker container <your_docker_name>
~~~

In my case, when having a ubuntu container running, it displayed the next information:

![docker_resources_usage](/ma01_docker_and_ros/resources/docker_resource_usage.png)

For more information you can check:

- [Stats | Docker Docs](https://docs.docker.com/engine/reference/commandline/stats/)

### Monitoring:

There are options available to check and monitor container performance like [cAdvisor](https://github.com/google/cadvisor) and [Prometheus](https://prometheus.io/). For now, let's focus on **cAdvisor**.

It can be run using a Docker Container, like follows:

~~~bash
sudo docker container run \
    --volume=/:/rootfs:ro \
    --volume=/var/run:/var/run:rw \
    --volume=/sys:/sys:ro \
    --volume=/var/lib/docker/:/var/lib/docker:ro \
    --publish=8080:8080 \
    --detach=true \
    --name=cadvisor \
    google/cadvisor:latest
~~~

This may not work in all system for the next reason you should keep in mind:

- Access problems to the specified volumes.
- Firewall and protections on port 8080.
- Restrictions imposed by AppArmor/SELinux.

You can get more information by considering:

~~~bash
sudo docker logs cadvisor
~~~

Be careful with what do you diable in terms of secutity when trying to run this, as you may expose yourself to risks.

For more information check:

- [Run Metrics | Docker Docs](https://docs.docker.com/config/containers/runmetrics/)

## Security:

Even when it is just a container, you should consider it as a running service/process in the host system and put all the security measures inside of it as you may do in the host system.

Docker already uses namespaces for isolation, which are the processes, netowrk, mount, hostname, shared memory and user. However, there are other parts used who aren't isolated, for example, SELinux, Cgroups, Devices (**/dev** for *mem* or *sd*), Kernel Modules and filesystems like **/sys**, **/proc/sys**, **proc/sysrq-trigger**, **/proc/riq** and **proc/bus** (but this are mounted on read only). Then, you require some special measuraments:

- Search for official images (by docker, vendor or someone else recognized in the area) as they are the base of the building block system. For example, when using ```docker search``` check for the **OFFICIAL** column.

~~~bash
docker search alpine
# Review the official and not official options.
~~~

- Prefer to set up the Docker Deaemon access with TCp to secure a Docker remote API.

- Consider turning off the default inter-container communication over the network with ```--icc=false``` on the Docker host when it is possible.

- Set Cgroups resources restriction to prevent **DoS** (Denial of Services) attacks.

- Any device node pre-create on a image cannot interact (talk) with the kernel if the **nodev** options is present.

Other guidelines you can consider are:

- Do not run services as root and treat the root in the container with caution.

- Avoid using images that you can find with the ```-insecure-registry=[]``` option.

- Do not run random containers from images of doubtful precedence or from a Docker registry you do not know.

- Have your kernel and system up to date.

- Avoid using ```--priviledged``` and drop the container privileges quickly when developing.

- Configure **Mandatory Access Control** through SELinux or AppArmor.

- Collect and check logs, even four auditing, then do regular auditing.

- Run containers on hosts which are specially designed for this purpose.

- Prefer mounting devices with ```--device``` rather than ```--privileged```

- Prohibit **SUID** and **SGID** inside the container.

For more information, check the next resources:

- [Official Images | Docker Docs](https://github.com/docker-library/official-images)

- [Security | Docker Docs](https://docs.docker.com/engine/security/)

### Checking SELinux and AppArmor in your host:

If you do not have a configuration of these tools, someone connected to a Docker container running in your host may gain additional access and expose your security, for example, if **SELinux** is disabled and a user is created (also, added to the Docker group to run **sudo** commands) you may have the power to shutdown the host.

As we will set up some Docker Security practices, let's first make sure we have **SELinux** and **AppArmor** active in our Ubuntu:

- Checking  **AppArmor**:

~~~bash
# Checking service
sudo systemctl status apparmor # or apparmor.service

# Checking loading profiles
sudo apparmor_status
~~~

- If the status of **AppArmor** is disable, run the next commands:

~~~bash
sudo systemctl enable apparmor
sudo systemctl start apparmor
~~~

- Checking **SELinux**:

~~~bash
sestatus
~~~

- If **SELinux** isn't installed, you can run:

~~~bash
sudo apt update
sudo apt install selinux-utils selinux-policy-default
~~~

- If **SELinux** is disabled, you have to do the next:

~~~bash
# Open the SELinux file:
sudo nano /etc/selinux/config

# Change the SELinux to enforcing inside the file
SELINUX=enforcing

# Exit the file and reboot
~~~

In the puntual case of Ubuntu, you can also do the next:

~~~bash
# Install SELinux
sudo apt install policycoreutils selinux-utils selinux-basics

# Activate SELinux
sudo selinux-activate

# Set up enforcing mode
sudo selinux-config-enforcing

# Reboot
sudo reboot
~~~

After the installation, you can check SELinux with:

~~~bash
estatus 
~~~

### Setting Mandatory Access Control (MAC) with SELinux:

After you make sure you have **SELinux** on the enforcing mode, let's dive into it.

SELinux is a labeling system, then it labels every process, file, directory and system object. It also provides policy rules control access between labeled processes and objects, where the kernel will also enforce the rule.

In the case of the Docker containers, two measuraments appear:

- **Type enforcement:** The container processes are labeled with ```svirt_lxc_net_t``` and the container files with ```svirt_sandbox_file_t```, then containers processes can only access/write container files by respecting the rule.

- **Multi category Security Enforcement (MCSE):** By adding categories, you can protect a container from another container by still having labels.

The MCSE is based on Multi Level Security (MLS), then when a container is launches, it picks a random MCS label and saves it with the container metadata. so, the Docker daemon tells the kernel to apply the correct MCS when a process container start.

Again, to check SELinux is in enforcing mode, you can check:

~~~bash
sudo setenforce 1
getenforce

# You can check /etc/selinux/config file too.
~~~

Otherwise, make sure to enable it again, you can also try:

~~~bash
sudo selinux-activate
sudo reboot
~~~

Also, check that Docker has the ```--selinux-enabled``` option, it can be found on the */etc/docker/daemon.json

~~~bash
cat /etc/docker/daemon.json
~~~

If the option is not visible, you may have to modify the Docker service to use **SELinux**, so create a new dir on the service and add a *selinux.conf* file.

~~~bash
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo nano /etc/systemd/system/docker.service.d/selinux.conf
~~~

Inside the file, make sure to add the following content:

~~~conf
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd --selinux-enabled
~~~

Then, reload the *systemd* and *Docker* service:

~~~bash
sudo systemctl daemon-reload
sudo systemctl restart docker
~~~

Now, you should be ready to launch a Docker container with SELinux support, for this you have the next options:

- Run with default labels:

~~~bash
docker run --rm --security-opt label=type:container_t ubuntu /bin/bash
~~~

- Run with custom label:

~~~bash
docker run --rm --security-opt label=role:object_r --security-opt label=type:unconfined_t ubuntu /bin/bash
~~~

- Run without confinement:

~~~bash
docker run --rm --security-opt label=disable ubuntu /bin/bash
~~~

For more information you can check on:

- [What is SELinux? | DevOps School](https://www.devopsschool.com/blog/what-is-selinux-and-how-its-selinux-used-in-docker/)

- [Security | Docker Docs](https://docs.docker.com/engine/security/)

### Allow writting to volumes mounted with SELinux ON:

Sometimes we will need access to files that are located due to security. However, when SELinux, we can access in a proper and secure way, to this consider the next command.

- Create a volume with ```-z``` or ```-Z``` option:

~~~bash
docker container run -it -v /tmp:/tmp/host:z ubuntu /bin/bash
~~~

### Removing capabilities in order to prevent power downs:

AS you may know, for some actions inside your machine you require priviledged access. But how can I differ between a priviledged and a unpriviledged process? Well, in this case with the requirement of the user ID (**UID**). If it is 0, then you are a superuse or root; but if it is non-zero, it is unpriviledged. 

Why is this concerning? Because a priviledged process can bupass all kernel permission check. Then, when you are unpriviledged, you pass for a full permission checkin (the credentials inlcude effective UID, effective GID (Group ID) and supplementary group list).

Docker allow us to control, add and remove capabilities for a container, for example, with:

- **chown**: Change Ownership of files or directories
- **dac_override**: Discretionary Access Control Override, then it allows process to bypass file permission checks
- **fowner** : File Owner Override, allow to perform actios on files one doesn't own.
- **kill** : Terminate process.
- **setgid** : Set Group ID for a file or process.
- **setuid** : Set User ID for a file or process.
- **setpcap** : Set capabilities, it allows a process to modify its capability sets.
- **net_bin_service** : Bind to Low Numbered Ports, for prots below 1024 which are privileged.
- **net_raw** : Raw Socket Access, allows communication by using raw sockets.
- **sys_chroot** : Change Root Directory.
- **mknod** : Make device nodes, creation of device nodes like */dev/sda*.
- **setfcap** : Set File Capabilities.
- **audit_write** : Write to Audit log, allow writing to the Linux audit logs.

The capabilities you should keep in mind are:

- **CAP_SETPCAP:** This modifies process capabilities.
- **CAP_SYS_MODULE:** This inserts/removes kernel modules.
- **CAP_SYS_RAWIO:** This modifies Kernel Memory.
- **CAP_SYS_PACCT:** This configures process accounting.
- **CAP_SYS_NICE:** This modifies the priority of processes.
- **CAP_SYS_RESOURCE:** This overrides Resource Limits.
- **CAP_SYS_TIME:** This modifies system clock.
- **CAP_SYS_TTY_CONFIG:** This configures tty devices.
- **CAP_AUDIT_WRITE:** This writes the audit log.
- **CAP_AUDIT_CONTROL:** This configures audit subsystem.
- **CAP_MAC_OVERRIDE:** This ignores kernel MAC Policy.
- **CAP_MAC_ADMIN:** This configures MAC Configuration.
- **CAP_SYSLOG:** This modifies kernel printk behavior.
- **CAP_NET_ADMIN:** This configures network.
- **CAP_SYS_ADMIN:** This helps you catch all containers.

In the case of Docker Cli tools, you can use ```--cap-add``` or ```--cap-drop``` options respectively:

~~~bash
# docker container run --cap-drop <capability> <image> [command]
# docker container run --cap-add <capability> <image> [command]

# Drop Setting IDS
docker container run -it --rm --cap-drop setuid --cap-drop setgid ubuntu /bin/bash

# Activate all, except sys_admin
docker container run -it --rm --cap
~~~

For more information, you can check:

- [Options | Docker Docs](https://docs.docker.com/engine/reference/commandline/run/#options)

### Sharing namespaces between the host and the container

Docker creates six different namespaces (Process, Network, Mount, Hostname, Shared Memory and User) for each container it starts. However, there would be cases, like with Kubernetes, when we want that all the containers in a pod share the same network namespace.

This can be achieved with the ```host``` value for args like ```--net```, ```--pid``` and so on, like the following example:

~~~bash
docker container run -it --rm --net=host --pid=host --ipc=host ubuntu /bin/bash
~~~

# Docker Orchestration and Hosting:

Not all the time you will be in a development environment just running with a single host. Then you may need to spawn multiple containers in different hosts and then orchestrate them. There are differnt tool for this:

- [Docker Compose](https://docs.docker.com/compose): Create apps based on multiple containers.
- [Docker Swarm](https://docs.docker.com/compose): Cluster multiple Docker hosts.
- [Kubernetes](https://kubernetes.io/): Development, scheduling, updating, maintenance and scaling containers.

And there are more, but we will focus on the presented above during the nexts subsections.

## Running app with Docker Compose:

It is a native tool that allows to run interdependent containers to create an application.

To install it you have to run:

~~~bash
pip install docker-compose
~~~

To make it word you have to create and add content to a *.yaml* file where you will set up the Docker containers to consider and in what way to do it. The structure is presented below:

~~~yaml
# This is a yaml file, identation matters.
version: '<version_used>'
service: # To specify which containers to include
    <srv_1_name>: # Name of the first service
        image: <image_name> # Image to consider, if it is a Dockerfile you must provide a path to the file
        restart: <option> # If you want it to restart always or never
        ports: # Specify the port conecctivity between the container and the host
            - <in_port>:<map_port>  # Pair specification
        environment: # Specific set ups for your container
            <env>:<value>
        command: <cmd> # You can run commands (according the image) here
        network_mode: <mode> # It can be 'host' or a user-defined one.
        volumes: # Add volumes for interaction or data exchange (extend)
            - <host>:<container> # Provide the path pair to consider
        deploy: # Deployment options
            resources: # For managing the resources of the containers in deployment
                limits: # Set limits
                    memory: <num>M # Set MB of max memory to use
                reservation: # Save space for processes
                    memory: <num>M # Set MB of memory to reserve
    <srv_2_name>:
        ...
~~~

Then you can run it by using:

~~~bash
docker-compose up
~~~

Let's check an example with two ROS 2 images and a communication example, we have the
next *docker-compose.yml* file (as presented in [example 7](/ma01_docker_and_ros/docker_examples/07_docker_cmp_ros2/)):

~~~yaml
version: '3.8'

services:
  ros2_node_1:
    image: osrf/ros:jazzy-desktop-full
    container_name: ros2_node_1
    command: ros2 run demo_nodes_cpp talker
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=16
    volumes:
      - /dev:/dev 
    deploy:
      resources:
        limits:
          memory: 512M
        reservations:
          memory: 256M

  ros2_node_2:
    image: osrf/ros:humble-desktop-full
    container_name: ros2_node_2
    command: ros2 run demo_nodes_cpp listener
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=16
    volumes:
      - /dev:/dev
    deploy:
      resources:
        limits:
          memory: 512M
        reservations:
          memory: 256M

~~~

Then, you can execute it:

~~~bash
docker-compose up
~~~

![compose_example_hum_jazz](/ma01_docker_and_ros/resources/compose_example_hum_jaz.png)

If you want to bring down the stack you use ```docker-compose down```, you can only build the containers with ```docker-compose build```, you can execute a command in a certain service by using ```docker-compose exec <srv> <cmd>```, and you can list the containenrs in the stack with ```docker-compose ps```

An additional node, is that if you are using **VS Code** with the **Docker Extension**, when having a Docker Compose yaml, it may give you hints (for example, when pulling images by completing the name) and also allow you the execution from the IDE of the complete compose file or sections of it.

![compose_vs_code_help](/ma01_docker_and_ros/resources/compose_vs_code_help.png)

For more information you can check:

- [Docker Compose | Docker Docs](https://docs.docker.com/compose/)

- [Docker Compose Yaml | Docker Docs](https://docs.docker.com/compose/compose-file/)

- [Docker Compose CLI | Docker Docs](https://docs.docker.com/compose/compose-file/)

- [Docker Compose Github | Github](https://docs.docker.com/compose/compose-file/)

## Using clusters with Docker Swarm

It is a native tool for clustering which means it groups multiple Docker hosts into a single pool in which you can launch containers. So let's ilustrate its usage with a simple web demo:

1. Initialize Docker Swarm:

~~~bash
sudo docker swarm init
~~~

As a result it should displaye you a command to run your nodes like this:

```
docker swarm join --token <token_id> <ip>:2377
```

2. Create a simple web:

~~~Python
from flask import Flask

app = Flask(__name__)

@app.route('/')
def hello_world():
    return "Hello from a Docker Swarm"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
~~~

3. Create a Dockerfile for your web application

~~~Dockerfile
# Use the official Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Copy the app code into the container
COPY . /app

# Install dependencies
RUN pip install Flask

# Expose the port the app will run on
EXPOSE 5000

# Run the application
CMD ["python", "app.py"]
~~~

4. Build the Docker image:

~~~bash
sudo docker build -t flask-app .
~~~

5. Deploy the app on Docker Swarm:

~~~bash
sudo docker service create --name flask-app --replicas 3 -p 5000:5000 flask-app
~~~

6. Check the deployment of the application with the CLI terminal commands listed belows, and also on your port *5000* like ```http://<YOUR_UBUNTU_IP>:5000```:

~~~bash
# List:
sudo docker service ls

# Check service's tasks
sudo docker service ps flask-app
~~~

You should see something like this when you are done:

![docker_swarm_example](/ma01_docker_and_ros/resources/docker_swarm_example.png)

After this practice do not forget to stop and remove what you have implemented, you can do the next:

~~~bash
# Stop the services
sudo docker service rm flask-app

# Scale down the replicas
sudo docker service scale flask-app=0

# Leave Docker Swarm
sudo docker swarm leave --force
~~~

For more information, you can check:

- [Docker Swarm | Docker Docs](https://docs.docker.com/engine/swarm/)

## Secrets with Docker Swarm:

There would be cases where you need to control what information is visible to the applications, for example, a password may be required to start a process but it wouldn't be wise to add it in a plain text, then you can use **Docker Secrets** to prevent this, follow the example [09_ros2_with_secret](/ma01_docker_and_ros/docker_examples/09_ros2_with_secret).

1. Let's create a Docker Secrete, in this case a password (make sure do not use any of your password for an example), do not forget to init a swarm before:

~~~bash
# Locate yourself in a directory to safe the desired information
echo "no_my_password" > password.txt

# Create the docker secret
# sudo docker scret create <name> [elements]
sudo docker secret create ros2_pw password.txt

~~~

2. Set up your image and consider the secret inside it:

~~~Dockerfile
FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

COPY entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
~~~

3. Set up the entrypoint to read the Docker Secret

~~~bash
#!/bin/bash

export ROS2_SECRET=$(cat /run/secrets/ros2_pw)

echo "ROS 2 Secret: $ROS2_SECRET"

source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
~~~

4. Go to the location of the Dockerfile and the entrypoint to build the image:

~~~bash
sudo docker build -t ros2_wt_secret .
~~~

5. Create a docker compose file (docker-compose.yml) to define the service and add the secret:

~~~yaml
version: "3.8"

services:
  ros2_talker:
    image: ros2_wt_secret
    deploy:
      replicas: 1
    secrets:
      - ros2_pw
    environment:
      - ROS2_SECRET_PATH=/run/secrets/ros2_pw

secrets:
  ros2_pw:
    external: true
~~~

6. Then, deploy the application by using the *Docker Stack*:

~~~bash
sudo docker stack deploy -c docker-compose.yml ros2_stack
~~~

7. Verifiy the deployment:

~~~bash
# List
sudo docker stack services ros2_stack

# View 
sudo docker service logs ros2_stack_ros2_talker
~~~

You should see something like this:

![docker_secrets_with_ros](/ma01_docker_and_ros/resources/docker_secret_with_ros.png)

Before we finish this topic, there another commands you should know:

- You can inspect a secret:

~~~bash
# docker secret inspect <name>
docker secret inspect ros2_pw
~~~

- You can list all the secrets available with:

~~~bash
docker secret ls
~~~

- To delete a secret you can use:

~~~bash
# docker secret rm <name>
docker secret rm ros2_pw
~~~

- To update a service to remove a secrete you use:

~~~bash
docker service update --secret-rm <secrete_name> <secret_service>
~~~

Finally, to stop and remove what you created you can run:

~~~bash
# Delete stack
sudo docker stack rm ros2_stack

# Delete secret
sudo docker secret rm ros2_pw

# Leave swarm
sudo coker warm leave --force
~~~

For more information, you can check:

- [Docker Secrets | Docker Docs](https://docs.docker.com/engine/swarm/secrets/)

## Setting up a Kubernetes clustering

[Kubernetes](https://kubernetes.io/) is an open source container orchestration tool that was started by Google. It allos deplyment, scheduling, updaing, maintenance and scaling process of nodes, all defined by the user by using YAML or JSON files.

Some key concepts to have in mind are:

- **Pods:** It is the deployment unit that can consist of multiple containers, where each container shares a different namespaces with others in the same pod.

- **Node:** Is a worker node in the Kubernetes cluster and is managed through the master. Pods are deployed on a node, which includes service to run them:

  - *Docker:* Running containers.
  - *Kubelet:* Interacion with the master.
  - *Proxy / Kube-Proxy:* Connection between service and pod.

- **Master:** Host cluster-level control services:

  - *API Server:* RESTful API for interaction with master and nodes.
  - *Scheduler:* For jobs in the cluster, like creation of nodes.
  - *ReplicaSet:* Ensuring certain number of replicas at a given time.
  - *etcd:* The master communicates with it to store configuration inofrmation.

- **Services:** Each pod receives a unique and own IP adress but it can be problematic due to controller configuration. However, an abstraction in the term of services was created to label these in order to define a logical set for managing purposes.

- **Labels:** Key-value pairs attached to objects.

- **Volumes:** A directory that is accesible to the containers in a pods, they aren't the same as Docker Volumes.

For the installation we will need **kubectl**, consider using **curl** as follows:

~~~bash
curl -LO "https://dl.k8s.io/release/v1.24.0/bin/linux/amd64/kubectl"
sudo mv kubectl /usr/local/bin/
sudo chmod +x /usr/local/bin/kubectl
~~~

Also, we will need to install Minikube, you can install it with:

~~~bash
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64
sudo install minikube-linux-amd64 /usr/local/bin/minikube
minikube version
~~~

Now, let's set up the cluster for a simple ROS 2 application.

1. Create an image, in our case, with a Dockerfile looks like this:

~~~Dockerfile
FROM osrf/ros:humble-desktop-full

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN ["/bin/bash", "-c",  "source /opt/ros/humble/setup.bash && colcon build"]

CMD ["ros2 run demo_nodes_cpp talker"]
~~~

2. Build the image:

~~~bash
docker build -t ros_humble_test .
~~~

3. Define *Kubernetes* results:

~~~yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ros2-humble-deployment
spec:
  replicas: 3
  selector:
    matchLabels:
      app: ros-humble-test
  template:
    metadata:
      labels:
        app: ros-humble-test
    spec:
      containers:
      - name: ros-humble-test
        image: ros2_humble_test
        command: ["/bin/bash", "-c", "/opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp listener "]
        resources:
          limits:
            memory: "1Gi"
            cpu: "500m"
---
apiVersion: v1
kind: Service
metadata:
  name: ros-humble-service
spec:
  selector:
    app: ros-humble-test
  ports:
    - protocol: TCP
      port: 8080
      targetPort: 8080
~~~

4. Start Minikube:

~~~bash
minikube start
~~~

5. Deply to Kubernetes:

~~~bash
kubectl apply -f ros_deployment.yaml
~~~

6. Verify the deployment:

~~~bash
kubectl get pods
kubectl logs <pod-name>
~~~

In my case, it looks like this:

![kubernetes_get_first_pods](/ma01_docker_and_ros/resources/kubernetes_get_first_pods.png)

The image is getting and error, and we will cover that topic later.

In case you want to stop the process, you can do the next steps:

~~~bash
minikube stop
minikube delete
~~~

# Integrating DevContainers... 

TODO: Add devcontainers... info

~~~Dockerfile
# Our footprint the Open Source Robotics Foundation image for ROS 2 humble
FROM docker.io/osrf/ros:humble-desktop-full

# Add maintainer info
LABEL maintainer="DanielFLopez1620 <dfelipe.lopez@gmail.com"

# Argument for user
ARG USERNAME=ros2_user

# User Identifier (1000 - 10000 for application accounts)
ARG USER_UID=1000 

# Group identifier (100 + for the user's group)
ARG USER_GID=$USER_UID

# Create the user and give proper permissions
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update system package
RUN apt-get update && apt-get upgrade -y

# Install python package manager
RUN apt-get install -y python3-pip

# Set enviromental variable for calling shell
ENV SHELL /bin/bash

# Configure Colcon Mixin
RUN colcon mixin remove default
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Install Gazebo
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-gz

# Setup CycloneDDS
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install TurtleBot3
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-turtlebot3*

# Configure User
USER $USERNAME
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros2_user/.bashrc
RUN echo "source /home/ros2_user/ws/install/setup.bash || echo 'Workspace not Ready. Run colcon build at /home/ros2_user/ws'" >> /home/ros2_user/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros2_user/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/ros2_user/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> /home/ros2_user/.bashrc

CMD ["/bin/bash"]
~~~


For more information I encourage you to check the projects of [JuanCSUCoder]() which have more field than me on Docker, and you may enjoy them:

- [Flatboat](https://github.com/JuanCSUCoder/FlatBoatProject) : A tool for integrating Docker, Kubernetes and DevContainers into the ROS / ROS 2 workflow.

- [RobotEn](https://github.com/JuanCSUCoder/RobotEn) : Docker and devcontainer environments for your ROS / ROS 2 projects.

# More usage cases of Docker:

We have presented different ideas on Docker with Cli tools and Dockerfile, now it is time to present some usage cases of them, they can be:

- **Prototyping:** Set up a container to test a service or idea we want to implement.
- **Collaboration and distribution:** You can share your images with others to develop on the same terms and avoid dependencies issues or the "do not work on my machine" problem.
- **Continious Integration ([CI](https://aws.amazon.com/devops/continuous-integration/)):** Merging changes into a central repository where tests are running automatically (automated builds) in order to check the quality and security of the code.
- **Continious Delivery ([CD](https://continuousdelivery.com/)):** To introduce changes (new feaures, configuration and changes) into production safely and quickly in a sustainable way.
- **Platform-as-a-Service ([PaaS](https://cloud.google.com/learn/what-is-paas?hl=es)):** For example, with tools like [OpenShift](https://www.redhat.com/es/technologies/cloud-computing/openshift), [CoreOs(https://www.redhat.com/es/technologies/cloud-computing/openshift/what-was-coreos)], [Atomic.io](https://atomic.io/) or [OKD](https://okd.io/).

## Testing with Docker:

Imagine you are working on a Python Project and you got into the test phase, but you have been working with Python 3.9 and the tests must be implemented in Python 3.10. However, when you install Python 3.10 you got issues and problems, now what? 

Well, you can create your own Docker Container for this tests, for example, with a Dockerfile that comes from a **python** in the **3.10** tag. This example is covered on the [06_testing_python](/ma01_docker_and_ros/docker_examples/06_testing_python), where we have a simple addition/substraction codes with brief (and simple) tests to be run with **pytests**, if you check the Dockerfile it will look like this:

~~~Dockerfile
FROM python:3.10
RUN pip install pytest
ADD scripts /scripts
WORKDIR /scripts
CMD ["/usr/local/bin/pytest"]
~~~

To build this image, go to the proper directory and run:

~~~bash
# Move to the directory
cd /path/to/06_testing_python

# Build
docker image build -t python_test -f Dockerfile .
~~~

Then, run the container, do not forget to use the command for the test:

~~~bash
docker container run python_test pytest simple_excersize.py
~~~

In conclussion, you do not need to have all set up in your machine, as you can build images and run containers that uses different versions of the programming languages that are installed in your machine.

# Docker Tips and Tricks:

## Using debug mode

Let's be honest, sometimes you need logs so... why don't you try debugging?

To make this possible you need to reconfigure the Docker Daemon:

~~~bash
# You can do it with the daemon command
dockerd -D

# Or by chaning the json file
cat /etc/docker/daemon.json
# { "debug": true}
~~~

For example, if I run a **Ubuntu** container, I will receive the next:

~~~bash
docker container run --rm ubuntu echo "hello from Ubuntu"

# Then checking the logs...
journalctl -u docker.service
~~~

![docker_debug_example](/ma01_docker_and_ros/resources/docker_debug_example.png)

For more information, you can check:

- [Docker Daemon | Docker Docs](https://docs.docker.com/config/daemon/)

# Additional links and information:

- [Docker Engine API 1.47| Docker Docs](https://docs.docker.com/reference/api/engine/version/v1.47/)

- [Docker SDK for Python | Docker Py](https://docker-py.readthedocs.io/en/stable/)

- [Docker daemon Socket | Docker Docs](https://docs.docker.com/reference/cli/dockerd/#daemon-socket-option)

- [Protect the Docker daemon socket | Docker Docs](https://docs.docker.com/engine/security/protect-access/)

- [Swagger Editor | Swagger](https://editor.swagger.io/)
