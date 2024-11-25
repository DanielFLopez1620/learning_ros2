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