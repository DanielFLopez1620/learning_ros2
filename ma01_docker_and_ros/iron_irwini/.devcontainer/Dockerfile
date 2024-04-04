FROM docker.io/osrf/ros:iron-desktop-full
ARG USERNAME=learner
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_UID -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAMe ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

USER $USERNAME
RUN echo "source /opt/ros/iron/setup.bash" >> /home/learner/.bashrc
CMD ["/bin/bash"]