FROM ros:humble-ros-base
ARG USERNAME=brad
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DIALOUT_GID=20

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get update && apt-get install -y udev python3-serial && \
    groupadd -g ${DIALOUT_GID} dialout || true && \
    usermod -aG dialout ${USERNAME} || true


RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-humble-desktop

ENV SHELL /bin/bash

USER $USERNAME
CMD ["/bin/bash"]
