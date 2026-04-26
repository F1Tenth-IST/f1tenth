FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
#RUN apt-get install -y ros-humble-foxglove-bridge
RUN apt-get install -y ros-humble-plotjuggler-ros

ENV SHELL=/bin/bash

# ------------------------
# System dependencies
# ------------------------
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    python3-venv \
    libblas-dev \
    liblapack-dev \
    gfortran \
    pkg-config \
    libnanoflann-dev \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash

# ------------------------
# Install acados
# ------------------------
RUN git clone https://github.com/acados/acados.git /home/$USERNAME/acados && \
    cd /home/$USERNAME/acados && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make -j$(nproc) && \
    make install

ENV ACADOS_INSTALL_DIR=/usr/local
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

RUN pip3 install -e /home/$USERNAME/acados/interfaces/acados_template

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
# Python = Python3
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1 

# ------------------------
# ROS + acados sourcing
# ------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
#RUN echo "source /home/$USERNAME/f1tenth_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source $ACADOS_INSTALL_DIR/examples/c/env.sh" >> ~/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************


# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

CMD ["/bin/bash"]