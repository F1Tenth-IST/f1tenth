# 📦 Installation Guide

This guide walks through setting up the full F1TENTH stack either:

- 🐳 Inside the provided Docker container (recommended)
- 💻 Natively on a Linux machine (requires additional installation of ACADOS)

---


## 🚀 Setup Instructions

### 1. Open/build the Docker Container

Open this repository in VSCode
Then:
  Ctrl + Shift + P → "Dev Containers: Rebuild and Reopen in Container"
This will automatically:
- Build the Docker image (ROS 2 Humble base)
- Install some system dependencies
- Install ACADOS
- Configure the environment (default space is /home/ros)
- Mount the workspace to /home/ros/f1tenth


### 2. Initialize Submodules
Inside the container, make sure you are under /home/ros user and that your repo is cloned/mounted on f1tenth folder (~/f1tenth).

```bash
cd ~/f1tenth/src

git config --global url."https://github.com/".insteadOf git@github.com:

git submodule init
git submodule update
```

### 3. Ignore Optional (not used yet) heavy packages
As of now the COLCON_IGNORE files are not in the repos, but they maybe in the future, rendering this step unnecessary. Last update: 26/04/2026

```bash
cd ~/f1tenth

touch src/zed-ros2-wrapper/COLCON_IGNORE
touch src/f1tenth_system/zed_rgb_node/COLCON_IGNORE
touch src/f1tenth_system/zed_sdk_cpp/COLCON_IGNORE
touch src/control/mpc_curv_ls_v1/COLCON_IGNORE
touch src/control/mpc_curv_ls_v2/COLCON_IGNORE
touch src/control/mpc_curv_ls_v3/COLCON_IGNORE
touch src/control/mpc_curv_off_ls_v1/COLCON_IGNORE
touch src/control/mpc_curv_off_ls_v2/COLCON_IGNORE
touch src/control/mpc_curv_off_v1/COLCON_IGNORE
```

### 4. Install ROS dependencies/libraries
This block is functional as of 26/04/2026 and should take a few minutes to complete.

```bash
cd ~/f1tenth

sudo apt update
sudo apt install \
  ros-humble-diagnostic-updater \
  ros-humble-ackermann-msgs \
  ros-humble-diagnostic-updater \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-asio-cmake-module \
  ros-humble-serial-driver \
  ros-humble-urg-node \
  ros-humble-robot-localization \
  ros-humble-mavros-msgs \
  ros-humble-nmea-msgs \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-xacro
```

### 5. Build main workspace/packages
This step will take a few minutes to complete, expect more than 10 minutes.
Some of the packages will give stderr output, but it is safe to ignore as long as they all say "Finished" without any aborted or failed packages.

```bash
cd ~/f1tenth

colcon list
colcon build --symlink-install
```

At the end of this step, you should have a `f1tenth` folder in your home directory ready to run almost everything on the real setup. Nevertheless, it is recommended to install the following packages/software to make the setup more convenient and have an integrated development environment (foxglove bridge, f1tenth_gym_ros).

### 6. Install Foxglove Bridge
This foxglove bridge is used to visualize the vehicle's state and sensor data in real-time, but it is not the client side, it refers to the server side. The client side can be easily installed like any other application.

```bash
cd ~/f1tenth

rosdep update
rosdep install --from-paths src --ignore-src -r -y

sudo apt install ros-humble-foxglove-bridge
```
For installation on the Jetson nano, feel free to skip the next step refering simulation.


### 7. Install F1Tenth Gym Ros Simulation Environment

1. Create directory for all simulation related packages
```bash
cd ~
mkdir -p sim_ws/src
```

2. Clone F1TENTH Gym
```bash
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym
pip3 install -e .
```

3. Clone F1TENTH Gym ROS (our fork)
```bash
cd ~/sim_ws/src
git clone https://github.com/F1Tenth-IST/f1tenth_gym_ros
```

4. Install dependencies
```bash
cd ~/sim_ws
rosdep install -i --from-path src --rosdistro humble -y
```

5. Build workspace
```bash
cd ~/sim_ws
colcon build
```


### 8. Source workspaces & update bashrc
```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth/install/setup.bash
source ~/sim_ws/install/setup.bash
```

Add to bashrc so that the workspaces are always loaded when you open a new terminal.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/f1tenth/install/setup.bash" >> ~/.bashrc
echo "source ~/sim_ws/install/setup.bash" >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc

source ~/.bashrc
```

---

## ⚙️ Native Installation (Skip if using Dev Container)

Not yet tested, but it should work.

If you are **NOT using the Dev Container**, you must manually install ACADOS and Python dependencies.

👉 If you used VSCode Dev Containers, **you can skip this entire section**.

---

### 1. Install ACADOS

```bash
cd ~

git clone https://github.com/acados/acados.git
cd acados

git submodule update --init --recursive

mkdir build
cd build

cmake -DACADOS_WITH_QPOASES=ON ..
make -j$(nproc)
sudo make install

# Environment variables
echo "export ACADOS_INSTALL_DIR=/usr/local" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
echo "source \$ACADOS_INSTALL_DIR/examples/c/env.sh" >> ~/.bashrc

source ~/.bashrc
```

### 2. Install ACADOS Python Interface
```bash
pip3 install -e ~/acados/interfaces/acados_template
```

### 3. Install Python Dependencies
```bash
cd ~/f1tenth
pip3 install --no-cache-dir -r requirements.txt
```

### 4. Ensure Python Points to Python3
```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
```