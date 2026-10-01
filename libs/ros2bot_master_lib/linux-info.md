# Linux Info

## Fast Fetch

Good tool to retrieve system information.

```
get https://github.com/fastfetch-cli/fastfetch/releases/download/2.69.0/fastfetch-linux-aarch64.deb
sudo apt install ./fastfetch-linux-aarch64.deb
fastfetch
```

## JetPack

Current version query

```
sudo apt-cache show nvidia-jetpack | grep "Version"
```

```
cat /etc/nv_tegra_release
```

## ROS2

Query current version information.

```
echo $ROS_DISTRO
```

```
printenv ROS_DISTRO
```

```
env | grep ROS
```

## Node / JavaScript

```
nvm -v
```

```
node -v
```

## Python

Current version.
```
python3 --version
```

Install virtual environment.
```
sudo apt install python3.12-venv
```

## ROS 2 Distributions and Compatible Ubuntu Versions
* ROS 2 Lyrical Luth (May 2026 – May 2031): Compatible with Ubuntu 26.04 LTS (Resolute).
* ROS 2 Kilted Kaiju (May 2025 – December 2026): Compatible with Ubuntu 24.04 LTS.
* ROS 2 Jazzy Jalisco (May 2024 – May 2029): Compatible with Ubuntu 24.04 LTS (Noble Numbat).
* ROS 2 Iron Irwini (May 2023 – November 2024): Compatible with Ubuntu 22.04 LTS.
* ROS 2 Humble Hawksbill (May 2022 – May 2027): Compatible with Ubuntu 22.04 LTS (Jammy Jellyfish).
* ROS 2 Galactic Geochelone (May 2021 – December 2022): Compatible with Ubuntu 20.04 LTS.
* ROS 2 Foxy Fitzroy (June 2020 – June 2023): Compatible with Ubuntu 20.04 LTS (Focal Fossa).
* ROS 2 Rolling Ridley (Rolling release): Tracks upcoming and current Ubuntu development versions.

## Stereolabs / ZED Camera

Install dependencies

```
sudo apt update
sudo apt install zstd udev lsb-release wget less -y
```

Download SDK
```
https://www.stereolabs.com/developers/release
```

Make install executable & install
```
chmod +x ZED_SDK_Tegra_L4T39.2_v5.5.0.zstd.run
./ZED_SDK_Tegra_L4T39.2_v5.5.0.zstd.run
```

Query ZED SDK Info
```
cat /usr/local/zed/include/sl/Camera.hpp | grep "ZED_SDK_"
```

## Slamtec / Rplidar / S2

Verify the device node appears under Linux (typically /dev/ttyUSB0). You may need to grant dialout permissions:
```
sudo usermod -aG dialout $USER
```

Clone and build the official driver in your ROS 2 workspace:
* https://github.com/Slamtec/sllidar_ros2
* cCompile the package inside your workspace using: colcon build.

Launch the RPLIDAR S2 node using the package launch file:
```
ros2 launch sllidar_ros2 sllidar_s2_launch.py
```

To visualize the scan data, open RViz2 in your environment and add a LaserScan display subscribed to /scan

## CUDA 13 Installation

```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin
sudo mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/13.4.2/local_installers/cuda-repo-ubuntu2404-13-4-local_13.4.2-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2404-13-4-local_13.4.2-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2404-13-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-13-4
```

```
sudo apt-get update
grep -RniE 'cuda|nvidia' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null
ls -la /var/cuda-repo-ubuntu2404-13-4-local/
```

```
sudo dpkg -i ./cuda-repo-ubuntu2404-13-4-local_13.4.2-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2404-13-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
```

```
sudo apt-get -y install cuda-toolkit-13-4
```

### TensorRT 11

Download
```
https://developer.nvidia.com/tensorrt/download/11x
```

## ROS2

Automatically source ROS and rplidar test project

```
cat >> ~/.bashrc <<'EOF'

# ROS 2 Jazzy + sllidar workspace
source /opt/ros/jazzy/setup.bash
[ -f ~/sllidar_test_ws/install/setup.bash ] && source ~/sllidar_test_ws/install/setup.bash
EOF
source ~/.bashrc
```

## Robot Board Serial Setup

The robot board is a CH340 (`1a86:7523`, revision `8134`). The other CH340
observed on this machine was revision `8133`. The CP2102N (`10c4:ea60`) is
the lidar, not the master board. ttyUSB numbers and USB paths change when
devices are moved between the Orin and the hub.

With the board connected and powered, run the setup from the library directory:

```bash
cd ~/Ros2bot/libs/ros2bot_master_lib
bash setup_master_board.sh --check
bash setup_master_board.sh
```

The script requires a bound `ch341` kernel driver, a unique `8134` adapter,
and no other udev rule assigning `r2bserial`. It creates the stable alias
and verifies a version reply. Keep the `rplidar` rule for the CP2102N.
For the original failure analysis and driver/module diagnostic commands,
see `~/Documents/ros2bot-master-board-troubleshooting.md`.

