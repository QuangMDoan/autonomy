# Install ros2 kilted on ubuntu 24.04.3

- link: https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html


## System setup

### Set locale

Make sure you have a locale which supports UTF-8. 

If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. 

We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
$ locale  # check for UTF-8

$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

$ locale  # verify settings

```

### Enable required repositories

You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.

```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
```

The ros-apt-source packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source package will configure ROS 2 repositories for your system. 

Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.


```bash
$ sudo apt update && sudo apt install curl -y
$ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

$ echo $ROS_APT_SOURCE_VERSION
$ 1.1.0


$ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

$ echo "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.noble_all.deb

$ sudo dpkg -i /tmp/ros2-apt-source.deb

```

### Install development tools (optional)

If you are going to build ROS packages or otherwise do development, you can also install the development tools:


```bash
$ sudo apt update && sudo apt install ros-dev-tools
```

## Install ROS 2

Update your apt repository caches after setting up the repositories.

```bash
$ sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. 

It is always recommended that you ensure your system is up to date before installing new packages.

```bash
$ sudo apt upgrade

Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Calculating upgrade... Error!
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 libnvidia-gl-580 : Conflicts: libnvidia-egl-gbm1
E: Unable to correct problems, you have held broken packages.

```

### What’s actually wrong (skeptical view)

- NVIDIA drivers are monolithic: all libnvidia-* packages must be from the same driver series.
- You currently have mixed generations installed (likely 550/560 leftovers + 580 repo enabled).
- libnvidia-egl-gbm1 is from an older driver branch and explicitly conflicts with the 580 GL library.
- Apt cannot auto-resolve this because NVIDIA marks these conflicts as mutually exclusive.

This is not a “held package” in the casual sense — it’s a driver family mismatch.


#### Step 1: See what NVIDIA packages you actually have

Run this first to confirm the mess:

You’ll almost certainly see multiple versions (550/560 + 580).

```bash
$ dpkg -l | grep -i nvidia

qd@a18q:~$ dpkg -l | grep -i nvidia
ii  cuda-nsight-compute-13-1                        13.1.0-1                                 amd64        NVIDIA Nsight Compute
ii  cuda-nsight-systems-13-1                        13.1.0-1                                 amd64        NVIDIA Nsight Systems
ii  cuda-nvtx-13-1                                  13.1.68-1                                amd64        NVIDIA Tools Extension
ii  libnvidia-cfg1-580:amd64                        580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA binary OpenGL/GLX configuration library
ii  libnvidia-common-580                            580.95.05-0ubuntu0.24.04.3               all          Shared files used by the NVIDIA libraries
ii  libnvidia-compute-580:amd64                     580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA libcompute package
ii  libnvidia-decode-580:amd64                      580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA Video Decoding runtime libraries
ii  libnvidia-egl-wayland1:amd64                    1:1.1.13-1ubuntu0.1                      amd64        Wayland EGL External Platform library -- shared library
ii  libnvidia-encode-580:amd64                      580.95.05-0ubuntu0.24.04.3               amd64        NVENC Video Encoding runtime library
ii  libnvidia-extra-580:amd64                       580.95.05-0ubuntu0.24.04.3               amd64        Extra libraries for the NVIDIA driver
ii  libnvidia-fbc1-580:amd64                        580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA OpenGL-based Framebuffer Capture runtime library
ii  libnvidia-gl-580:amd64                          580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA OpenGL/GLX/EGL/GLES GLVND libraries and Vulkan ICD
ii  linux-modules-nvidia-580-open-6.14.0-37-generic 6.14.0-37.37~24.04.1                     amd64        Linux kernel nvidia modules for version 6.14.0-37
ii  linux-modules-nvidia-580-open-generic-hwe-24.04 6.14.0-37.37~24.04.1                     amd64        Extra drivers for nvidia-580-open for the generic-hwe-24.04 flavour
ii  nsight-compute-2025.4.0                         2025.4.0.12-1                            amd64        NVIDIA Nsight Compute
ii  nvidia-compute-utils-580                        580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA compute utilities
ii  nvidia-driver-580-open                          580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA driver (open kernel) metapackage
ii  nvidia-firmware-580-580.95.05                   580.95.05-0ubuntu0.24.04.3               amd64        Firmware files used by the kernel module
ii  nvidia-kernel-common-580                        580.95.05-0ubuntu0.24.04.3               amd64        Shared files used with the kernel module
ii  nvidia-kernel-source-580-open                   580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA kernel source package
ii  nvidia-prime                                    0.8.17.2                                 all          Tools to enable NVIDIA's Prime
ii  nvidia-settings                                 510.47.03-0ubuntu4                       amd64        Tool for configuring the NVIDIA graphics driver
ii  nvidia-utils-580                                580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA driver support binaries
ii  screen-resolution-extra                         0.18.3ubuntu0.24.04.1                    all          Extension for the nvidia-settings control panel
ii  xserver-xorg-video-nvidia-580                   580.95.05-0ubuntu0.24.04.3               amd64        NVIDIA binary Xorg driver

```

### Step 2: Decide your strategy (important)

You have two sane options:

### Option A — Stay on your current driver (safer)

If your system was working before and you don’t need 580 yet.

### Option B — Fully move to 580 (cleaner long-term)

Best if you recently enabled a new NVIDIA PPA or Ubuntu just pushed 580.

Do not try to partially upgrade. That’s what broke it.


## Option B: Clean upgrade to NVIDIA 580 (recommended if you’re moving forward)

This is the most reliable fix.

### 1. Purge all NVIDIA packages

```bash
$ sudo apt purge '^nvidia-.*' '^libnvidia-.*'
$ sudo apt autoremove
```

### 2. Reboot (non-negotiable)

You should boot using nouveau or basic graphics — that’s fine.

```bash
$ sudo reboot
```

### 3. Install driver 580 cleanly

```bash
$ sudo apt update
$ sudo apt install nvidia-driver-580
```

or, if you prefer auto-selection:

```
$ sudo ubuntu-drivers autoinstall
```

### 4. Verify

After reboot:

```
$ nvidia-smi
```

