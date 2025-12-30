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

No devices were found

This output tells us something important and a bit counter-intuitive: the package manager thinks NVIDIA is installed, but the kernel does not see a GPU driver at all.

```

>The system is not seeing the NVIDIA kernel driver at all, despite the userspace packages being installed. This is no longer an apt dependency problem; it’s a driver–kernel attachment failure.

>nvidia-smi saying “No devices were found” is not a userspace problem. It means the NVIDIA kernel module is missing, unloaded, or blocked, or the GPU isn’t being bound to it.

Let’s reason this through carefully.
First, what this is not:

- Not a CUDA issue
- Not a PATH issue
- Not a ROS issue
- Not a permissions issue

This is a driver ↔ kernel failure.

#### The key facts:

- nvidia-driver-580 is installed
- nvidia-smi → “No devices were found”
- That message means the kernel module is not loaded or not allowed to load
- If the GPU were dead or unsupported, you’d usually see a different error

Let’s walk this forward, skeptically, and rule out the real causes.

#### Step 1: Confirm the GPU exists at the hardware level

Expected:
>01:00.0 VGA compatible controller: NVIDIA Corporation ...

- If nothing appears → BIOS / VM / hardware passthrough issue
- If it does appear → kernel-driver problem (most likely)

```
$ lspci | grep -i nvidia
02:00.0 VGA compatible controller: NVIDIA Corporation Device 2c58 (rev a1)
02:00.1 Audio device: NVIDIA Corporation Device 22e9 (rev a1)
```

####  Step 2: Check if the NVIDIA kernel module is loaded

```
lsmod | grep nvidia
```

##### Outcomes:

- Nothing printed → driver never loaded
- nvidia, nvidia_drm, nvidia_uvm present → module loaded, but device blocked elsewhere

Given your symptoms, expect nothing.

#### Step 3: Check Secure Boot (very common failure)

On Ubuntu, Secure Boot silently blocks NVIDIA modules unless they’re enrolled.

```
mokutil --sb-state
```

If you see:

>SecureBoot enabled

Then this is almost certainly the problem.

##### Why this matters

- NVIDIA kernel modules are out-of-tree
- Secure Boot blocks them unless you:
- enroll a MOK key, or
- disable Secure Boot

#### Fastest fix (recommended unless you need SB):

- Reboot
- Enter BIOS/UEFI
- Disable Secure Boot
- Boot back into Ubuntu
Run:
```
$ sudo modprobe nvidia
$ nvidia-smi
```

If Secure Boot was the blocker, nvidia-smi will immediately start working.

#### Step 4: Check kernel / driver mismatch

If Secure Boot is already disabled, then suspect a kernel build issue.

```
uname -r
dkms status
```

You should see something like:

> nvidia/580.105.08, <kernel>, installed

If instead you see:
- added
- built
- or nothing for nvidia

Then DKMS did not successfully build the module.

##### Force rebuild:
```
sudo apt install --reinstall nvidia-dkms-580
sudo dkms autoinstall
sudo reboot
```

#### Step 5: Check for nouveau interference

Even when purged, nouveau can still load early.

```
lsmod | grep nouveau
```

If it appears:
```
sudo nano /etc/modprobe.d/blacklist-nouveau.conf
```

Add
```
blacklist nouveau
options nouveau modeset=0
```

Then:

```
sudo update-initramfs -u
sudo reboot
```

#### Why this happened (important perspective)

- Apt did its job: userspace is consistent
- The failure is below apt, in the kernel boundary
- NVIDIA drivers are fragile across:
    - kernel updates
    - Secure Boot
    - DKMS rebuild timing

The “31 not upgraded” line is a red herring — unrelated

This is exactly why GPU issues feel opaque: the failure mode is silent refusal, not an error.

### Minimal checklist (most likely path)

In order of probability:

- lspci | grep -i nvidia → confirm GPU exists
- mokutil --sb-state → disable Secure Boot if enabled
- dkms status → rebuild if missing
- Ensure nouveau is gone

If you paste the outputs of Steps 1–3, we can pinpoint it in one more round instead of guessing.



### Analysis 

```
$ lspci | grep -i nvidia

02:00.0 VGA compatible controller: NVIDIA Corporation Device 2c58 (rev a1)
02:00.1 Audio device: NVIDIA Corporation Device 22e9 (rev a1)

$ lsmod | grep nvidia

nvidia_uvm           2076672  0
nvidia_drm            135168  0
nvidia_modeset       1638400  1 nvidia_drm
nvidia              104091648 2 nvidia_uvm,nvidia_modeset
drm_ttm_helper         16384  2 nvidia_drm,xe
nvidia_wmi_ec_backlight    12288  0
video                  77824  5 nvidia_wmi_ec_backlight,dell_wmi,xe,i915,nvidia_modeset
wmi                    28672  10 dell_wmi_sysman,video,nvidia_wmi_ec_backlight,dell_wmi_ddv,alienware_wmi,dell_wmi,wmi_bmof,dell_smm_hwmon,dell_smbios,dell_wmi_descriptor

```

#### What this definitively tells us

##### 1. Hardware is real and visible

> lspci | grep -i nvidia
02:00.0 VGA compatible controller: NVIDIA Corporation Device 2c58

###### PCIe sees the GPU. This rules out:
- dead hardware
- BIOS completely hiding the GPU
- wrong machine / VM

So the GPU exists and is enumerated.

##### 2. Kernel driver is loaded and attached
```
nvidia
nvidia_modeset
nvidia_drm
nvidia_uvm
```


##### This means:

- The NVIDIA kernel modules successfully loaded
- Secure Boot is probably not blocking the module (otherwise they wouldn’t load)
- The kernel accepted the driver

##### This rules out:

- unsigned driver rejection
- missing kernel headers
- dkms build failure


#### Check if Secure Boot is silently blocking the driver

This is extremely common on Ubuntu and gives exactly your symptoms.

Check Secure Boot state:

```
$ mokutil --sb-state
SecureBoot disabled
```

#### Check if Nouveau is grabbing the GPU first

```
$ lsmod | grep nouveau

```

If nouveau is loaded, NVIDIA can’t attach.

This usually means the blacklist didn’t apply because the NVIDIA module failed to build.



##### So why does nvidia-smi say “No devices were found”?
At this point, the failure is not installation-level. It’s runtime / mode-level.

There are only a few realistic causes left.

#### Most likely causes (ranked)
🔴 1. GPU is in compute-disabled / power-gated state (common on laptops)

On many laptops (especially Dell/Alienware, which your modules suggest), the GPU:

- is powered off by ACPI
- only turns on when a PRIME offload client requests it
- loads the kernel driver, but exposes no active devices

This produces exactly:

- lspci sees GPU
- lsmod shows NVIDIA
- nvidia-smi sees nothing

```
# quick test 
$ cat /proc/driver/nvidia/gpus/*

cat: '/proc/driver/nvidia/gpus/0000:02:00.0': Is a directory
```

If this directory exists but shows minimal info or errors → power-gating issue.


🔴 2. PRIME configuration incomplete or broken

Check:

```
$ prime-select query
on-demand
```
If it says intel (or nothing), the NVIDIA GPU is not the active provider.

Try: 

```
sudo prime-select nvidia
sudo reboot
```

After reboot:
```
nvidia-smi
```

