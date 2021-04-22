# Tinker21 UPBoard Hardware Setup Guide

## Install Ubuntu 20.04

Download from official site

[Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)

## Install UPBoard Kernel

Ref: [Install Ubuntu for UP, UP Squared, UP Core, UP Core Plus and UP Xtreme](https://github.com/up-board/up-community/wiki/Ubuntu_20.04)

Add upboard kernel repo:

```bash
sudo add-apt-repository ppa:aaeon-cm/5.4-upboard
sudo apt update
```

Remove generic linux kernel:
```bash
sudo apt-get autoremove --purge 'linux-*-generic'
```

Install upboard kernel (18.04 and 20.04 share the same 5.4 kernel):
```bash
sudo apt-get install linux-generic-hwe-18.04-5.4-upboard
```

Install updates:

```bash
sudo apt dist-upgrade -y
sudo update-grub
```

Reboot

```bash
sudo reboot
```

After the reboot, you can verify that the kernel is indeed installed by typing

```bash
uname -a
```

> Linux upxtreme-UP-WHL01 5.4.0-1-generic #2~upboard2-Ubuntu SMP Thu Jul 25 13:35:27 UTC 2019 x86_64 x86_64 x86_64 GNU/Linux

## Disable HSUART DMA

Ref: [UP Wiki - Hang on Shutdown or Reboot for UP Board](https://github.com/up-board/up-community/wiki/Ubuntu_20.04)

To resolve kernel panic on poweroff

edit /etc/modprobe.d/blacklist.conf and add lines to the bottom

```bash
blacklist dw_dmac_core
install dw_dmac /bin/true
install dw_dmac_core /bin/true
```

Then execute

Ref: [Modules loading despite being added to the blacklist](https://askubuntu.com/questions/51321/modules-loading-despite-being-added-to-the-blacklist)

```bash
sudo update-initramfs -u
```

## Install CAN Driver

Disable original UPBoard SPI devices
Edit /etc/default/grub, add following kernel command line parameters:
```bash
GRUB_CMDLINE_LINUX="up_board.spidev0=0 up_board.spidev1=0"
```


Install self-written MCP2515 CAN driver in this repo

Go to Hardware/mcp2515_kernel folder, and execute:
```bash
rm -r build
sudo make install
```

## Install ZSH

Install ZSH

Install OhMyZsh

Install zsh-autosuggestions zsh-auto-completion

Install powerlevel10k with fonts

## Install ROS Noetic

Ref: [ROS Wiki: noetic/Installation](http://wiki.ros.org/noetic/Installation)

Setup your sources.list

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update package list

```bash
sudo apt update
```

Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

```bash
sudo apt install ros-noetic-desktop-full
```

Add environment setup instructions to .zshrc
```bash
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```
