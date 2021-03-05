# Tinker21 UPBoard Hardware Setup Guide

## 1. Install Ubuntu 20.04

## 2. Disable HSUART DMA

Ref: [UP Wiki - Hang on Shutdown or Reboot for UP Board](https://github.com/up-board/up-community/wiki/Ubuntu_20.04)

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

## 3. Install CAN Driver

## 4. Install ROS Noetic

## 5. Install ZSH (Optional)

Install ZSH

Install OhMyZsh

Install zsh-autosuggestions zsh-auto-completion

Install powerlevel10k with fonts
