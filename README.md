# Tinker_UP21
Tinker21 UPBoard Project

# Setup Guide

## - Setup Hardware Drivers and Environment

Please follow [Hardware/README.md](Hardware/README.md)

## - Compile Workspace

```bash
cd Workspace
catkin_make
```

Add workspace to init (replace ```/home/one/Tinker_UP21``` with absolute path of this directory)
```bash
echo "source /home/one/Tinker_UP21/Workspace/devel/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

## - Setup Chassis Node

### Add execution privilliges

Execute following command in current directory
```bash
chmod +x Workspace/src/rcbigcar/scripts/*
```

### Create startup script

Write following content to ```/etc/systemd/system/launch_ros_robot.service```

Replace ```/home/one/Tinker_UP21/``` with absolute path of this directory
```ini
[Unit]
Description=ROS Robot Controller

After=network-online.target
Wants=network-online.target

[Service]
User=root
ExecStart=/home/one/Tinker_UP21/Workspace/src/rcbigcar/scripts/launch.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable services
```bash
sudo systemctl daemon-reload
sudo systemctl enable launch_ros_robot.service
sudo systemctl start launch_ros_robot.service
```

Check if controller status is active
```bash
sudo systemctl status launch_ros_robot.service
```

Example output:
```
● launch_ros_robot.service - ROS Robot Controller
     Loaded: loaded (/etc/systemd/system/launch_ros_robot.service; enabled; ven>
     Active: active (running) since Thu 2021-03-18 16:11:10 CST; 46min ago
   Main PID: 1020 (launch.sh)
      Tasks: 22 (limit: 3948)
     Memory: 79.3M
     CGroup: /system.slice/launch_ros_robot.service
             ├─1020 /bin/bash /home/one/Tinker_UP21/Workspace/src/rcbigcar/scri>
             ├─1079 /usr/bin/python3 /opt/ros/noetic/bin/roslaunch rcbigcar all>
             ├─1247 /usr/bin/python3 /opt/ros/noetic/bin/rosmaster --core -p 11>
             ├─1259 /opt/ros/noetic/lib/rosout/rosout __name:=rosout __log:=/ro>
             └─1263 /home/one/Tinker_UP21/Workspace/devel/lib/rcbigcar/robot __>

3月 18 16:11:10 one-upboard systemd[1]: Started ROS Robot Controller.
```
## - Setup SLAM

Install Hokuyo driver
```bash
sudo apt install ros-noetic-urg-node
```

Install Hokuyo udev rules

Create ```/etc/udev/rules.d/99-one.hokuyo-usb.rules``` and write following content

```bash
# Assign Hokuyo Lidar by serial number (WIP)
# http://wiki.ros.org/urg_node
SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", SYMLINK+="hokuyo"
```

Create new workspace and install cartographer
```bash
cd ~
mkdir Tinker_ENV21
cd Tinker_ENV21


```

## - Setup Robotic Arm
