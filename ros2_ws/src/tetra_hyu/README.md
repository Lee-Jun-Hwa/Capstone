# Hyulim TETRA_ROS2(Humble)


## Install Dependencies
```bash
  sudo apt install ros-humble-joint-state-publisher
```


## Install Dependencies Packages
```bash
  cd ~/ros2_ws/src/
  git clone https://github.com/RoverRobotics-forks/serial-ros2.git
```

## Uploading USB Rules
```bash
  cd ~/ros2_ws/src/tetra_hyu
  chmod +x create_udev_rules.sh
  ./create_udev_rules.sh
```