# micro-ROS-Teleop-ESP32
The goal of this program is to teleoperate an Arduino Robot using ROS2. To make this possible an ESP32 with micro-ROS is connected to Arduino over UART.

# Installation
```
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source install/local_setup.bash
```

## Create firmware
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
ros2 run micro_ros_setup configure_firmware.sh teleop_arduino -t udp -i [LOCAL MACHINE IP ADDRESS] -p 8888
```

## WIFI configuration
```
ros2 run micro_ros_setup build_firmware.sh menuconfig
```

## Build and flash firmware
```
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```

## Agent micro-ROS
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Debugging Monitor Installation
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf
./install.sh esp32
```

## Copy into .bashrc
```
vim ~/.bashrc
alias esp='. $HOME/esp/esp-idf/export.sh'
```

```
source ~/.bashrc
get_idf
cd ~/esp
cp -r $IDF_PATH/examples/get-started/hello_world .
cd ~/esp/hello_world
idf.py set-target esp32
idf.py menuconfig
```

## Launch monitor
```
esp
idf.py -p /dev/ttyUSB0 monitor
```

## Launch micro-ROS Agent
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Move robot
O: Forward

1: Left

2: Stop

3: Right

4: Backward

5: Auto

```
ros2 topic pub --once /microROS/teleop_arduino std_msgs/msg/Int32 data:\ [ACTION]\
```

![](https://github.com/alvaro0308/micro-ROS-Teleop-ESP32/blob/main/resources/gif.gif)
