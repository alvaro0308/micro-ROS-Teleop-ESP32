# micro-ROS-Teleop-ESP32

# Installation
``
mkdir microros_ws

cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update

rosdep install --from-path src --ignore-src -y

colcon build

source install/local_setup.bash
``

## Create firmware
``
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i [LOCAL MACHINE IP ADDRESS] -p 8888
``

## WIFI configuration
``
ros2 run micro_ros_setup build_firmware.sh menuconfig
``

## Build and flash firmware
`
ros2 run micro_ros_setup build_firmware.sh

ros2 run micro_ros_setup flash_firmware.sh
`

## Agent micro-ROS
``
ros2 run micro_ros_setup create_agent_ws.sh

ros2 run micro_ros_setup build_agent.sh

source install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
``

## Publish message
### O: Forward
### 1: Left
### 2: Stop
### 3: Right
### 4: Backward
### 5: Auto
``
ros2 topic pub --once /int1 std_msgs/msg/Int32 '{data: [MESSAGE]}' 
``


## Debugging Monitor Installation
``
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp

cd ~/esp

git clone --recursive https://github.com/espressif/esp-idf.git

cd ~/esp/esp-idf

./install.sh esp32
``

## Copy into .bashrc
``
vim ~/.bashrc

alias get_idf='. $HOME/esp/esp-idf/export.sh'
``

``
source ~/.bashrc

get_idf

cd ~/esp

cp -r $IDF_PATH/examples/get-started/hello_world .

cd ~/esp/hello_world

idf.py set-target esp32

idf.py menuconfig
``

## Launch monitor
``
idf.py -p /dev/ttyUSB0 monitor
``
