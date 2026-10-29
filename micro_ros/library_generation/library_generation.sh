#!/bin/bash

######## Configure Raspberry Pi Pico SDK  ########

apt update

apt install -y gcc-arm-none-eabi

######## Init ########

cd /uros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

######## Adding extra packages (tf2_msgs) ########
pushd firmware/mcu_ws > /dev/null

    # Workaround: Copy just tf2_msgs
    git clone -b jazzy https://github.com/ros2/geometry2
    cp -R geometry2/tf2_msgs ros2/tf2_msgs
    rm -rf geometry2


popd > /dev/null

######## Build for Raspberry Pi Pico SDK  ########
rm -rf firmware/build

ros2 run micro_ros_setup build_firmware.sh /project/library_generation/toolchain.cmake /project/library_generation/colcon.meta

find firmware/build/include/ -name "*.c"  -delete
mkdir -p /project/libmicroros/include
cp -R firmware/build/include/* /project/libmicroros/include

cp -R firmware/build/libmicroros.a /project/libmicroros/libmicroros.a

######## Fix include paths  ########
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

apt -y install rsync
for var in ${INCLUDE_ROS2_PACKAGES}; do
    rsync -r /project/libmicroros/include/${var}/${var}/* /project/libmicroros/include/${var}
    rm -rf /project/libmicroros/include/${var}/${var}
done

######## Generate extra files ########
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > /project/available_ros2_types
# sort it so that the result order is reproducible
sort -o /project/available_ros2_types /project/available_ros2_types

cd firmware
echo "" > /project/built_packages
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> /project/built_packages; popd > /dev/null; done;
# sort it so that the result order is reproducible
sort -o /project/built_packages /project/built_packages

######## Fix permissions ########
sudo chmod -R 777 /project/libmicroros /project/available_ros2_types /project/built_packages
sudo chmod -R -x+X /project/libmicroros
