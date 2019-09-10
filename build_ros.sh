echo "Building ROS nodes"

cd rosws
catkin_make
source devel/setup.bash
cd ..
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Examples/ROS
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$(pwd)/rosws/devel/lib/pkgconfig
cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
