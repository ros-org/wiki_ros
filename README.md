# tutorial community

get all ros resources at <http://wiki.ros.org/>

***
# quick installation (for indigo)

>(arm)$ sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

>(!arm)$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

>(arm)$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

>(!arm)$ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

>(arm)$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

>$ sudo apt-get update

>(!arm)$ sudo apt-get install ros-indigo-desktop-full

>(arm)$ sudo apt-get install ros-indigo-ros-base

>(arm)$ sudo apt-get install python-rosdep

>(arm)$ sudo apt-get install python-twisted

>$ sudo rosdep init

>$ rosdep update

>$ sudo apt-get install python-rosinstall

>(arm)$ unset GTK_IM_MODULE

***
# quick configuration (for indigo)

>$ source /opt/ros/indigo/setup.bash

>$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

>$ mkdir -p ~/workspaces/hitrobot

>$ (for devel)git clone git@github.com:hitrobotgroup/release

>$ (for release)git clone -b $ROS_DISTRO https://github.com/hitrobotgroup/release

>$ (for devel)git clone git@github.com:hitrobotgroup/dbparam

>$ (for release)git clone https://github.com/hitrobotgroup/dbparam

>$ (for devel)ln -s ~/workspaces/hitrobot/ros_org ~/catkin_ws

>$ (for release)ln -s ~/workspaces/hitrobot/release ~/catkin_ws

>$ cd ~/catkin_ws/src

>$ catkin_init_workspace

>$ cd ~/catkin_ws/

>$ catkin_make

>$ source ~/catkin_ws/devel/setup.bash

>$ echo "source ~/catkin_ws/base.sh" >> ~/.bashrc

>$ cd ~/catkin_ws/src

create package as:

>$ catkin_create_pkg [package_name] [depend1] [depend2] [depend3] ...

install package as: (replace underscores with dashes of the package name)

>$ sudo apt-get install ros-indigo-[stack_or_package_name]

***
# interesting stacks

* [slam_gmapping](http://wiki.ros.org/slam_gmapping) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/slam_gmapping-hydro-devel.zip)
* [hector_mapping](http://wiki.ros.org/hector_mapping) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/hector_slam-catkin.zip)
* [octomap_mapping](http://wiki.ros.org/octomap_server) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/octomap_mapping-indigo-devel.zip)
* [navigation](http://wiki.ros.org/navigation) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/navigation-indigo-devel.zip)
* [hector_navigation](http://wiki.ros.org/hector_navigation) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/hector_navigation-catkin.zip)
* [teb_local_planner](http://wiki.ros.org/teb_local_planner) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/teb_local_planner-indigo-devel.zip)
* [navigation_layers](http://wiki.ros.org/navigation_layers) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/navigation_layers-indigo.zip)
* [straf_recovery](http://wiki.ros.org/straf_recovery) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/straf_recovery-master.zip)
* [people](http://wiki.ros.org/people) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/people-indigo-devel.zip)
* [joystick_drivers](http://wiki.ros.org/joystick_drivers) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/joystick_drivers-indigo-devel.zip)
* [diagnostics](http://wiki.ros.org/diagnostics) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/diagnostics-indigo-devel.zip)
* [cob_control](http://wiki.ros.org/cob_control) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/cob_control-indigo_dev.zip)
* [cob_driver](http://wiki.ros.org/cob_driver) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/cob_driver-indigo_dev.zip)
* [urg_node](http://wiki.ros.org/urg_node) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/urg_node-indigo-devel.zip)
* [sick_tim](http://wiki.ros.org/sick_tim) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/sick_tim-indigo.zip)
* [lms1xx](http://wiki.ros.org/LMS1xx) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/LMS1xx-master.zip)
* [rplidar_ros](http://wiki.ros.org/rplidar_ros) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rplidar_ros-master.zip)
* [velodyne](http://wiki.ros.org/velodyne) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/velodyne-master.zip)
* [openni2_launch](http://wiki.ros.org/openni2_launch) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/openni2_launch-indigo-devel.zip)
* [robot_web_tools](http://wiki.ros.org/robot_web_tools) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/robot_web_tools-develop.zip)
* [laser_filters](http://wiki.ros.org/laser_filters) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/laser_filters-indigo-devel.zip)
* [scan_tools](http://wiki.ros.org/scan_tools) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/scan_tools-indigo.zip)
* [robot_pose_publisher](http://wiki.ros.org/robot_pose_publisher) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/robot_pose_publisher-develop.zip)
* [tf2_web_republisher](http://wiki.ros.org/tf2_web_republisher) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/tf2_web_republisher-develop.zip)
* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) - [zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rosbridge_suite-develop.zip)
* [roslibjs](http://wiki.ros.org/roslibjs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/roslibjs-develop.zip) |  [eventemitter2.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/eventemitter2.min.js) |  [roslib.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/roslib.min.js)
* [ros2djs](http://wiki.ros.org/ros2djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros2djs-develop.zip) | 
[easeljs.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/easeljs.min.js) |  [ros2d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros2d.min.js)
* [ros3djs](http://wiki.ros.org/ros3djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros3djs-develop.zip) | 
[three.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/three.min.js) |  [ros3d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/ros3d.min.js)
* [nav2djs](http://wiki.ros.org/nav2djs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/nav2djs-develop.zip) | 
[nav2d.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/nav2d.min.js)
* [keyboardteleopjs](http://wiki.ros.org/keyboardteleopjs) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/keyboardteleopjs-develop.zip) | 
[keyboardteleop.min.js](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/keyboardteleop.min.js)
* [rms](http://wiki.ros.org/rms) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/rms-develop.zip)
* [sophus](http://wiki.ros.org/sophus)
* [ecl](http://wiki.ros.org/ecl)

***
# interesting repositories
* [jsoncpp](https://github.com/open-source-parsers/jsoncpp) - 
[zip](https://raw.githubusercontent.com/ouiyeah/wiki_ros/master/src/jsoncpp-master.zip)

* freeglut3-dev
* libv4l-dev
* git clone https://github.com/ar-tools/ar_tools

* sudo apt-get install cmake
* sudo apt-get install libgoogle-glog-dev
* sudo apt-get install libatlas-base-dev
* sudo apt-get install libeigen3-dev
* sudo apt-get install libsuitesparse-dev

***
# interesting nodes
* tf_broadcaster
* rosbridge_shell
* rosbridge_driver
* rosbridge_mapoint
* rosbridge_waypoint

***
# cartographer
````
# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build

# Create a new workspace in 'catkin_ws'.
mkdir -p ~/workspaces/hitrobot/cartographer
cd ~/workspaces/hitrobot/cartographer
wstool init src

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
sed -i 's/ceres-solver.googlesource.com/github.com\/ceres-solver/' src/.rosinstall
wstool update -t src

# Add eigin path in src/ceres-solver/CMakelist.txt
# include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
# include_directories(/usr/include/eigen3)
mkdir -p src/ceres-solver/build
cd src/ceres-solver/build
cmake ..
make –j
sudo make install
cd ../..

# Install proto3.
src/cartographer/scripts/install_proto3.sh

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
````

***
# interesting robots
