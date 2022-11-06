# ROS2 Beginner Tutorial
## Installing ROS2 on Ubuntu 22.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html) to install ROS2 Humble on your new Ubuntu 22.04 Desktop (binary installation).
## Installing ROS2 on Ubuntu 20.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) carefully and select Ubuntu 20.04 wherever it is necessary and install from source.
## Tutorials
- Follow these ![tutorial](http://docs.ros.org/en/humble/Tutorials.html) to understand the basics of ROS2. If you have installed the ROS2 from source, the ros2 setup path will be your 
```
. <installation directory>/install/setup.bash
```
- Source installation installs turtlesim and other packages. No need to use the commands listed in the above tutorial. 
- `colcon` can be installed by running the command:
```
sudo apt install python3-colcon-common-extensions
```
## Build Package and run Publisher-Subscriber
Follow the below instructions to build the simple publisher and subscriber package.
- Clone this repository. It should create beginner_tutorials folder in your present working directory.
```
git clone https://github.com/okritvik/beginner_tutorials.git
```
- Run the below commands:
```
cd beginner_tutorials
rosdep install -i --from-path src --rosdistro humble -y
colcon build
. install/setup.bash
ros2 run cpp_pubsub talker
```
- Open a new terminal, source it from 
```
. <ros2_installation_directory>/install/setup.bash
```
and source your present package installation
```
. install/setup.bash
```
- Run the subscriber
```
ros2 run cpp_pubsub listner
```

## Static Code Analysis
### cpplint
Run the below command from the project root folder `beginner_tutorials`
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/cpp_pubsub/src/*.cpp
```
### cppcheck
Run the below command from the project root folder `beginner_tutorials`
```
cppcheck --enable=all --std=c++17 src/cpp_pubsub/src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheck.txt
```
