# Hybrid ROS 1 and ROS 2 Packges

This is an example of how to configure the same codebase for usage with ROS 1 and ROS 2 at the same time. The goal with this approach is too keep as much common code for better code maintenance across ROS versions.

## Steps to Build
1. Create a toplevel directory, a common source directory, and a workspace for ROS 1 and ROS 2, respectively. Then softlink common source to both workspaces.
```bash
mkdir -p ~/hybrid_toplevel/{one_ws,two_ws,src_common}
ln -s ../src_common ~/hybrid_toplevel/one_ws/src
ln -s ../src_common ~/hybrid_toplevel/two_ws/src
```

2. Clone this repository into the common src directory
```bash
git clone https://github.com/DavidPL1/roscpp_hybrid_example.git ~/hybrid_toplevel/src_common/roscpp_hybrid_example
``` 

3. Build ROS 1 version with either catkin or colcon
```bash
# catkin version
source /opt/ros/one/setup.bash
cd ~/hybrid_toplevel/one_ws
catkin init
catkin b cpp_hybrid
```
*OR*
```bash
# colcon version
source /opt/ros/one/setup.bash
cd ~/hybrid_toplevel/one_ws
colcon build --packages-select cpp_hybrid
```
in case you're on focal with noetic, replace `/opt/ros/one/setup.bash` with `/opt/ros/noetic/setup.bash`

4. Source the devel space and run the ROS 1 nodes
```bash
source ~/hybrid_toplevel/one_ws/devel/setup.bash
rosrun cpp_hybrid talker # run listener in a separate terminal
# alternatively run server and client in separate terminals for the service example
```

5. **In a new terminal** (to not mix ROS one and ROS two environment variables) build the ROS 2 version with colcon
```bash
source /opt/ros/humble/setup.bash
cd ~/hybrid_toplevel/two_ws
colcon build --packages-select cpp_hybrid
```
In case you're using another ROS 2 distribution, choose the according path to source it.

6. Source the install space and run the ROS 2 nodes
```bash
source ~/hybrid_toplevel/two_ws/install/setup.bash
ros2 run cpp_hybrid talker # run listener node in a separate terminal
# alternatively run server and client in separate terminals for the service example
```

# Explanation

Sourcing the base ROS 1 or ROS 2 distribution, among other things sets the environment variable `ROS_VERSION` to either `1` or `2`.
Based on this environment variable we dictate what happens while building the package.

**package.xml**: The same package.xml is used, ensuring package exploration and building the dependency graph works correctly with both colcon and catkin.
Each tag in this file supports adding `condition="..."`, where the condition within the string is evaluated and decides whether the tag is considered or ignored. By setting the condition in ROS 1 specific elements to "${ROS_VERSION} == 1" and respectively "${ROS_VERSION} == 2" for ROS 2, we ensure each distribution only considers what is relevant to it. Common tags such as package depends with a consistent name across ROS versions don't need any addition (e.g. `<depends>geometry_msgs</depends>`).

**CMakeLists.txt**: In the toplevel CMakeLists file we setup the common project name and options. Then we evaluate the `ROS_VERSION` environment variable to decide which CMake script with specific calls to catkin (for ROS 1) or ament (for ROS 2) to include.
After including the specialized CMake scripts, we generate a header file which defines which ROS version to use within the code base. 

**cpp/hpp files**: By including the generated header, in this case `#define MJ_ROS_VERSION` is called with the ROS version CMake was called with. This definition can now be checked in the code to decide whether to compile ROS 1 or ROS 2 specific code. You can choose a different name to differentiate between ROS versions, just make sure to not choose a too generic name to avoid redefinition.
Within each cpp/hpp file we begin by including the `ros_version.h` header, then we proceed with separate blocks for ROS version specific includes and definitions.
For instance, in `client.cpp` we define a macros *INFO* and *INFO_STREAM* which redirect the input either to *RCLCPP_INFO* and *RCLCPP_INFO_STREAM* or *ROS_INFO* and *ROS_INFO_STREAM* based on the ROS version compiled with. This allows for using the same ROS version independent macro for logging.
Similar to this concept, we alias the `rclcpp` and the `ros` namespaces to `ros_cpp` to get independent calls to basic functions like `ros_cpp:ok()` and `ros_cpp::shutdown()`.
In this example this kind of separation and aliasing is somewhat overkill, but it serves as an example applicable to much more complicated code which is not easy to split up. The goal of this approach is to keep as much common code, making code maintenance much easier then coordinating fixes and updates between separate branches or even repositories for each version which inevitably diverge over time.

We recommend choosing an approach like in `server.cpp`, where some core functionality is defined -- in this case `core_add` which adds two ints and returs the sum -- and the specific ROS code calls these core functions. Ideally the ROS specific code would be kept in separate implementation files. Function and type definitions might be spread to separate header files or, if this is not possible (e.g., within a class declaration which can not be separated into multiple header files), conditionally compiled using `#if MJ_ROS_VERSION == ...` directives.