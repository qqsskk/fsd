# FS Driverless 2019

This is a repository for the autonomous code of a driverless FS race car. Original repository:
https://github.com/AMZ-Driverless/fsd_skeleton

- - - -

# Aliases (useful commands)
Once running `update_dependencies`, some aliases for commands will be added. Restart the terminal and the following commands will be available:
* `FSD_source`: sources environment from anywhere
* `FSD_cd`: change directory to root directory of skeleton_repo
* `FSD_build`: clean and build project (catkin clean and catkin build)
* `FSD_launch_acceleration`: launch mission, e.g. acceleration, trackdrive, autox etc
* `FSD_rviz_acceleration`: launch RVIZ with custom config for mission, e.g. acceleration, trackdrive, autox etc 
* `FSD_ATS`: run automated test

Look at`fsd_aliases` to see full list, or add more custom aliases.
- - - -

# Setting up the Workspace
**1 Clone the repository:**

```
cd ~
git clone git@github.com:AMZ-Driverless/fsd_skeleton.git
```
**2 Install dependencies**
```
cd ~/fsd_skeleton
./update_dependencies.sh
```

**3 Build workspace**
```
cd ~/fsd_skeleton
catkin build
```

**4 Source environment**

Assuming you've run `./update_dependencies.sh` succesfully and restarted the terminal.
```
FSD_source
```
Else,
```
cd ~/fsd_skeleton
source fsd_environment.sh
```

**5 Test setup**
```
roslaunch fsd_common_meta trackdrive.launch
```
in new terminal
```
rqt_graph
```
You should see all the nodes int the pipeline running
- - - -

# Conventions
- - - -
## ROS naming conventions
We use the naming conventions defined at http://wiki.ros.org/ROS/Patterns/Conventions
### Work packages:
`work_package`, lowercase and `_` as separator, e.g. `lidar`.
### ROS packages:
`workpackage_somename`, lowercase and `_` as separator, e.g. `lidar_trimmer`, as to make it clear what the package is used for.
### ROS nodes
`node_name`, lowercase and `_` as separator. Can be short.
### ROS topics
`topic_name`, lowercase and `_` as separator.
### ROS messages
`CamelCased.msg` for message filenames. Message types are always CamelCase, whereas message fields are lowercase and `_` as separator, e.g.
```
MyMessage.msg:
Header header
Float64 my_float
geometry_msgs/Point my_point
```

## Style guides
### ROS C++:
Google Style (http://wiki.ros.org/CppStyleGuide)

* Files: `under_scored`, exception for `.msg` files, `CMakeLists.txt`.
* Classes/types: `CamelCase`
* Functions/methods: `camelCase`
* Variables: `under_scored` and DESCRIPTIVE.
* Constants: `ALL_CAPITALS`.
* Global variables: AVOID except special cases. Rather have parameters defined in `config.yaml`.

### ROS Python
PEP-8 style (http://wiki.ros.org/PyStyleGuide)