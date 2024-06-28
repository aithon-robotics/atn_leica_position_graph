# atn_leica_position_graph
Thesis: "Factor graph based stat estimation for UAVs in construction environments"

# Installation
Install [ROS noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).

Install gtsam & graph_msf by following [these installation instructions](https://github.com/leggedrobotics/graph_msf/blob/main/doc/installation.md)
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin init
cd src
git clone https://github.com/aithon-robotics/atn_leica_posiition_graph.git
git clone https://github.com/aithon-robotics/atn_leica_timesync.git
cd ..
catkin build
```

# Launch
To start atn_leica_position_graph sensor fusion two Terminals are needed:
```
source ~/catkin_ws/devel/setup.bash
roslaunch atn_leica_position_graph graph_leica.launch
```

```
source ~/catkin_ws/devel/setup.bash
rosrun atn_leica_timesync atn_leica_timesync_node 
```

# Related packages

| Package         | Description                     | Link                                                           |
| --------------- | ------------------------------- | -------------------------------------------------------------- |
| atn_leia_timesync| Timesynchronization of Total Station & System Clock | [atn_leia_timesync](https://github.com/aithon-robotics/atn_leica_timesync)         |
| graph_msf        | Factor Graph Framework this package is based on     | [graph_msf](https://github.com/leggedrobotics/graph_msf/tree/main) |

