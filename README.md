# Course: Intelligent Mobile Robot Techniques

## To run the navigation

- To start (where `xx` is the world number, 0~299):

```bash
cd ~/barn_ws
source devel/setup.bash
python3 src/scripts/run.py --gui --world_idx xx
```

- To completely terminate the processes:

```bash
cd ~/barn_ws/src
source kill_ros_gazebo.bash
```

- Useful TF commands:

```bash
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf view_frames
```



## References
- [DWA](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py).
