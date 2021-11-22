# simple_navigation_goals

```
mkdir -p ~/bot_ws/src
cd ~/bot_ws/src
git glone git@github.com:mjd-x/simple_navigation_goals.git
cd ..
catkin_make install
source devel/setup.bash
```

`~/bot_ws/src/simple_navigation_goals/launch/movebase_seq.launch`

```
roslaunch simple_navigation_goals movebase_seq.launch
```

[Sending a sequence of goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py)
