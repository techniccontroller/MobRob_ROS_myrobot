# MobRob_ROS_myrobot
ROS-Package: **myrobot_model** package for MobRob

![alt text](https://techniccontroller.de/wp-content/uploads/IMG_20200413_191813_edit-508x381.jpg "MobRob")

## Features

- visual model of MobRob with movable joints: [urdf/mobrob_move.urdf](urdf/mobrob_move.urdf)
- visual model of MobRob with fixed joints: [urdf/mobrob_fixed.urdf](urdf/mobrob_fixed.urdf)
- own service descriptions for Mobrob: [srv/AttinyCommand.srv](srv/AttinyCommand.srv)
- own topic descriptions for MobRob: [msg/Pose.msg](msg/Pose.msg)
- xacro model of MobRob with inertial properties for gazebo simulation: [urdf/mobrob_move.xacro](urdf/mobrob_move.xacro)
- launch files for MobRob: [launch/](launch/)

## Visualize the model in RViz

Run following command to visualize the [urdf/mobrob_move.urdf](urdf/mobrob_move.urdf) model in RViz:

```
roslaunch myrobot_model display.launch
```

![alt text](https://github.com/techniccontroller/MobRob_ROS_myrobot/blob/master/urdf/mobrob_move.png "mobrob_move.urdf")

Run following command to visualize the [urdf/mobrob_fixed.urdf](urdf/mobrob_fixed.urdf) model in RViz:

```
roslaunch myrobot_model display.launch model:='$(find myrobot_model)/urdf/mobrob_fixed.urdf'
```


## Animate joints in the model

Change the joint publisher in [launch/display.launch](launch/display.launch) to animate the joint state by own state publisher. 
