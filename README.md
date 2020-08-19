# MobRob_ROS_myrobot
ROS-Package: **myrobot_model** package for MobRob

![alt text](https://techniccontroller.de/wp-content/uploads/IMG_20200413_191813_edit-508x381.jpg "MobRob")

## Features

- visual model of MobRob with movable joints: [urdf/mobrob_move.urdf](urdf/mobrob_move.urdf)
- visual model of MobRob with fixed joints: [urdf/mobrob_fixed.urdf](urdf/mobrob_fixed.urdf)
- own service descriptions for Mobrob: [srv/AttinyCommand.srv](srv/AttinyCommand.srv)
- own topic descriptions for MobRob: [msg/Pose.msg](msg/Pose.msg)

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
