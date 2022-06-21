# Lift Navigation ROS Program
Turtlebot3 navigation on map of two floors and lift created via Gmapping

## Usage

1. Launch empty world simulator
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

2. Change `image` in `lift_map.yaml` to path of `lift_map.pgm`

3. Open map to navigate 
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:={absolute path of lift_map.yaml}
```

## Other Authors
* Hesham AlKhouja
