## Networking

- Laptop

```bash
export ROS_IP=192.168.31.4
export ROS_MASTER_URI=http://192.168.31.105:11311

export ROS_IP=192.168.1.119
export ROS_MASTER_URI=http://192.168.1.119:11311
```

- Xavier
```bash
export ROS_IP=192.168.31.105
export ROS_MASTER_URI=http://192.168.31.105:11311
```
```bash
# Launch xarm
roslaunch dashgo_moveit_config dashgo_xarm.launch robot_ip:=192.168.31.180
```