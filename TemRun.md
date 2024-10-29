- Run in jetson orin

```bash
sudo su
cd /workspace/ws/
source devel/setup.bash
export ROS_IP=192.168.31.5
export ROS_MASTER_URI=http://192.168.31.105:11311
roslaunch hri recepcionist_laptop.launch
```

- Run in jetson xavier

```bash
cd /workspace/ws/
export ROS_IP=192.168.31.105
export ROS_MASTER_URI=http://192.168.31.105:11311
roslaunch hri recepcionist_xavier.launch
```

- Run in laptop

- Run KWS

```bash
cd /workspace/ws/
export ROS_IP=192.168.31.4
export ROS_MASTER_URI=http://192.168.31.105:11311
roslaunch hri kws.launch
```

- Run main engine

```bash
cd /workspace/ws/
export ROS_IP=192.168.31.4
export ROS_MASTER_URI=http://192.168.31.105:11311
rosrun frida_task_manager task_manager_server.py
```
