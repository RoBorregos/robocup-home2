Pull the image
```bash
sudo run.sh
```
Create the container
```bash
docker compose up
```

> [!NOTE]
> The container wont start the zed until it has the models loaded up on memory. you will see  the next logs:
> ```bash
> zed  | [2024-11-17 17:08:44 UTC][ZED][INFO] Please wait while the AI model is being optimized for your graphics card
> zed  |  This operation will be run only once and may take a few minutes
> ```
> until you doont see the next log: youll need to wait:
> ```bash
> zed  | [ INFO] [1731863532.828266244]: ZED connection [LIVE CAMERA with ID 0]: SUCCESS
> ```

To manually run the zed in the container
```bash
docker compose up -d
```

```bash
docker exec -it zed bash
```

```bash
source devel/setup.bash
export ROS_IP=192.168.31.3
export ROS_MASTER_URI=http://192.168.31.105:11311
```

```bash
roslaunch zed_wrapper zed2_robot.launch
```