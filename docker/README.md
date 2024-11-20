# Docker images setup

### Machine prerequisites

- Docker Engine (Linux) / Docker Desktop (Not recommended)
- NVIDIA Driver
- NVIDIA Container Toolkit ([Instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html))
- Docker group ([Linux post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/))

## Build images

Inside the `docker` folder in each module, there are different docker images. The convention for the naming of the **images** chosen is `home:area-version`, where version contains the platform used: `cpu`, `cuda` or `l4t-version`. Also, the `Dockerfile` should be named according to the feature and version.

Current images available:
- Noetic base full

### Noetic base example
This is the example for setting up a basic ROS Noetic image, used to create all the workspaces and capable of the basic ROS features (catkin, rviz, rqt).

The dockerfile was named `Dockerfile.noetic.full`, and the following command was executed to build the image from the root of the repo:

```bash
docker build -f docker/Dockerfile.noetic.full -t home/main:noetic-full . # -f to specify the path to the file, -t to add the name and the tag
docker images # In the output, the image home/main with tag noetic-full should appear
```

## Create a container

With the image built, create a container with a name similar to `home-feature`. The philosophy of the containers is to be ephimeral and any important command that is needed to run inside a container should be instead in the `Dockerfile` of the image.

### Noetic base example
Using the previously built image (`home/main:noetic-full`), the following `run` command was executed in the root of the repo to create an interactive bash container:

```bash
xhost + # To support visual features
docker run -it --name home-main --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 --user $(id -u):$(id -g) -v $(pwd):/workspace home/main:noetic-full bash # The name is home-main, most of the additional parameters are needed for visual tools
```

The parameter `-v $(pwd):/workspace` mounts the current directory in the local machine, inside of the `/workspace` folder of the container.

## Execution of the container

After the `run` command for the creation, for entering the container and developing, the following commands have to be executed:

```bash
docker start container-name
xhost +
docker exec -it --user $(id -u):$(id -g) container-name bash
```

**Important:** The `user` parameter is neccessary in order to run docker in rootless mode, otherwise, files and folders created inside the container will be protected and only editable using `sudo`.

## Examples and Detailed Explanations of ROS Nodes

### `receptionist_task_manager.py`
The `receptionist_task_manager.py` node manages the receptionist task, including guest interaction, face recognition, and navigation to the living room. It coordinates various subtask managers for HRI, manipulation, navigation, and vision to achieve the desired behavior.

### `breakfast_task_manager.py`
The `breakfast_task_manager.py` node manages the breakfast task, including picking and placing objects, pouring, and navigating to different locations. It ensures the robot can perform the necessary actions to serve breakfast.

### `demo_tmr24.py`
The `demo_tmr24.py` node demonstrates the task manager for the breakfast task, including guest interaction, face recognition, and serving breakfast. It showcases the capabilities of the robot in a controlled environment.

### `task_manager_server.py`
The `task_manager_server.py` node serves as a template for developing multiple task managers for various tasks in RoboCup@Home. It provides a base structure for managing tasks and coordinating subtask managers.

### `hri_tasks.py`
The `hri_tasks.py` node manages the implementation of Human-Robot Interaction (HRI) tasks, including speaking, guest information retrieval, and guest analysis. It ensures the robot can effectively interact with humans.

### `manipulation_tasks.py`
The `manipulation_tasks.py` node manages the implementation of manipulation tasks, including picking, placing, and moving arm joints. It ensures the robot can perform the necessary actions to manipulate objects.

### `nav_tasks.py`
The `nav_tasks.py` node manages the implementation of navigation tasks, including moving to specific locations and storing the current location. It ensures the robot can navigate effectively within its environment.

### `vision_tasks.py`
The `vision_tasks.py` node manages the implementation of vision tasks, including saving face names, checking for persons, and finding free seats. It ensures the robot can accurately process visual information and interact with its environment.
