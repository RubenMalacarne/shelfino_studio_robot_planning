# CODE to work the robot planning project


## Start the docekr container

To run everything related to Shelfino and the Path Generator, first start the Docker container (or click "run service inside the GUI o inside yml file"):

```bash
docker compose up --build
```
> note: if you already build: 

```bash
docker compose up
```

after that open docker with:
```bash
docker exec -it ros2_robot_planning_project bash
```


## Build the project

and compile the project with:
```bash
colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
```
**Note**:
> If you encounter errors like "obstacles_msgs" ... something, run:

```bash
source install/setup.bash && colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
```

## Launch a demo
launch the project: 

```bash
ros2 launch projects evacuation.launch.py 
```

To test the navigation and the basic path generator (with waypoints), open two terminals and:

```bash
ros2 run planning_pkg nav2_pathclient.cpp
```
```bash
ros2 run planning_pkg main_base
```