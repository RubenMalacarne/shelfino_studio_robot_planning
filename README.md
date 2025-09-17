# Course Project: Robot Planning and Application- AY 2024/2025

## Project Overview
Designed for static environments involving to develop a path planning algorithm to move the robot.
This project combines theoretical knowledge with practical implementation, focusing on algorithms such as Probabilistich Random Map and Combinatorial planning to find the optimal path for a robot in a random environments.

User can select different algorithms to see how they perform in various scenarios. It's just launch the different launch files and run the service to send the path planning request.

This Demo Video shows the project in action: [Demo Video](https://drive.google.com/file/d/1mH0Wm3G9giH5IraRVNd6KBnHFPfy1Ne8/view).

## Technologies Used

- **Robot**: Shelfino of the University of Trento
- **Programming Languages**: C++ (core ROS 2 nodes), Python (to launch the nodes)
- **Robotic Framework**:  [ROS 2](https://docs.ros.org/en/humble/index.html)
- **Simulation Environment**: [Gazebo](https://gazebosim.org/docs/latest/ros2_integration/)
- **Motion Planning**: [NAV2](https://docs.nav2.org/) (only to send the commands to the robot)
- **Task Planning**: Custom ROS 2 nodes
- **Docker user:** YES
- **CI/CD-GithubAction:** YES 
- **Radiocommand**: [RadioMaster pocket](https://radiomasterrc.com/collections/pocket-radio) with [OpenTX](https://www.open-tx.org/) firmware



## Simulation Scene

The simulation scene includes the following key components:

- 2 shelfino robots, that spawn in a random position in the map
- A static environment with obstacles (can be Rectangles or circles obstacles) and walls 
- Differente shape of the map (exagon, square)
- Rviz2 for visualizing the robot's path and environment and other useful information


![alt text](images/gazebo_rviz.png)

## Installation Guide

This guide explains how to set up the project on your local machine, using docker.

>**Important**: If you want to run the project without docker, you need to have ROS 2 Humble installed on your machine, along with Gazebo and other dependencies (see dockerfile).

>**Note**: docker use 6 GB

If you prefer you can see this video tutorial: [Video Tutorial](https://drive.google.com/file/d/1NN9B0Se9HPpIK6yeOjC1gBDoOoel8Dkg/view) to install and setup the project, or you can follow the steps below:

1. **Prerequisites**: Ensure you have Docker and Docker Compose installed on your machine. You can download them from the following links:
   - [Docker Installation Guide](https://docs.docker.com/get-docker/)
   - [Docker Compose Installation Guide](https://docs.docker.com/compose/install/)

2. **Clone the Repository**: Start by cloning the project repository to your worksapce folder.

   ```bash
   git clone https://github.com/RubenMalacarne/shelfino_studio_robot_planning.git src
   git submodule update --init --recursive
   cd src
   ```

3. **Allow Docker to Access X Server**: To enable GUI applications within the Docker container, you need to allow access to your X server. Run the following command:
   ```bash
      xhost +local:root
      ```

4. **Build and run the Docker Compose**: Navigate to the directory containing the `docker-compose.yml` file and build the Docker image.

   ```bash
   docker compose up -d rp_project
   ```

5. **Access the Docker Container**: Once the container is running, you can access it using the following command:

   ```bash
   docker exec -it ros2_robot_planning_project bash
   ```

6. if you want use a radiocommand of the joy_manager_ros2 package, you need to connect the radiocommand to your computer via USB and give the permission to access the device. You can do this by running the following command (inside docker):

   ```bash
   sudo chmod 666 /dev/input/js0
   sudo chmod 666 /dev/input/event*
   sudo chmod 666 /dev/hidraw*
   ```

7. **compile the project**: inside the docker container

   ```bash
   colcon build --symlink-install --parallel-workers 1 && source install/setup.bash 
   ```
   > Note: can be happen some error during the build, just ignore it, do the `source install/setup.bash` and try to build again.

Now you are inside the docker container and you can run the project. ;)

## Usage Instructions
To run the simulation and test the path planning algorithms, follow these steps:

1. **Launch the Simulation**: Use the provided launch files to start the simulation environment.

   ```bash
   ros2 launch projects evacuation.launch.py
   ```

2.  **Launch the planning algorithm**:

      for probabilistic planning: [video example](https://drive.google.com/file/d/1mH0Wm3G9giH5IraRVNd6KBnHFPfy1Ne8/view)
      ```bash
      ros2 launch planning_pkg prm_executor.launch.py
      ``` 

      for combinatorial planning:[video example](https://drive.google.com/file/d/1z2rgzg68H7R2UbFWQP6JDs7ZisLfmGm1/view)
      ```bash
      ros2 launch planning_pkg comb_executor.launch.py
      ``` 

3.  **Send Path Planning Requests**: Use the provided service to send path planning requests to the robot.

       ```bash
       ros2 service call /service_trigger_dubins_path std_srvs/srv/Trigger {}
       ```

## Project Structure

```
workspace/
    ├── planning_pkg                --> package that contains the path planning part
    |   ├── launch                  --> folder that contains the launch files
    |   ├──src                      --> folder that contains the source code
    |   |    ├──algo_path_planning  --> folder that contains the path planning algorithms
    |   |    ├──utils               --> folder that contains utility functions
    |   |    └──path_orchestrator_xxx.cpp  --> main node that orchestrates the path planning
    |   └──include                  --> folder that contains the header files
    ├── projects                    --> package that contains the launch file to launch the project  
    ...

```

## Link of the Report

If you want interest to see the report of the project, you can find it here: [Report Link](https://drive.google.com/drive/folders/1Q7MqyY9uJhHKeTNzAhIWHm9b5ftJSl7X?usp=sharing) 

Enjoy the project! :D
