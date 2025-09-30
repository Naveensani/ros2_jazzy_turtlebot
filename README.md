# ros2_jazzy_turtlebot
Repository to test out Docker Images and Container with ROS2 Jazzy

# Cloning Instructions:

To clone a specific version (here v1.0.0) from Github:

```bash

git clone --branch v1.0.0 --depth 1 https://github.com/Naveensani/ros2_jazzy_turtlebot.git

```
# Building Instructions:

To build the Image (Can take a while):

```bash

cd ros2_jazzy_turtlebot/

docker build -t naveensani22/ros2_jazzy_turtlebot:1.0.0 .

```
naveensani22/ros2_jazzy_turtlebot:1.0.0 name is helpful if you are planning to upload this image that is build into Dockerhub. Here naveensani22 is my Dockerhub username. Any name can be used for this matter.
And the image that is build can be see using:

```bash
docker images
```

# Running Instructions:

To Run this image as a container (Please use the name of the container that you want after --name and use the name of image that you used at the end. And give all the devices,dislay and permissions as required):

```bash

docker run -it \
  --name ros2_jazzy_turtle_docker \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:1.0.0

```

After this command, you should see the terminal of the container that is running now. And the later use this same container, use (use the name of the container that you used):

```bash
docker start -ai ros2_jazzy_turtle_docker
```
