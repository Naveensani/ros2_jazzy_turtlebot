# ros2_jazzy_turtlebot
Repository to test out Docker Images and Container with ROS2 Jazzy

# Part 1: How to clone the repo and build the image using the files (Dockerfile and others), and then running it as a container

# Cloning Instructions:

To clone a specific version (here v1.0.0) from Github:

```bash

git clone --branch v1.0.0 --depth 1 https://github.com/Naveensani/ros2_jazzy_turtlebot.git

```
# Building Instructions:

To build the Image (Can take a while) (Use Image name of your preference):

```bash

cd ros2_jazzy_turtlebot/

docker build -t naveensani22/ros2_jazzy_turtlebot:1.0.0 .

```
naveensani22/ros2_jazzy_turtlebot:1.0.0 name for the image is helpful if you are planning to upload this image that is build into Dockerhub. Here naveensani22 is my Dockerhub username. Any name can be used for this image that is about to be build.
And the image that is build can be seen using:

```bash
docker images
```

# Running Instructions:

To Run this image as a container (Please use the name of the container that you want after --name and use the name of image that you named at the end. And give all the devices,display and permissions as required):

```bash

docker run -it \
  --name ros2_jazzy_turtle_docker \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:1.0.0

```

After this command, you should see the terminal of the container that is running now. And to exit the container use:

```bash
exit
```

And to later use this same container, use (use the name of the container that you named):

```bash
docker start -ai ros2_jazzy_turtle_docker
```
# Part 2: Doing docker pull for images and running it as a container

```bash
docker pull naveensani22/ros2_jazzy_turtlebot:1.0.1
```
After this, the image that is pulled can be seen using:

```bash
docker images
```
# Running Instructions:

To Run this image as a container (Please use the name of the container that you want after --name and use the name of image pulled. And give all the devices,display and permissions as required):

```bash

docker run -it \
  --name ros2_jazzy_turtle_docker_pull \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:1.0.1

```

After this command, you should see the terminal of the container that is running now. And to exit the container use:

```bash
exit
```

And to later use this same container, use (use the name of the container that you named):

```bash
docker start -ai ros2_jazzy_turtle_docker_pull
```

# Part 3: Updating the docker image once we do modifications:

There are two ways of updating the docker image after we do modifications to it. So after we run a docker image as a container and make changes in that container, we can either commit it/save it as a new image OR we can update the earlier Dockerfile with the commands that we ran on the container to make modifications and save the new dockerfile. Then we can build the new image using it with "docker build" command.

# 1st Option: Commit as an Image

```bash
docker commit Container_Name Preferred_Image_Name
```
# 2nd Option: Updating the Dockerfile and then building the image (Recommended)

Update the Dockerfile and save it. And then go *into the directory with the Dockerfile* and do:

```bash
docker build -t DockerhubUsername/ros2_jazzy_turtlebot:1.0.0 .
```
