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

************

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

************

# Part 3: Updating the docker image for the USER (fixed commands):

For example, let’s take the name of the container running our image as “turtlebot_container”.

1. First we pull the latest stable image:

```bash
docker pull naveensani22/ros2_jazzy_turtlebot:stable
```
2. Then, we have to remove the current container:

```bash
docker rm turtlebot_container
```
3. Then run the new image as container:

```bash
docker run -it \
  --name turtlebot_container\
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:stable

```
This opens a new container with the same name of the old container. But this loses all the changes that the user made in the old container.

************

# Documentation of how to do all of things, that we figured out while trying to do things (Refer Part 14 to get recommended method)

# Part 4: This is how we created the docker container ros2_jazzy_turtle (Initially)

To Allow X11/Display access:

```bash
xhost +local:docker
```
Running the Container:

```bash
docker run -it \
  --name ros2_jazzy_turtlebot \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  ros:jazzy bash

```
To see the images in the system:

```bash
docker images
```
To see the running containers in the system:
```bash
docker ps
```
To see the running containers in the system:
```bash
docker ps -a
```

# Part 5: This is how we used VS code with the Docker Container
1. Install VS Code on your host.
2. Install the extension: Remote - Containers (Dev Containers).
3. Starting the container
```bash
xhost +local:docker
docker start -ai ros2_jazzy_turtlebot
```
4. In VS Code → Press F1 → search: “Remote-Containers: Attach to Running Container”.
5. Select ros2_jazzy_turtlebot
6. To manually stop: docker stop ros2_jazzy_turtlebot

# Part 6: Setting up the ROS2 workspace and the TurtleBot4 Simulator package
```bash
apt update
apt upgrade

# Go to home directory
cd ~

# Make workspace and src folder
mkdir -p ~/turtlebot4_ws/src

# Go into the src folder
cd ~/turtlebot4_ws/src

git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy

Install dependencies:
cd ~/turtlebot4_ws
rosdep update
rosdep install --from-path src -yi

Build the packages:
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

source ~/turtlebot4_ws/install/setup.bash
```

# Part 7: Running a simulation using gazebo from inside the container

```bash
# Launch Gazebo
ros2 launch turtlebot4_gz_bringup sim.launch.py

```

IF DISPLAY IS FAILING:

```bash
# Inside the container
export DISPLAY=:1 [0 or 1, has to be same as host display number]

# Verify
echo $DISPLAY  # should print :1

# Launch Gazebo
ros2 launch turtlebot4_gz_bringup sim.launch.py

```

# Part 8: To automatically Start a Docker container when the system is turned on

```bash
docker update --restart always ros2_jazzy_turtlebot

#OR This (below command) can be used to avoid the launch file to starting again after the container stops (not for manual stopping)

docker update --restart unless-stopped ros2_jazzy_turtlebot
```

# Part 9: To Run a Launch File when the Docker Container starts running

Inside Container Terminal

```bash
cd /
nano ros_entrypoint.sh

#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/root/turtlebot4_ws/install/setup.bash" --

export DISPLAY=:0 
exec ros2 launch turtlebot4_gz_bringup sim.launch.py
exec "$@"

```
After stopping the container:

```bash
xhost +local:docker 
docker start -ai ros2_jazzy_turtlebot
```
In Another Terminal to get terminal of container:

```bash
docker exec -it ros2_jazzy_turtlebot bash
```

# Part 10: To Close a Docker Container

In host terminal:

```bash
docker stop ros2_jazzy_turtlebot
```

# Part 11: To Close the system

In host terminal:

```bash
systemctl poweroff
```

# Part 12: To access camera from inside docker container

To find the name of the connected device:

In host terminal:

```bash
ls /dev > before.txt

# plug in USB

ls /dev > after.txt
diff before.txt after.txt
```
Use the name here (instead of /dev/video0):

In host terminal:

```bash
xhost +local:docker

docker run -it \
  --name ros2_jazzy_device_test \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --device /dev/video0 \
  ros:jazzy bash

```

In container terminal:

```bash
apt-get update
apt-get install -y ffmpeg
ffplay -f v4l2 -i /dev/video0  #to see live camera feed for us  

```

# Part 13: To access USB data from inside docker container

In host terminal:

```bash
docker run -it --rm \
  --device=/dev/sdb1 \
  ubuntu bash

```

In Container Terminal (find the exact name of the usb using the above method used for the camera):

```bash
apt-get update
apt-get install -y mount util-linux   [tools for mounting]
mkdir /mnt/usb
mount /dev/sdb1 /mnt/usb
ls /mnt/usb   [to view the data]

```
# Part 14: To Build a Docker Image with Dockfile and other files (Recommended)

First write a Dockerfile like this as an example inside a folder:-

The Dockerfile:

```bash
# 1. Base image
FROM ros:jazzy

# 2. Install system dependencies and tools
RUN apt update && \
    apt upgrade -y

# 4. Create and initialize TurtleBot4 workspace
RUN mkdir -p /root/turtlebot4_ws/src
WORKDIR /root/turtlebot4_ws/src

# 5. Clone the TurtleBot4 simulator repo
RUN git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy

# 6. Install ROS 2 package dependencies
WORKDIR /root/turtlebot4_ws
RUN rosdep update && \
    rosdep install --from-path src -yi

# 7. Build the workspace
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# 10. Set working directory and default entrypoint
WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot4_gz_bringup", "sim.launch.py"]

```
And add other files in the same folder that are used in the Dockerfile (like maybe a ros_entrypoint.sh):

```bash
#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/turtlebot4_ws/install/setup.bash"

if [ "$#" -gt 0 ]; then
    "$@" &
fi

exec /bin/bash

```

And then go to that folder and do (the dockerhub username is preferred to be used, if planning to push to dockerhub):

```bash
docker build -t dockerhubusername/ros2_jazzy_turtlebot:1.0.0 .  [the dot at the end is required]

```

To Run a container using this image:

```bash
docker run -it \
  --name ros2_jazzy_turtle_docker \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  dockerhubusername/ros2_jazzy_turtlebot:1.0.0

```
# Part 15: To Push and Version a Image to Dockerhub and to Pull:

To Push:

```bash
docker login
docker push dockerhubusername/ros2_jazzy_turtlebot:1.0.0  [here use your dockerhub username instead of naveensani22]

```
```bash
docker push naveensani22/ros2_jazzy_turtlebot:1.0.1

```

To Pull:

```bash
docker pull naveensani22/ros2_jazzy_turtlebot:1.0.0

```
# Part 16: To Version and Push the files in the folder to Github:

```bash
git add .
git commit -m "Comments Required"
git tag -a v1.0.0 -m "Version 1.0.0: With Comments"
git push origin main --tags

```
# Part 17: To manually update the image and run it when an updated image is added in Dockerhub:


For example, let’s take the name of the container running our image as “turtlebot_container”.

First we pull the latest stable image:

```bash
docker pull naveensani22/ros2_jazzy_turtlebot:stable
```
Then, we have to remove the current container

```bash
docker rm turtlebot_container
```
Then run the new image as container:

```bash
docker run -it \
  --name turtlebot_container\
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:stable
```
This opens a new container with the same name of the old container.

# Part 18: Updating the docker image once we do modifications:

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
