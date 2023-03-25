# edu_perception
ROS2 software for perception applications.

# Deploying

A Docker container is used to run this software on our robots. Normally, all our robots are shipped with a Docker container registered to start after a reboot. If you want to deploy a newer version, or whatever the reason, make sure to remove the previously deployed container. To check which containers are running, use the following command:

## Deploying on IPC127e

This section describes how the software is deployed on an IPC127e in a Docker environment. First clone the repository on the robot by executing this command:

```bash
git clone https://github.com/EduArt-Robotik/edu_perception.git
```

Then navigate into the docker folder in the cloned repository:

```bash
cd edu_perception/docker/ipc127e
```

Using make the Docker image can be build:

```bash
make all
make clean
```

Note: this could take some while, ~20min.

After executing these command a new Docker image with the name "eduart-perception-ipc127e:0.1.0" should be created. It can be checked by following command:

```bash
docker image ls
```

The docker container can easily started by the command:

```bash
docker run --name eduard-perception-ipc127e-0.1.0 --restart=always --privileged -v /dev:/dev --network host --pid=host --group-add dialout --env EDU_ROBOT_NAMESPACE=/eduard/green eduart-perception-ipc127e:0.1.0
```

With the flag "--restart=always" the container will come up after rebooting the system. If this is not wanted please remove this flag. The environment variable EDU_ROBOT_NAMESPACE is useful when multiple robots are in the some network. We recommend to adapt the namespace according the robot color. Here in this case green is used.
