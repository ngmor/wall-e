# wall-e-xcompile

A Raspberry Pi ROS 2 cross compilation docker container adapted from [here](https://github.com/m-elwin/rosberrypi)

# Build

```bash
# Install dependencies
sudo apt install qemu-user-static binfmt-support docker.io

# Explicitly pull arm64 ROS 2 Humble docker image
docker pull --platform linux/arm64 arm64v8/ros:humble

# Build cross compilation docker (from this directory)
docker build --tag wall-e-pi-xcompile .
```

# Usage
```bash
# Assuming ROS workspace structure of ${ws}/src/wall-e
cd ${ws}

# Use docker to cross compile (change out 'colcon-aarch64' for another command
./src/wall-e/docker/wall-e-pi-xcompile/run colcon-aarch64 ${build-args}

# OR

# Use docker to cross compile (runs the above command)
./src/wall-e/docker/wall-e-pi-xcompile/xcompile ${build-args}

# Could also add an alias to .bashrc, something like:
alias colcon-wall-e-xcompile='docker run -it -u $(id -u):$(id -g) -v "$(pwd)":/ros_ws "wall-e-pi-xcompile" colcon-aarch64'
```