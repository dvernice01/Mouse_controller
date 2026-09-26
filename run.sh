xhost +
docker run -it --rm --net host --ipc host --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -e GZ_SIM_RESOURCE_PATH=/root/ros_workspace/install/ros2_control_demo_description/share \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v ./my_workspace/:/root/ros_workspace \
    --name control_demos \
    ros2_control_demos bash -c "cd /root/ros_workspace && exec bash"