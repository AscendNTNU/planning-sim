FROM ascendntnu/ros-kinetic:1.0

RUN mkdir -p /catkin_ws/src/planning-sim/
COPY . /catkin_ws/src/planning-sim/
RUN /ros_entrypoint.sh bash -c "cd /catkin_ws && catkin_make"
