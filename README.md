# antsy_simulation
simulation package for ANTSY

## WIP - for now, it only supports Gazebo. It will support IsaacSim

Launch the simulation with `ros2 launch antsy_simulation simulator.launch.xml`
This will launch the Gazebo simulator, create the ROS-Gz bridge and configure the relevant topics. Once it receives a robot description, it will spawn our Yetti!

### Relevant ROS2 topics:
- /actuators -> we publish our actuator data here so our pet moves in the simulator
- /joint_states -> position of every joint in the simulator
- /odom -> pretty self-explanatory, right?
- /imu -> it currently does not work
- /clock -> simulation time. Do not confuse with your local time ;) 