# antsy_simulation
simulation package for ANTSY

## MuJoCo

Launch the simulation with `ros2 launch antsy_simulation simulator.launch.xml`
This launches the MuJoCo simulator, publishes simulation time, joint states and odometry, and starts the gait controller by default.

### Relevant ROS2 topics:
- /actuators -> we publish our actuator data here so our pet moves in the simulator
- /joint_states -> position of every joint in the simulator
- /odom -> pretty self-explanatory, right?
- /clock -> simulation time. Do not confuse with your local time ;)
