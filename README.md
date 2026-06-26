# antsy_simulation

Simulation package for ANTSY. The main runtime node is the MuJoCo simulator wrapper in `scripts/mujoco_simulator.py`.

## MuJoCo

Launch the full simulation stack with:

```bash
ros2 launch antsy_simulation simulator.launch.xml
```

By default this launches:

- `mujoco_simulator`
- `antsy_description/description.launch.py`
- `antsy_control/follow_velocity_rectangle.launch.xml`

Relevant topics:

- `actuators`: controller output consumed by the simulator
- `joint_states`: measured joint positions and velocities from MuJoCo
- `odom`: simulated body odometry
- `clock`: simulation clock

The default MuJoCo model uses contact friction `1.2 0.10 0.001`
for sliding, torsional, and rolling friction. The torsional term is
intentionally higher than MuJoCo's very low default-like value used in
early tests, because the spherical feet otherwise yaw-slip during straight
walking and make simulator `/odom` drift laterally relative to leg odometry.

## Node

### `mujoco_simulator`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_path` | `share/antsy_simulation/mujoco/antsy.xml` | MuJoCo XML model to load. |
| `run_headless` | `True` | If true, do not launch the interactive MuJoCo viewer. |
| `realtime_factor` | `1.0` | Wall-time scaling applied to the simulation timer. |
| `state_publish_rate` | `50.0` | Frequency in Hz for `joint_states` and `odom` publication. |

## Launch file

### `simulator.launch.xml`

Arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `run_headless` | `false` | Passed to `mujoco_simulator.run_headless`. |
| `publish_description` | `true` | If true, include the description launch. |
| `start_controller` | `true` | If true, include the control launch. |
| `use_sim_time` | `true` | Passed into the included description and control launches. |
| `model_path` | `$(find-pkg-share antsy_simulation)/mujoco/antsy.xml` | Passed to `mujoco_simulator.model_path`. |
| `realtime_factor` | `1.0` | Passed to `mujoco_simulator.realtime_factor`. |
| `state_publish_rate` | `50.0` | Passed to `mujoco_simulator.state_publish_rate`. |
