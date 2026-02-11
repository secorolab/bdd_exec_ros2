# bdd-exec-ros2

## Dependencies

- Python packages:
  + [rdf-utils](https://github.com/minhnh/rdf-utils)
  + [bdd-dsl](https://github.com/minhnh/bdd-dsl)
  + [coord-dsl](https://github.com/secorolab/coord-dsl)
- ROS packages:
  + [minhnh/bdd_ros2_interfaces](https://github.com/minhnh/bdd_ros2_interfaces)

## Nodes

### Mockup behaviour

The [FSM](./models/pickplace.fsm) for the pick-place behaviour generates to the [Python implementation](./bdd_exec_ros2/behaviours/fsm_pickplace.py)
with [coord-dsl](https://github.com/secorolab/coord-dsl).
The package is also required to execute the mockup behaviour node.

## Virtual environment setup with ROS2

If you want to setup a [ROS2 Python virtual environment](https://docs.ros.org/en/rolling/How-To-Guides/Using-Python-Packages.html),
you'd need to allow using the ROS2 Python packages in the environment, e.g. with
[`uv`](https://docs.astral.sh/uv) in `zsh`:

```sh
source /opt/ros/rolling/setup.zsh
cd $ROS_WS_HOME  # where the 'src' folder is located
uv venv --system-site-packages venv
touch ./venv/COLCON_IGNORE
colcon build
```

Additionally you'd need to add the following to `setup.cfg`, if you specify `entry_points` in `setup.py`:

```ini
[build_scripts]
executable = /usr/bin/env python3
```

Now you can activate both environments with:

```sh
source "$ROS_WS_HOME/venv/bin/activate"
source "$ROS_WS_HOME/install/setup.zsh"
```
