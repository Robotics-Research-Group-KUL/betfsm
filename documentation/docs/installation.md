# Installation


## Installation using pip



Create an environment (or use once of your already existing python environments). It is not recommended
installing directly, without pip environment, on the host.

```
    python -m venv my_env
    source my_env/bin/activate
```

When you want to use BeTFSM in a pure python environment, you can install it using pip:

```
    pip install "git+https://github.com/Robotics-Research-Group-KUL/betfsm.git#egg=betfsm&subdirectory=betfsm"
```

After this installation, your can run one of the examples (make sure that you have activated your environment):

```
    gui_example_large
```

You can look at the available examples in:
```
my_env/lib/python3.12/site-packages/betfsm_examples/
```
These examples can be started using the associated scripts in `my_env/bin` or directly from the above
directory (in that case, make sure that the environment is activated)

If you manage your python project using `pyproject.toml` (with standard setuptools), you can add a dependency on BeTFSM using:
```
dependencies = [
    "betfsm @ git+https://github.com/Robotics-Research-Group-KUL/betfsm.git#subdirectory=betfsm"
]
```

## Installation in ROS2

You can add BeTFSM to your workspace by cloning the reposity in the `src` directory of your workspace:

```
cd ./src
git clone https://github.com/Robotics-Research-Group-KUL/betfsm.git

```

You can build using the usual ROS2 approach (for an editable install):
```
cd your-workspace
colcon-build --symlink-install
```

The examples are installed and can be run using ROS2:
```
ros2 run betfsm gui_example_large
```
The examples can also be run directly from their source directory:
```
your-workspace/src/betfsm/betfsm/betfsm_examples
```

## installation in ROS2 together with Crospi

See [the cROSpi website](https://rob.pages.gitlab.kuleuven.be/crospi/)