# PILLAR Project: Cognitive Architecture

## Build
```bash
colcon build
```

## Usage

### Terminal 1

Source the local setup files and launch the core part of the system.

```bash
source install/setup.bash
ros2 launch mdb_launch.py
```

### Terminal 2

Source the local setup files and launch an experiment.

```bash
source install/setup.bash
ros2 service call /commander/load_config core_interfaces/srv/LoadConfig "{file: '/path/to/experiment.yaml'}"
```