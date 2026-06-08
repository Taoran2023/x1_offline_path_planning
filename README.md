# Offline Mapping & Path Planning Workspace

ROS2 (Humble) packages for offline map processing, low-resolution topological
mapping, and multi-agent path planning over a pre-built point-cloud map.

## Packages

- `block_map` — voxel/block occupancy & traversability map
- `lowres_map` — low-resolution topological graph built on top of the block map
- `frontier_grid` — frontier extraction grid
- `path_planning` — tree-search planning, path stitching/pruning, visualization
- `stub_exp_comm_msgs`, `stub_swarm_data` — message stubs so the workspace
  builds standalone without external swarm/comm dependencies

## Clone & Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url> .

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Always build with `--symlink-install` — config/launch file edits then take
> effect without rebuilding.

## Run

The map (`.pcd`) is distributed separately (not in this repo). Place it
somewhere accessible and pass its path as a launch argument:

```bash
ros2 launch path_planning test_map.launch.py pcd:=/path/to/your_map.pcd
```

This launches the `test_map` node (loads the map, builds the low-res topo
graph, runs the tree-search planner) plus RViz with the bundled config.

Planner parameters live in `path_planning/config/`:
- `caltech_slam.yaml` (default, see `test_map.launch.py`)
- `mapping.yaml`

Switch between them by editing `CONFIG` in
`path_planning/launch/test_map.launch.py`.

---

*More detailed docs (architecture, parameter reference, workflow) to follow.*
