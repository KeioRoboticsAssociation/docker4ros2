# Docker4ROS2

This repository provides a minimal example for running ROS 2 Humble in Docker.
It includes a simple Python package located in the `src/hello_py` directory.

## Requirements
- Docker (tested with Docker 20.10 or later)
- git

## Quick Start
1. Install Docker following the [official instructions](https://docs.docker.com/get-started/).
2. Clone this repository:
   ```bash
   git clone <repo-url>
   cd docker4ros2
   ```
3. Start the development container:
   ```bash
   ./run_docker.sh
   ```
4. Inside the container build the workspace and source the setup files:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```
5. Run the example node:
   ```bash
   ros2 run hello_py hello
   ```

## Notes
- The `src/hello_py` package is a simple example that prints `Hello, ROS2!` every second.
- If you wish to create additional packages, add them inside the `src` directory and rebuild with `colcon build`.
- ROS 2 Humble packages are only provided for Ubuntu 22.04 (Jammy). If you are on
  Ubuntu 24.04 or another release, `apt-get install ros-humble-*` will fail. In
  that case use the Docker instructions above or install ROS 2 from source on a
  supported distribution.

### Using Docker Compose
If you prefer `docker compose`, a preconfigured setup is provided in
`.devcontainer/docker-compose.yml`:

```bash
docker compose -f .devcontainer/docker-compose.yml build
docker compose -f .devcontainer/docker-compose.yml run ros2-dev
```

This launches the same development environment as `run_docker.sh` and mounts
the repository in `/home/ros/ros2_ws` inside the container.

