# basic_offboard_px4

![CI Build](https://github.com/hugo2410/basic_offboard_px4/actions/workflows/docker-build.yml/badge.svg)
[![GHCR Latest Image](https://img.shields.io/github/v/tag/hugo2410/basic_offboard_px4?label=GHCR%20Latest&logo=docker)](https://github.com/hugo2410/packages/container/basic_offboard_px4)


A Drone Simulation Environment based on PX4, MAVROS, ROS 2, Gazebo, and Docker

---

## Overview

basic_offboard_px4 is a project that simulates a drone flying through a series of waypoints in offboard mode. The simulation integrates the following key components:

- **PX4:** A robust autopilot software for drones.
- **MAVROS:** A MAVLink extendable communication node for ROS 2.
- **ROS 2:** The Robot Operating System framework facilitating communication and control.
- **Gazebo:** A 3D simulation environment that visualizes the drone's flight and environment.

Everything is containerized using Docker, which makes it easy to set up, run, and maintain a consistent simulation environment without worrying about local dependencies.

---

## Features

- **Modular Simulation:** Combines PX4, MAVROS, ROS 2, and Gazebo for a comprehensive offboard flight simulation.
- **Dockerized Environment:** All dependencies are encapsulated within a Docker container to ensure reproducible and isolated builds.
- **Automated Workflows:** GitHub Actions workflows automatically build and push the Docker image, and execute the build process within the container.
- **Commit Hook:** An integrated commit hook ensures code quality and triggers the CI/CD pipelines to keep your simulation environment up to date.
- **Efficient Build Management:** The workflows are set up to cancel previous runs when a new push is made, ensuring that only the latest changes are being processed.

---

## Prerequisites

- **Docker:** Ensure you have Docker installed on your system.
  You can download and install Docker from the official installation guide:
  [Get Docker](https://docs.docker.com/get-docker/)

- **Git:** To clone the repository and work with the commit hooks.

---

## Getting Started

### Building and Running the Docker Container

1. **Clone the Repository**

   ```bash
   git clone https://github.com/your-username/basic_offboard_px4.git
   cd basic_offboard_px4
   ```

2. **Build the Docker Image**

   Build the image using the Dockerfile provided in the repository:

   ```bash
   docker build -t my-docker-image .
   ```

   Replace `my-docker-image` with your desired image name.

3. **Push the Docker Image (Optional)**

   If you want to push the Docker image to a registry, use:

   ```bash
   docker push my-docker-image
   ```

   Make sure you’re logged in to your registry before pushing.

4. **Run the Docker Container**

   Execute the container and start the simulation. For example, to run the container and automatically execute the simulation with colcon build, run:

   ```bash
   docker run --rm -w /root/ros_ws my-docker-image colcon build
   ```

   Adjust the working directory path (`/root/ros_ws`) if your configuration differs.

---

## Commit Hook and CI/CD Workflows

This repository includes a smart commit hook that automatically performs quality checks and triggers the GitHub Actions workflows upon every commit. These workflows handle:

- **Docker Image Build & Push:**
  The image is built based on your code changes and pushed to the GitHub Container Registry (or your preferred registry). The workflows are configured to use an intelligent caching mechanism so that subsequent builds are faster.

- **Automated Simulation Build:**
  Once the Docker image is updated, the workflow also executes `colcon build` inside the container to compile your simulation packages, ensuring that the latest changes are always tested.

- **Efficient Run Management:**
  The workflows are set up with concurrency rules that cancel any queued or running jobs when a new push is made. This ensures that only the most recent commit is processed, keeping your CI/CD pipeline efficient and up-to-date.

---

## Contributing

Contributions are welcome! If you’d like to add features or fix issues, please fork the repository and submit a pull request. Make sure to follow the commit message guidelines and coding standards outlined in the commit hook.

---

## License

[MIT License](LICENSE)

---

Feel free to reach out with any questions or suggestions. Enjoy simulating drone flights with basic_offboard_px4!
