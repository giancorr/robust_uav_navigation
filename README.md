# Container Structure

The Docker container is configured with the following structure:

Docker Container    
│ ├── /root
    │ └── Micro-XRCE-DDS-Agent (Inside the container)
    │ └── PX4 Firmware (Shared via Docker volume, exists outside the container)
    │ └── ~/ros2_ws/src
            │ ├── content of the `ros2_ws-src` (Source code for the ROS2 workspace) 
            │ ├── px4-ros-com (Pre-loaded in the container) 
            │ └── px4-msgs (Pre-loaded in the container) 
               

## Details

- **Auto-deletion:** After its execution, the Docker container is automatically deleted. This helps in maintaining a clean environment by freeing up resources on the host machine.

- **`ros2_ws-src` folder:** This folder should contain the source code for the ROS2 workspace. The Docker container comes pre-loaded with `px4-ros-com` and `px4-msgs`, which are dependencies for the PX4-ROS2 communication.

- **PX4 Firmware:** The PX4 Firmware is not included in the Docker container. Instead, its folder is shared with the container using a Docker volume. This allows the firmware to be updated without having to rebuild the Docker container.


### Prerequisites

What things you need to install the software and how to install them:

- Docker 

### Installing

A step by step series of examples that tell you how to get a development environment running:

1. Clone the repository to your local machine.
3. Clone the PX4 Firmware with `git clone --single-branch -b release/1.14 git@github.com:PX4/PX4-Autopilot.git --recursive`
4. Build the docker imagege with `cd docker && docker build -t leo-img -f px4_humble_dockerfile.txt .`
5. Run the container with `./run_cnt.sh`.

## Running the tests

Explain how to run the automated tests for this system.

## Deployment

Add additional notes about how to deploy this on a live system.

## Built With

* [PX4](https://px4.io/) - The flight stack used
* [ROS](https://www.ros.org/) - Robot Operating System

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
