ROBOT_ID=0

build:
	docker build -f Dockerfile.lidar . -t swarmslam

cpu_run:
	docker run -itd --rm --ipc host --net host --pid host --name swarmslam swarmslam

gpu_run:
	docker run -itd --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

agx_run:
	docker run -itd --runtime nvidia --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

start:
	docker start swarmslam

stop:
	docker stop swarmslam

kill:
	docker kill swarmslam

attach:
	docker exec -it swarmslam bash

zenoh_install:
	docker exec -it swarmslam bash -c "echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && apt-get update && apt-get install -y zenoh-bridge-ros2dds"

lidar_cslam:
	docker exec -it swarmslam bash -c "ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=$(ROBOT_ID)"
