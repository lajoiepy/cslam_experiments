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

lidar_cslam:
	docker exec -it swarmslam bash -c "source /opt/ros/humble/setup.bash; source Swarm-SLAM/install/setup.bash; ros2 launch cslam_experiments experiment_lidar.launch.py robot_id:=$(ROBOT_ID)"
