ROBOT_ID=0

build:
	docker build -f Dockerfile.lidar . -t swarmslam

cpu_run:
	docker run -itd --rm --ipc host --net host --pid host --name swarmslam swarmslam

gpu_run:
	docker run -itd --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

agx_run:
	docker run -itd --runtime nvidia --gpus all -v /dev:/dev -v $(PWD)/../:/Swarm-SLAM/src/cslam_experiments -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

start:
	docker start swarmslam

stop:
	docker stop swarmslam

kill:
	docker kill swarmslam

attach:
	docker exec -it swarmslam bash

swarmslam-lidar:
	docker exec -it swarmslam bash -c "source /opt/ros/humble/setup.bash; source Swarm-SLAM/install/setup.bash; export ROS_DOMAIN_ID=0; ros2 launch /Swarm-SLAM/src/cslam_experiments/launch/robot_experiments/experiment_lidar.launch.py robot_id:=$(ROBOT_ID)"

bridge:
	bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics"

relay:
	docker exec -it swarmslam bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; ros2 run topic_tools relay /os_cloud_node/points /r0/pointcloud"
