build:
	docker build -f Dockerfile.lidar . -t swarmslam

nvidia_build:
	docker build -f Dockerfile.lidar.nvidia . -t swarmslam

cpu_start:
	docker run -itd --cpuset-cpus="2,6,10,14" --rm --ipc host --net host --pid host --name swarmslam swarmslam

gpu_start:
	docker run -itd --cpuset-cpus="2,6,10,14" --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

stop:
	docker kill swarmslam

attach:
	docker exec -it swarmslam bash