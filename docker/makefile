build:
	docker build -f Dockerfile.lidar . -t swarmslam

cpu_start:
	docker run -itd --rm --ipc host --net host --pid host --name swarmslam swarmslam

gpu_start:
	docker run -itd --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

agx_start:
	docker run -itd --runtime nvidia --gpus all -v /dev:/dev -e NVIDIA_DRIVER_CAPABILITIES=all --rm --ipc host --net host --pid host --name swarmslam swarmslam

stop:
	docker kill swarmslam

attach:
	docker exec -it swarmslam bash