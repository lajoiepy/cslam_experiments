build:
	docker build -f Dockerfile . -t bridge
run:
	docker run --net=host -t --rm --name bridge bridge "127.0.0.1"
relay:
	docker exec -t bridge bash -c "source /opt/ros/foxy/setup.bash; ros2 run topic_tools relay /os_cloud_node/points /r0/pointcloud" 
