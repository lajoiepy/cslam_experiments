## Experiments
This repo contains Swarm-SLAM launch examples.

[Datasets launch files](https://github.com/lajoiepy/cslam_experiments/tree/main/launch/datasets_experiments)

[Real setups launch files](https://github.com/lajoiepy/cslam_experiments/tree/main/launch/robot_experiments)

[Configurations](https://github.com/lajoiepy/cslam_experiments/tree/main/config)

Important note: If you want to use the datasets launch files as is, add a symlink to your datasets ROS 2 bags in the `data/` folder and modify the [bag path](https://github.com/lajoiepy/cslam_experiments/blob/629a597e90a56a9fe020633fe7c7c10fab0f8f8b/launch/datasets_experiments/graco_lidar.launch.py#L82) accordingly. As an example, currently the `graco_lidar` launch files expect the first three sequences from the [GRACO dataset](https://sites.google.com/view/graco-dataset/download) to be under `data/Graco_Ground/Graco-0`, `data/Graco_Ground/Graco-1`, and `data/Graco_Ground/Graco-2`.
