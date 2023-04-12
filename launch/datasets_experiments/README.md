### Launch parameters
- `max_nb_robots` : Maximum number of robots in the swarm (needs to be greater or equal to the number of robots your are using)
- `robot_delay_s` : Delay between launching each robot. Ajust depending on the computing power of your machine.
- `launch_delay_s` : Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames.
- `config_file` : Configuration to use from [this folder](https://github.com/lajoiepy/cslam_experiments/tree/main/config), see the [parameters description](https://lajoiepy.github.io/cslam_documentation/html/md__home_lajoiepy__documents_phd_projects__c-_s_l_a_m_cslam_config_cslam__r_e_a_d_m_e.html)
- `rate` : rate at which to play the bag
- `enable_simulated_rendezvous` : enables a schedule of availability, useful to test the neighbors manager.
- `rendezvous_config` : Rendezvous config file. Format `ROBOT_ID,RDV1_START_SECOND,RDV1_END_SECOND,RDV2_START_SECOND,...`
