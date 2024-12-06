Template Matching Algorithm by Thuan Nguyen, Adam Bukres, and Nathan Woo

The initial approach without the use of a gas tank template is found within the main branch of this repository.

Our final and most accurate approach is found in the gas-tank-template branch of this repository and reads in the gas_tank26.npy from the datasets folder.

Both approaches can be run via the command "python3 ./read_pcd.py" and will read from the provided ROS2 publisher.

Alorithms folder contains the python files of the algorithms we used. 
Datasets folder contains the real world data we tested on. We have extra .npz files in here that we generated from the bag files to make testing our algorithms easier. 
Startup folder contains a script that is ran on startup.
Tests folder contains our generated dataset.

Container can be started by running the run.sh script. Once run, there is a glitch that requires uninstalling numpy and reinstalling version 1.24.0.

Sample run: 

`ros2 bag play datasets/rosbag2_2024_06_26_21_42_35-automated_refuel_session/rosbag2_2024_06_26_21_42_35-automated_refuel_session_0.mcap -l`
`rviz`
`python3 read_pcd.py`