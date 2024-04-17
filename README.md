## HOW TO USE TIDYBOT

1. Open the project in its devContainer.

2. Open noVNC and type in the following command in the terminal "ros2 launch uol_tidybot tidybot.launch.py" (The robot does not have the functionality required to successfully navigate the higher level environments)
   
3. run the following command to spawn in cubes for the robot to push "ros2 run  uol_tidybot generate_objects --ros-args -p n_objects:=10"

4. Run the tidy scene program using the command: "ros2 run sm_tidyBot tidy_scene"


