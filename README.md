# How to run the code.

Clone the repo.

- rosrun sphero_stage_2 start.py. (To start the simulation)

Anyone of the following (Run the main algorithm):

- rosrun sphero_stage_2 main_2.py (For Rendezvous Protocol)

- rosrun sphero_stage_2 main_2_ob_scr.py (For line and triangle config)

- rosrun sphero_stage_2 main_2_ob_scr_square.py (For square config)


- rosrun sphero_stage_2 main_2_ob_scr_square_switching.py (For square config)

(optional)

- rosrun rqt_reconfigure rqt_reconfigure (For dynamically changing the parameter values)

use 2D nav on top of rviz to give the goal point in the map. (To see the navigation behavior). 

You should see all the robots approaching the same goal and once they observe an obstacle, then navigate randomly.


