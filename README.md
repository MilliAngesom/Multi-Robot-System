# How to run the code (Specific to one robot)

New Package name: sphero_stage_2.

## Important Detials !!
No modification needed in "rviz_utils" folder.

Only use "utils_2" folder to add utility modules if needed.
1. vector.py is a file that represents a vector and has usable methods to facilitate smooth functioning of boids. (To understand the code, uncomment the code at the end of the file and run that file.)
2. boid_2.py file represents boid class. It has to be used to create every robot instance and contains methods that are common to all robots like navigation, etc.

Use main_2.py for running the main algorithm. (Handles all the boids in one file)


## Now, you are ready to test the code!
- rosrun sphero_stage_2 start.py. (To start the simulation)
- rosrun sphero_stage_2 main_2.py (Run the main algorithm)

use 2D nav on top of rviz to give the goal point in the map. (To see the navigation behavior). 

You should see all the robots approaching the same goal.
