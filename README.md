# Micromouse-simulation
A virtual simulation of the Micromouse competition focused on the decision making process of the robot
Capstone project for the Machine Learning Engineering from Udacity

Startercode provided by Udacity, which includes the tester.py, maze.py and showmaze.py

Code explanation:

>Robot.py

-Main file which contains the exploration part of this project. In it has some paratemeters
that can be changed, such as if the simulation will be rendered or not, if you want verbose
and debug to be outputed and if a sound beep should be played when robot reaches the goal.

-The rendered simulation can be closed by pressing the "ESC" key and can be paused py pressing
the "p" key.

-When testing the custom maze, the maze number 4, the "customMaze" variable must be set to True
so that the program will know change the background image to that maze.

-The program will output a log file for the simulation that has to be in a folder named "logs".

-The images required for the simulation are included.

>Astar.py

A* algorithm. It will receive the explored path by the robot and will output the best path to
the robot in the second run. It has a default variable of verbose set to False that can be chan-
ged when testing.


