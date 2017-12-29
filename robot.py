import random
import os
import winsound
import pygame, sys
from Astar import Astar

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.location = [1, 1]
        self.heading = 'North'
        self.mazeDim = maze_dim
        
        # CHANGE BELOW IF TESTING MAZE 4!
        customMaze = False
        
        # Get correct maze for rendering
        if (self.mazeDim == 12):
            self.backgroundImage = os.path.join("images","Maze1.png")
        elif(self.mazeDim == 14):
            self.backgroundImage = os.path.join("images","Maze2.png")
        elif (self.mazeDim == 16):
            self.backgroundImage = os.path.join("images","Maze3.png")
        
        if (customMaze == True):
            self.backgroundImage = os.path.join("images","Maze4.png")

                    
# Parameters to change
        
        self.renderSimulation = True                    # Render simulation?
        self.updateDelay = 0.5                          # Update delay to the render function
        # Debugging variables
        self.verbose = True                             # Verbose?
        self.debug = False                              # Debug?
        self.soundOn = False                            # Beeps when robot reaches center of maze
        
# Additional variables
        
        # A* variables
        self.nodes = dict()                             # dict with the nodes explored by the robot. Will be of type { location: {'neighbors': [(locationNeighbor1), (locationNighbor2), ...] } }
        self.startLocation = (1,1)                      # Saving start location to send to A* later
        self.endLocation = (self.mazeDim/2,self.mazeDim/2)     # The approx. end location.

        # Tremaux's algorithm variables
        self.exploratedPath = dict()                    # Will be a dict of type ; {location: timesVisited} e.g. {(1,1): 2}
        self.goingBackwards = False                     # Going backwards?

        # Exploring variables
        self.timeStep = 1                               # Current time step of the simulation.
        self.prevLocation = None                        # Previous location of the robot
        self.prevHeading = self.heading
        self.backVirtualSensor = 0                      # Virtual sensor that remembers available space backwards.
        self.prevBackVirtualSensor = 0                  # Previous value of backVirtualSensor. Only used for logging
        self.deadEnd = False                            # Dead-end?
        self.leftDeadEnd = False                        # Has the robot left a dead-end path after entering it?
        self.lastJunctionPoint = None
        self.endRun = False                             # Bool to end first run
        self.secondRun = False                          # Bool to tell the algorithm that it is on the second run.
        self.goal = False                               # Bool to tell that the robot has reached the goal in the current run
        self.goalTimeStep = 0                           # Time step when the robot has reached the goal
        self.endRunTimeStep = 0                         # Time step when the exploration run was ended
        self.secondRunPath = []                         # Path of the second run taken by the robot. For rendering purposes.
        # getting goal locations
        self.goalLocations = [[self.mazeDim/2, self.mazeDim/2], [self.mazeDim/2+1, self.mazeDim/2], [self.mazeDim/2, self.mazeDim/2 +1], [self.mazeDim/2 + 1, self.mazeDim/2 +1]]
        
        # Logging variables
        self.collisionText = None                       # Collision text to be written on log file
        
        self.simulation_filename = os.path.join("logs","simulation_log.txt")
        self.simulation_filename = open(self.simulation_filename, 'wb')

        # Initializing log files with title
        self.simulation_filename.write("########################################################\n\n               Robot Maze Simulation Log            \n                     Livio Tonini\n\n########################################################\n\n")

    def update_transform(self, rotation, movement, sensors):

        """ Function will be called to simulate the action chosen by the robot and update its sense of location and direction. """
        
        headings = ('West', 'North', 'East', 'South')
        rot = {-90:'turning left', 0:'not turning', 90:'turning right'}
        mov = 5     # How much will actually the robot move. Starts as greater than 3 to check for erros later
        
        # Updating previous variables
        self.prevBackVirtualSensor = self.backVirtualSensor                 # updating the prevBackVirtualSensor value
        self.prevLocation = self.location[:]
        self.prevHeading = self.heading
        
        # Chdcking for collisions and updates how much the robot actually moves given obstacles
        if (rotation == -90):                       # Turns left
            if (movement > 0):                          # Goes forward
                if (sensors[0] < movement):             # Hit a wall
                    mov = sensors[0]                    # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement              
            else:                                       # Goes backwards
                if (sensors[2] < abs(movement)):             # Hit a wall
                    mov = -sensors[2]                    # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement                  
                    
        elif (rotation == 90):                      # Turns Right
            if (movement > 0):                          # Goes forward
                if (sensors[2] < movement):             # Hit a wall
                    mov = sensors[2]                    # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement
            else:                                       # Goes backwards
                if (sensors[0] < abs(movement)):             # Hit a wall
                    mov = -sensors[0]                    # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement
                    
        else:                                       # Doesn't turn
            if (movement > 0):                          # Goes forward
                if (sensors[1] < movement):             # Hit a wall
                    mov = sensors[1]                    # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement
            else:                                       # Goes backwards
                if (self.backVirtualSensor < abs(movement)): # Hit a wall
                    mov = -self.backVirtualSensor        # Moves the available space in the given direction which is less than the intended ammout
                    self.collisionText = ("\nHit a wall. Was able to move {} squares after {} and trying to move {} squares".format(mov, rot[rotation], movement))
                    if (self.verbose):
                        print self.collisionText
                else:                                   # Can move the desired ammount in the given direction
                    self.collisionText = "\nNo collision. robot moves as intended."
                    mov = movement
         
        # Updating heading direction and backVirtualSensor
        if (rotation == -90):   # Turn left
            if (self.heading == 'West'):
                self.heading = 'South'
            else:
                self.heading = headings[headings.index(self.heading) -1]
            self.backVirtualSensor = sensors[2] + mov     # the available space in the new backwards direction is the previous available space in the previous right direction plus the moved distance in that direction
        elif (rotation == 90):  # Turn right
            if (self.heading == 'South'):
                self.heading = 'West'
            else:
                self.heading = headings[headings.index(self.heading) +1]
            self.backVirtualSensor = sensors[0] + mov     # the available space in the new backwards direction is the previous available space in the previous right direction plus the moved distance in that direction
        else:                   # Doesn't turn
            self.backVirtualSensor += mov                 # The new available space in the backwards direction is the old distance plus the distance moved in the current direction.
        
        # Updating the position. location = [x,y]
        if (self.heading == 'North'):
            # update the y coordinate
            self.location[1] += mov
        elif (self.heading == 'West'):
            # update the x coordinate
            self.location[0] -= mov
        elif (self.heading == 'East'):
            # update the x coordinate
            self.location[0] += mov
        else:   # heading = South
            # update the y coordinate
            self.location[1] -= mov

        # Check if the robot has reached the center of the maze
        if (self.location in self.goalLocations and self.goal == False and self.secondRun == False):            
            self.goalTimeStep = self.timeStep           # Getting time step that the goal was reached
            self.goal = True                            # Goal was reached.
            self.end_exploration(sensors)               # End the exploration run   
            
            if (self.soundOn):
                winsound.Beep(440,1000)                     # Goal was reached
            
        if (self.location in self.goalLocations and self.goal == False):
            self.goal = True
            if (self.renderSimulation):
                self.render_simulation(self.updateDelay, True)

    def logging(self, rotation, movement, sensors):

        """ Create and update log files to have a better understanding of each step. """

        f1 = self.simulation_filename
        
        # updating the simulation file log
        f1.write("\nTimeStep: {}".format(self.timeStep))
        f1.write("\nInitial heading: {}     Initial position: {}".format(self.prevHeading, self.prevLocation))
        f1.write("\nSensors input - Left: {}, Forward: {}, Right: {}".format(sensors[0], sensors[1], sensors[2]))
        f1.write("\nBackwards virtual sensor: {}".format(self.prevBackVirtualSensor))
        f1.write("\nRotation chosen: {}, Movement chosen: {}".format(rotation, movement))
        f1.write(self.collisionText)
        f1.write("\nFinal heading: {}      Final position: {}\n\n".format(self.heading, self.location))
 
    def render_simulation(self, update_delay=2.0, saveScreen=False):

        """ Create a visual representation of the robot in the maze. """
              
        # Icon sizes
        blueDotIconSize = redXIconSize = 10
        doubleRedXIconSize = (20,10)
        exclamationMarkIconSize = 15
        
        # Setting variables
        frame_delay = max(1, int(update_delay * 1000))                      # delay between GUI frames in ms (min: 1)
        HeadingsRotationsCorrelator = {"North":0, "East":-90, "West":90, "South":180}
        screen_size = 600                                                   #
        delta_square = screen_size/self.mazeDim                             # Pixels to move the mouse for each square in the maze
        initial_position = (delta_square/2, screen_size- (delta_square/2))  # Initial Position of the mouse
        bPause = False
        
        screen = pygame.display.set_mode((screen_size, screen_size))
        
        # Getting images
        background  = pygame.image.load(self.backgroundImage).convert()                     
        robot       = pygame.image.load(os.path.join("images","Arrow.png")).convert()                      # Image of the robot
        blueDot     = pygame.image.load(os.path.join("images","BlueDot.png")).convert()                    # Visual representation of 1 mark in a square
        redX        = pygame.image.load(os.path.join("images","RedX.png")).convert()                       # Visual representation of 2 marks in a square
        doubleRedX  = pygame.image.load(os.path.join("images","doubleRedX.png")).convert()                 # Visual representation of 3 marks in a square
        exclamationMark = pygame.image.load(os.path.join("images","exclamation.png")).convert()            # Visual representation of more than 3 marks in a square. Unwanted state

        pygame.time.wait(frame_delay)                                           # Time delay between actions

        # Calculating new player position
        x = initial_position[0] + (self.location[0]-1)*delta_square
        y = initial_position[1] - (self.location[1]-1)*delta_square
        position = (x, y)
        direction = HeadingsRotationsCorrelator[self.heading]
        robotImage = pygame.transform.rotate(robot, direction)
        
        rect = robotImage.get_rect()
        rect.center = position
                 
        # Exiting PyGame if pressing ESC
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == 27:  # Esc
                    print "\n\nSimulation closed by pressing Esc key."
                    pygame.display.quit()
                    pygame.quit()
                    sys.exit(0)
                if event.key == pygame.K_p:  # Esc
                    print "\n\nSimulation paused by pressing P key."
                    bPause = not bPause
                    
        # Pause?
        while (bPause):
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == 27:  # Esc
                        print "\n\nSimulation closed by pressing Esc key."
                        pygame.display.quit()
                        pygame.quit()
                        sys.exit(0)
                    if event.key == pygame.K_p:  # Esc
                        print "\n\nSimulation paused by pressing P key. Press it again to resume."
                        bPause = not bPause
                    
        # RENDERING
        screen.blit(background, (0,0))                                          # Rendering background                              
        
        if (not self.secondRun):
            for loc in self.exploratedPath:                                         # Rendering marks      
                x = delta_square/2 + (loc[0]-1)*delta_square                        # X center location of square
                y = screen_size - delta_square/2 - (loc[1]-1)*delta_square          # Y center location of square
                
                if (self.exploratedPath[loc] == 1 ):                                    # 1 mark => draw a blue dot
                    screen.blit(blueDot, (x-blueDotIconSize/2,y-blueDotIconSize/2) )
                elif (self.exploratedPath[loc] == 2):                                   # 2 marks => draw a red X
                    screen.blit(redX, (x-redXIconSize/2,y-redXIconSize/2) )
                elif (self.exploratedPath[loc] == 3):                                   # 3 marks => draw double red X
                    screen.blit(doubleRedX, (x-doubleRedXIconSize[0]/2,y-doubleRedXIconSize[1]/2) )
                else:                                                                   # More than 3 marks -> draw exclamation mark
                    screen.blit(exclamationMark, (x-exclamationMarkIconSize/2,y-exclamationMarkIconSize/2) )
        if (self.endRun):  
            self.pathPoints = []            
            for loc in self.secondRunPath:
                x = delta_square/2 + (loc[0]-1)*delta_square                        # X center location of square
                y = screen_size - delta_square/2 - (loc[1]-1)*delta_square          # Y center location of square
                screen.blit(blueDot, (x-blueDotIconSize/2,y-blueDotIconSize/2) )
                self.pathPoints.append((x,y))
        if (self.secondRun):
            pygame.draw.lines(screen, (0,255,0), False, self.pathPoints, 2)

        screen.blit(robotImage, rect)                                           # Rendering player
        pygame.display.flip()                                                   # Update
        
        if (saveScreen):                                                        # Saving current screen. Will be called upon reaching goal
            if (not self.secondRun):
                pygame.image.save(screen, "exploratedPath.jpeg")
            else:
                pygame.image.save(screen, "SecondRunPath.jpeg")
                
    def end_exploration(self, sensors):
        
        """ Function called to end the first run, closing the log files of the exploration and... """
    
        if (self.verbose):
            print "\nEnding first run..."
            
        # build end node    
        if (self.debug):
            print "\nAdding final node to the nodes dict..."
        neighbors = self.get_neighbors(sensors)
        # add current node in the nodes list
        self.nodes[tuple(self.location)] = {'neighbors': neighbors}
        
        # Getting the end location
        self.endLocation = self.location        # Getting the end location, the first square that the robot reached the goal
        
        # Finalizng log files and closing them
        if (self.goal):
            self.simulation_filename.write("\n\n\nGoal was reached first at the time step {}".format(self.goalTimeStep))
        else:
            self.simulation_filename.write("\n\n\nThe robot did not reach the goal.")
            
        self.simulation_filename.write("\n\nThe robot explored {0:.2f}% of the maze.".format(100.0*len(self.exploratedPath)/(self.mazeDim*self.mazeDim)))
        self.simulation_filename.close()

        # Render final step of simulation
        if (self.renderSimulation):
            self.render_simulation(self.updateDelay, True)
            
        # Getting best path from A*
        aStar = Astar(self.startLocation, self.endLocation, self.nodes)          
        self.bestActions, self.secondRunPath, self.bestPathlength = aStar.calculating_path()

        # Reset location and heading
        self.location = [1,1]
        self.prevLocation = [1,1]
        self.heading = "North"
        self.prevHeading = "North"
        self.backVirtualSensor = 0
        self.prevBackVirtualSensor = 0
        self.timeStep = 0
        
        # Set variables to end run
        self.goal = False
        self.endRun = True
        
    def get_possible_actions(self, sensors):
        
        """ Given the current location in the maze and the sensors input, will get the possible actions that won't hit a wall. """
        
        rot = [-90, 0, 90]                          # Possible rotations
            
        # Detecting Dead ends
        if (max(sensors) == 0):   # Dead end
            self.deadEnd = True
            self.simulation_filename.write("\n******Deadend found on Timestep: {}******".format(self.timeStep))
        
        # Detecting junction points
        elif ( ( (sensors[0]>0) + (sensors[1]>0) + (sensors[2]>0) + (self.backVirtualSensor>0) ) >= 3  ):
            self.lastJunctionPoint = self.location[:]
        
        # Removing rotation possibilities that doesn't make sense
        if (sensors[0] == 0):
            rot.remove(-90)         # No need to turn left if there are no space to move that way
        if (sensors[1] == 0):       
            rot.remove(0)           # No need to go forwawrd if there are no space to move that way
        if (sensors[2] == 0):       
            rot.remove(90)          # No need to turn right if there are no space to move that way
                        
        actions = []        # List of actions -> [(roration, movement), (rotation, movement)...]
            
        for r in rot:
            actions.append((r,1))   # Adding possible movements that don't go back to the actions list.
        
        if (self.backVirtualSensor >=1):
            actions.append((0,-1))  # If there is space baackwards, add the action to move backward.

        if (self.debug):
            print "\nPossible actions: {}".format(actions)
    
        return actions        

    def get_neighbors(self, sensors):
        
        """ Will get the neighbor squares of the current location that the robot can go to, not including diagonals."""
        
        rot = [-90, 0, 90, 180]                          # Possible rotations
        neighbors = []
        MovesRotCorrelator = {-90: 0, 0: 1, 90: 2}  # Correlating rotations with sensors -> -90 rotation is related to the sensor in the 0 index (sensor[0])

        # Removing rotation possibilities that doesn't make sense
        if (sensors[0] == 0):
            rot.remove(-90)         # No need to turn left if there are no space to move that way
        if (sensors[1] == 0):       
            rot.remove(0)           # No need to go forwawrd if there are no space to move that way
        if (sensors[2] == 0):       
            rot.remove(90)          # No need to turn right if there are no space to move that way
        if (self.backVirtualSensor == 0):
            rot.remove(180)
                    
        headings = ('West', 'North', 'East', 'South')
        
        for r in rot:
            # Transform mouse direction and rotation on Cardinal Directions
            if (r == -90):   # Turn left
                index = headings.index(self.heading) - 1    # Next direction on headings list
                direction = headings[index - index/4*4]     # Keep index between [-3,3]
            elif (r == 90):  # Turn right
                index = headings.index(self.heading) + 1    # Previous direction on headings list
                direction = headings[index - index/4*4]     # Keep index between [-3,3]
            elif (r == 180):
                index = headings.index(self.heading) + 2    # Opposite direction on headings list
                direction = headings[index - index/4*4]     # Keep index between [-3,3]
            elif (r == 0):
                direction = self.heading
                
            if (r == 180):
                moves = range(1, min(4, self.backVirtualSensor + 1) )
            else:
                moves = range(1, min(4, sensors[MovesRotCorrelator[r]] + 1) )
            
            for m in moves:
                loc = self.location[:]
                # Updating the position. location = [x,y]
                if (direction== 'North'):
                    # update the y coordinate
                    loc[1] += m
                elif (direction == 'West'):
                    # update the x coordinate
                    loc[0] -= m
                elif (direction == 'East'):
                    # update the x coordinate
                    loc[0] += m
                else:   # heading = South
                    # update the y coordinate
                    loc[1] -= m
                        
    #           tempNeighbor = {'location': loc}
                tempNeighbor = (loc[0], loc[1])         # tuple
                neighbors.append(tempNeighbor)
 
        return neighbors
    
    def handle_deadEnd(self, sensors): 
        
        """ This funciton will choose how to manage dead-ends. """
        
        rotation = 0
        
        # Calculate distance to closest Junction point
        deltaMovement = abs(self.location[0] - self.lastJunctionPoint[0]) + abs(self.location[1] - self.lastJunctionPoint[1])
        movement = -min(3, deltaMovement, self.backVirtualSensor)
        
        # Tests to see if it will reach the junction point
        if (abs(movement) == deltaMovement):
            self.deadEnd = False        
            self.leftDeadEnd = True
        else:
            self.goingBackwards = True              # If it didn't returned to the junction point, continue to move back
            self.leftDeadEnd = False                # If it didn't returned to the junction point, hasn't left the dead-end 
            self.deadEnd = False
            
        return rotation, movement
    
    def action_to_location(self, action):
        
        """ This function will transform actions (rotation, movement) in the locations that the robot would be in that action was
        taken. This will be used for the heuristics. """ 
                
        newLocation = self.location[:]                          # New location of the robot if the given action is chosen
        tempHeading = self.heading                              # Temporary heading direction. Created not to use the self.heading 
        headings = ('West', 'North', 'East', 'South')
        rotation = action[0]
        movement = action[1]
        # Updating heading direction and backVirtualSensor
        if (rotation == -90):   # Turn left
            if (tempHeading == 'West'):
                tempHeading = 'South'
            else:
                tempHeading = headings[headings.index(tempHeading) -1]
        elif (rotation == 90):  # Turn right
            if (self.heading == 'South'):
                tempHeading = 'West'
            else:
                tempHeading = headings[headings.index(tempHeading) +1]
#        else:                   # Doesn't turn
        
        # Updating the position. location = [x,y]
        if (tempHeading == 'North'):
            # update the y coordinate
            newLocation[1] += movement
        elif (tempHeading == 'West'):
            # update the x coordinate
            newLocation[0] -= movement
        elif (tempHeading == 'East'):
            # update the x coordinate
            newLocation[0] += movement
        else:   # heading = South
            # update the y coordinate
            newLocation[1] -= movement
            
        return newLocation
    
    def distance_heuristics(self, beginLocation, endLocation):
    
        # Manhattan distance (taxi cab distance)
        distance = abs(beginLocation[0] - endLocation[0]) + abs(beginLocation[1] - endLocation[1])
        return distance


    def exploration_heuristic(self, sensors, actions):
        
        """ This function, when used, will help the robot decide where to go in each time step. Will be an "educated guess" on
        where to go, based on the available directions and places less visited. Will be based on Tremaux's algorithm to explore
        but adding a heuristic that measures the distance of the current node to the goal location. Also, as the robot will know
        when it is at a dead-end, it will mark 5 times the dead-end to don't enter it again, instead of marking the dead-end node
        just once."""
        
        if (self.location[0] <= self.mazeDim/2):
            if (self.location[1] <= self.mazeDim/2):
                self.endLocation = (self.mazeDim/2,self.mazeDim/2)
            else:
                self.endLocation = (self.mazeDim/2,self.mazeDim/2+1)
        else:
            if (self.location[1] <= (self.mazeDim/2)):
                self.endLocation = (self.mazeDim/2+1,self.mazeDim/2)
            else:
                self.endLocation = (self.mazeDim/2+1,self.mazeDim/2+1)
                
        #  TREMAUX'S ALGORITHM MODIFICATIONS -> Dead-ends get marked 5 times.
        #  TREMAUX'S ALGORITHM MODIFICATIONS -> AT DEAD-ENDS, GO BACK UNTIL REACHES A BIFURCATION, AS MANY SQUARES (maximum 3) AS POSSIBLE
        #  TREMAUX'S ALGORITHM MODIFICATIONS -> When at a juction point with more than 1 possibility with equal ammount of marks -> Use the manhattan distance of current node to goal location as a heuristic to decide best action.
        #  TREMAUX'S ALGORITHM MODIFICATIONS -> Start as if it were leaving a dead-end
        
        # Mark current location.
        if (tuple(self.location) in self.exploratedPath.keys()):      # Previously visited location, add a "mark".
            if (self.debug):
                print "\nNode already visited with {} marks. Adding 1 mark to it...".format(self.exploratedPath[tuple(self.location)])

            self.exploratedPath[tuple(self.location)] += 1

            if (self.deadEnd):
                print "\nERROR! WASN'T SUPPOSED TO VISIT A DEAD-END NODE MORE THAN ONCE!"
           
        else:                                                   # New location, add it to the dict. and initialize with 1 "mark".
            if (self.deadEnd):                                  # If new location is a dead-end, initialize with 2 "marks".
                self.exploratedPath[tuple(self.location)] = 5
                if (self.debug):
                    print "\nNew dead-end! adding 5 marks to node."
            else:
                self.exploratedPath[tuple(self.location)] = 1
                if (self.debug):
                    print "\nNew node, adding it to the dict and starting with 1 mark."
                                            
        # Choose next action
        
        # Left the dead-end path?
        if (self.location == self.lastJunctionPoint):
            self.leftDeadEnd = True
            
        # Is this a junction point or a one way location?
        if (len(actions)<=2):                   # Are there at most 2 possibilities? i.e. going forward or backward
            # If in a path that will lead to a dead end, add more marks to the path so that the robot doesn't enter this path again.
            if (self.leftDeadEnd == False and (self.location != self.lastJunctionPoint) ):
                # Leaving a dead-end path. Adding more marks to each location until a junction point is reached.
                self.exploratedPath[tuple(self.location)] = 5
                if (self.debug):
                    print "\nAdded 5 marks to current location as it is a path to a dead-end"
                
            # Is it a dead-end?
            if (self.deadEnd):
                rotation, movement = self.handle_deadEnd(sensors)    # Call function to choose how to handle dead-ends
                if (self.debug):
                    print "\nDead end ahead, chose to go back."
                return rotation, movement
                    
            else:                               # At a node that is not a junction point and not a dead-end. Keep moving
                if (self.goingBackwards):
                    print actions
                    if (self.debug):
                        print "\nGoing backwards..."
                    if ( (0,1) in actions and len(actions)>=2):    
                        actions.remove((0,1))                       # No moving forward
                        if ( (0,-1) not in actions ):               # Robot will turn and so it will stop moving backwards
                            self.goingBackwards = False
                    elif ( (0,1) in actions and len(actions) <= 1):
                        self.goingBackwards = False
                        
                    rotation, movement = random.choice(actions)     # Choose action that is remaining
                    return rotation, movement
                
                if ( (0,-1) in actions ):                           # If not moving backwards
                    actions.remove((0,-1))
                if (self.debug):
                    print "\nGoing forward..."
                rotation, movement = random.choice(actions)         # Choose action that is remaining
                return rotation, movement
                    
        else:                                   # Junction point. Has more options than going forward or backward.
            self.goingBackwards = False
            self.deadEnd = False
            
            # Choose best action
            
            oneMarkActions = []             # Possible actions that will lead to squares that already has one mark to it.
            noMarkActions = []              # Possible actions that will lead to squares that currently has no marks to it.
            moreMarksActions = []           # Possible actions that will lead to squares that 2 or more marks to it
            
        
            # Calculating what are the best actions
            for a in actions:
                possibleLocation = tuple(self.action_to_location(a))              # The location that the robot would be if the action "a" is chosen
            
                if ( possibleLocation in self.exploratedPath.keys() ):              # Possible location already has marks -> Was already visited
                    if ( self.exploratedPath[possibleLocation] >= 2 ):              # Possible location has 2 or more marks -> Already visited 2 or more times
                        moreMarksActions.append(a)
                    elif ( self.exploratedPath[possibleLocation] == 1 ):            # Possible location has 1 mark -> Already visited once
                        oneMarkActions.append(a)
                else:                                                               # Possible location has no marks -> not visited yet
                    noMarkActions.append(a)         
            
            if (len(noMarkActions) > 0):                   # There are possible actions that will lead to unrmarked squares. Choose one of them.
                
                if (self.debug):
                    print "\nUnvisited Path found. Going to it"
                
                # Filter best actions according to distance to the closest goal location
                bestActions = [action for action in noMarkActions if self.distance_heuristics(tuple(self.action_to_location(action)), self.endLocation) <= self.distance_heuristics(tuple(self.location), self.endLocation) ]
                
                if (len(bestActions)>0):  
                    # Filter best actions according to distance to the the goal center point
                    bestActions2 = [action for action in bestActions if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                    if (len(bestActions2)>0):                               
                        chosenAction = random.choice(bestActions2)           
                    else:
                        chosenAction = random.choice(bestActions)
                else:
                    bestActions2 = [action for action in noMarkActions if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                    if (len(bestActions2)>0):                               
                        chosenAction = random.choice(bestActions2)
                    else:
                        chosenAction = random.choice(noMarkActions)         
                                
                if (chosenAction == (0,-1)):
                    self.goingBackwards = True          # If it was decide to go back, must continue to go back in next step, it it will be not a junction point
                
            elif (len(oneMarkActions) > 0):                                        # There are no possible actions that will lead to an unvisited location.
                if (self.debug):
                    print "\nNo possible action that will lead to an unvisited location. previous loc {} has {} marks.".format(self.prevLocation, self.exploratedPath[tuple(self.prevLocation)])
                if ( self.exploratedPath[tuple(self.prevLocation)] == 1 ):             # Previous location has 1 mark -> go back
                    if (self.debug):
                        print "\nPrevious location has one mark, go backward!"
                    # HOW WILL IT GO BACK? TURNING OR MOVING BACKWARDS?????????????????????????????
                    if ((0,-1) in tuple(actions)):
                        chosenAction = (0,-1)
                        self.goingBackwards = True                                      # UPDATE

                    else:
                        chosenAction = random.choice(oneMarkActions)
                
                else:                                                               # Previous location has 2 marks and there is a possible location with 1 mark only, chose them
                    if (self.debug):
                        print "\nPrevious location with more than one mark. Choosing other path..."
                    # Filter best actions according to distance to the closest goal location
                    bestActions = [action for action in oneMarkActions if self.distance_heuristics(tuple(self.action_to_location(action)), self.endLocation) <= self.distance_heuristics(self.location, self.endLocation) ]
                    if (len(bestActions)>0):       
                        # Filter best actions according to distance to the the goal center point
                        bestActions2 = [action for action in bestActions if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                        if (len(bestActions2)>0):
                            chosenAction = random.choice(bestActions2)           # Randomly chooses a bestAction. MAYBE CHANGE THIS?
                        else:
                            chosenAction = random.choice(bestActions)
                    else:
                        bestActions2 = [action for action in oneMarkActions if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                        if (len(bestActions2)>0):                               
                            chosenAction = random.choice(bestActions2)
                        else:
                            chosenAction = random.choice(oneMarkActions)
#                                        
                    if (chosenAction == (0,-1)):
                        self.goingBackwards = True
                    
            else:      
                lessMarksAction = [moreMarksActions[0]]
                
                # Getting actions with least ammount of marks
                for i in range(1,len(moreMarksActions)):
                    if (self.debug):
                        print "\nlessMarksAction: {}, moreMarksActions[i]: {}".format(lessMarksAction, moreMarksActions[i])
                    if ( self.exploratedPath[tuple(self.action_to_location(moreMarksActions[i]))] < self.exploratedPath[tuple(self.action_to_location(lessMarksAction[0]))] ):
                        lessMarksAction = [moreMarksActions[i]]
                    elif ( self.exploratedPath[tuple(self.action_to_location(moreMarksActions[i]))] == self.exploratedPath[tuple(self.action_to_location(lessMarksAction[0]))] ):
                        lessMarksAction.append(moreMarksActions[i])
                    else:
                        continue
                    
                # Filter best actions according to distance to the closest goal location
                bestActions = [action for action in lessMarksAction if self.distance_heuristics(tuple(self.action_to_location(action)), self.endLocation) <= self.distance_heuristics(self.location, self.endLocation) ]
                if (len(bestActions)>0):   
                        # Filter best actions according to distance to the the goal center point
                        bestActions2 = [action for action in bestActions if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                        if (len(bestActions2)>0):
                            chosenAction = random.choice(bestActions2)
                        else:
                            chosenAction = random.choice(bestActions)
                else:
                    bestActions2 = [action for action in lessMarksAction if self.distance_heuristics(tuple(self.action_to_location(action)), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) <= self.distance_heuristics(tuple(self.location), (self.mazeDim/2+0.5,self.mazeDim/2+0.5)) ]
                    if (len(bestActions2)>0):                               
                        chosenAction = random.choice(bestActions2)            
                    else:
                        chosenAction = random.choice(lessMarksAction)         
                
                if (self.debug):
                    print "\nNo possible actions with no marks or 1 marks!"
                                
                if (chosenAction == (0,-1)):
                    self.goingBackwards = True
                        
        chosenRotation, chosenMovement = chosenAction
        
        if (self.debug):
            print "\nChosen action: rotating {} degrees and moving {} squares".format(chosenRotation, chosenMovement)
        
        return chosenRotation, chosenMovement
    
    def choose_action(self,sensors):
        
        """ Choosing the next action for the robot """
        
        # Get possible actions
        actions = self.get_possible_actions(sensors)                          # Actions will be an array of (rotation, movement) possibilities
        
        # Choose possible actions
        rotation, movement = self.exploration_heuristic(sensors, actions)     # Choosing an action based on a heuristic
        
        # return action        
        return rotation, movement
    
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
 
        # Printing current state to console log
        if (self.verbose):
            print("\nTimeStep: {}".format(self.timeStep))
            print("current heading: {}     current position: {}".format(self.heading, self.location))
            print("Sensors input - Left: {}, Forward: {}, Right: {}".format(sensors[0], sensors[1], sensors[2]))
    
        # End current run?
        if (self.endRun):                   # Ending first run
            self.secondRun = True
            self.endRun = False                    
            
            return ('Reset', 'Reset')
        
        # Is it on second run?
        if (self.secondRun):                # On second run, get the best path discovered
            if (self.debug):
                print "\nSecond run!"
                            
            rotation = self.bestActions[self.timeStep -1][0]
            movement = self.bestActions[self.timeStep -1][1]

        else:                               
            # Building nodes dict for A*
            if (tuple(self.location) not in self.nodes.keys()):     # Add current node to the nodes dict to be used for A*
                neighbors = self.get_neighbors(sensors)         # Get neighbors of current node to build currentNode
                # add current node in the nodes list
                self.nodes[tuple(self.location)] = {'neighbors': neighbors}
     
            # Choose action in exploration
            rotation, movement = self.choose_action(sensors)
        
        # Printing chosen action to console log
        if (self.verbose):
            print("Rotation chosen: {}, Movement chosen: {}".format(rotation, movement))
        
        # Update transform -> simulating new state for internal use
        self.update_transform(rotation, movement, sensors)
        
        # Log
        if (not self.secondRun and self.endRun == False):
            self.logging(rotation, movement, sensors)
        
        # Render simulation
        if (self.renderSimulation):
            self.render_simulation(self.updateDelay)
        
        self.timeStep += 1                                      # Updating time step.
            
        return rotation, movement