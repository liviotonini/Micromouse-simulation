class Astar(object):
    
    def __init__(self, startLocation, endLocation, nodes, verbose=False):
               
        self.verbose = verbose
        if (self.verbose):
            print "\nAstar initiated..."
        
        self.nodes = nodes
        self.openSet = []
        self.closedSet = []
        self.endLocation = endLocation
        self.startLocation = startLocation
                
        # Build first node
        startH = self.heuristic(startLocation, endLocation)
        
        self.nodes[startLocation] = {'f':startH, 'g':0, 'h': startH, 'previous': None, 'neighbors':self.nodes[startLocation]['neighbors']}        
        self.openSet.append(startLocation)
        
        self.i = 1                                      # iteration value
                
    def heuristic(self, aLocation, bLocation):
        # Manhattan distance
        deltaX = abs(aLocation[0] - bLocation[0])
        deltaY = abs(aLocation[1] - bLocation[1])
        d = deltaX + deltaY
            
        return d
    
    def build_path(self, current):
        
        """ This function will build the best path and output it. """
        
        if (self.verbose):
            print "\nFinal path being built!"
        
        total_path = [current]
        
        while (current != self.startLocation):
            current = self.nodes[current]['previous']
            total_path.append(current)
           
        index = len(total_path)-1
        previousMovementDirection = "North"             # vertically
        finish = False
        actions = []

        pathLength = 0

        # Transforming locations into actions
        while (not finish):
            # Moving backwards on the array
            deltaX = total_path[index-1][0] - total_path[index][0]
            deltaY = total_path[index-1][1] - total_path[index][1]
        
            # Calculating rotation
            if (deltaX>0):
                if (previousMovementDirection == "North"):
                    rotation = 90
                    previousMovementDirection = "East"
                elif (previousMovementDirection == "South"):
                    rotation = -90
                    previousMovementDirection = "East"
                elif (previousMovementDirection == "West"):
                    rotation = 180
                    previousMovementDirection = "East"
                elif (previousMovementDirection == "East"):
                    rotation = 0
                    previousMovementDirection = "East"
                
            elif (deltaX<0):
                if (previousMovementDirection == "North"):
                    rotation = -90
                    previousMovementDirection = "West"
                elif (previousMovementDirection == "South"):
                    rotation = 90
                    previousMovementDirection = "West"
                elif (previousMovementDirection == "West"):
                    rotation = 0
                    previousMovementDirection = "West"
                elif (previousMovementDirection == "East"):
                    rotation = 180
                    previousMovementDirection = "West"
            
            elif(deltaY>0):
                if (previousMovementDirection == "North"):
                    rotation = 0
                    previousMovementDirection = "North"
                elif (previousMovementDirection == "South"):
                    rotation = 180
                    previousMovementDirection = "North"
                elif (previousMovementDirection == "West"):
                    rotation = 90
                    previousMovementDirection = "North"
                elif (previousMovementDirection == "East"):
                    rotation = -90
                    previousMovementDirection = "North"

            elif(deltaY<0):
                if (previousMovementDirection == "North"):
                    rotation = 180
                    previousMovementDirection = "South"
                elif (previousMovementDirection == "South"):
                    rotation = 0
                    previousMovementDirection = "South"
                elif (previousMovementDirection == "West"):
                    rotation = -90
                    previousMovementDirection = "South"
                elif (previousMovementDirection == "East"):
                    rotation = 90
                    previousMovementDirection = "South"
            else:
                if (self.verbose):
                    print "\nERROR!!!!"
                        
            # Calculating movement
            movement = min(3, (abs(deltaX)+abs(deltaY)) )
            actions.append([rotation, movement])
            
            pathLength += movement
            
            if (self.verbose):
                print "\nindex: {} action - ({}, {})".format(index, rotation, movement)
            index -= 1
            if (index == 0):
                finish = True
        # end while loop

        finish = False
        
        if (self.verbose):
            print "\nActions: {}".format(actions)
        
        return actions, total_path, pathLength
        
    def calculating_path(self):
        if (self.verbose):
            print "\nCalculating path..."
        
        noSolution = False
        finish = False
        
        while (len(self.openSet)>0 or noSolution == True or finish == True):
                    
            bestNext = self.openSet[0]                  # Initial assumption
            
            # Get best next
            for loc in self.openSet:
                if (self.nodes[tuple(loc)]['f'] < self.nodes[tuple(bestNext)]['f']):
                    bestNext = loc
                    if (self.verbose):
                        print "\nNew bestNext found with a better F"
                elif (self.nodes[loc]['f'] == self.nodes[bestNext]['f'] ):
                    if (self.nodes[loc]['g'] > self.nodes[bestNext]['g']):
                        if (self.verbose):
                            print "\nNew bestNext found with a better G"
                        bestNext = loc

            # end for loop of the best next
            
            current = bestNext
            if (self.verbose):
                print "\nCurrent: {}".format(current)
            
            # Finish?
            if (current == tuple(self.endLocation)):
                if (self.verbose):
                    print "DONE!"
                finish = True                                                  
                bestActions, total_path, pathLength = self.build_path(current)
                if (self.verbose):
                    print "bestActions: {}".format(bestActions)
                    print "\nbestActions length: {}".format(len(bestActions))
                return bestActions, total_path, pathLength
                break
                
            # best option moves from openSet to closedSet
            self.openSet.remove(current)
            self.closedSet.append(current)
            
            if (self.verbose):
                print "\nopenSet: {} \nclosedSet: {}".format(self.openSet, self.closedSet)
            
            neighbors = [ n for n in self.nodes[current]['neighbors'] if n in self.nodes ]

            if (self.verbose):
                print "\nGetting neighbors: {}".format(neighbors)
            for n in neighbors:
                if (self.verbose):                
                    print "\ncurrent neighbor: {}".format(n)                
                # Valid next spot?
                if (n not in self.closedSet):
                    
                    tempG = self.nodes[current]["g"] + 1                        # No diagonal move will be allowed, so the cost will always be added of 1
                    
                    if (self.verbose):
                        print "tempG: {}".format(tempG)
                    # Is this a better path?
                    if (n not in self.openSet):
                        self.openSet.append(n)
                        if (self.verbose):
                            print "\nneighbor {} added to openSet".format(n)
                    elif (tempG >= self.nodes[n]["g"]):
                        if (self.verbose):
                            print "\nneighbor {} not better".format(n)
                        # Not a better path
                        continue
                    
                    self.nodes[n]["g"] = tempG
                    self.nodes[n]["h"] = self.heuristic(n, self.endLocation)
                    self.nodes[n]["f"] = self.nodes[n]["g"] + self.nodes[n]["h"]
                    self.nodes[n]["previous"] = current
                    
                    if (self.verbose):
                        print "\nself.nodes[n] : {}".format(self.nodes[n])
                    
            self.i += 1
            if (self.verbose):
                print "\niteracao: {}".format(self.i)
            # end for
        # end if openSet > 0
        if (self.verbose):
            return "No Solution Found!"