"""
Spyder Editor

Este é um arquivo de script temporário.

"""
import pickle
from Astar import Astar

file_nodes = open('nodes_file', 'r')
nodesFile = pickle.load(file_nodes)

startLocation = (1,1)
endLocation = (7,8)

aStar = Astar(startLocation, endLocation ,nodesFile, True)
#print aStar.openSet
bestActions, total_path, pathLength = aStar.calculating_path()

print bestActions, len(bestActions), pathLength
