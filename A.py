import random
import numpy as np
import sys
import heapq
import matplotlib.pyplot as plt
from matplotlib import colors
import os
import cProfile


mazeMaxY = 101
mazeMaxX = 101


class maze:
    def __init__(self, fileName):
        self.isBackward = 0
        self.goal = self.generateGoal(fileName)
        self.start = self.generateStart(fileName, self.goal[0], self.goal[1])
        self.map = self.loadMaze(fileName)
        self.expansions = 0
    def loadMaze(self, fileName):
        actualMaze = np.loadtxt(fileName, delimiter = ' ').astype(int) 
        filledMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]
        for i in range(0, mazeMaxY):
            for j in range(0, mazeMaxX):
                if(self.isBackward == 1):
                    filledMaze[i][j] = state(0, j, i, None, self.start[0], self.start[1])
                else:
                    filledMaze[i][j] = state(0, j, i, None, self.goal[0], self.goal[1])
        return filledMaze
    def generateGoal(self, fileName):
        goal = []
        mazeFile = open(fileName)
        mazeData = mazeFile.readlines()
        while True:
            goalX, goalY = (random.randrange(0, mazeMaxX,1)), (random.randrange(0, mazeMaxY,1))
            if((mazeData[goalY][goalX*2] != "1")):
                break
        print("goal X is: " + str(goalX) + " goal Y is: " + str(goalY))
        goal.append(goalX)
        goal.append(goalY)
        mazeFile.close()
        return goal
    def generateStart(self, fileName, goalX, goalY):
        start = []
        mazeFile = open(fileName)
        mazeData = mazeFile.readlines()
        while True:
            startX, startY = (random.randrange(0, mazeMaxX,1)), (random.randrange(0, mazeMaxY,1))
            if((startX != goalX) and (startY != goalY) and (mazeData[startY][startX*2] != "1")):
                break
        print("start X is: " + str(startX) +" start Y is: " + str(startY))
        start.append(startX)
        start.append(startY)
        mazeFile.close()
        return start


class state:
    def __init__(self, gVal, x, y, parent, goalX, goalY):
        self.gVal = gVal
        self.x = x
        self.y = y
        self.parent = parent
        self.hVal = self.getHeuristic(goalX, goalY)
        self.fVal = self.getFVal()
        self.search = 0
        self.isBlocked = 'n'
        self.isOpen = 'n'
    def __eq__(self, other):
        return type(self) == type(other) and  ((self.x == other.x) and (self.y == other.y))
    def getHeuristic(self, goalX, goalY):
        hVal = (abs(goalX-self.x) + abs(goalY-self.y))
        #print(hVal)
        return hVal
    def getFVal(self):
        return (self.gVal + self.hVal)
    def getCoordinates(self):
        return ("(" + str(self.x) + "," + str(self.y) + ")")
    def __lt__(self, other):
        if(self.fVal == other.fVal):
            return ((self.gVal>other.gVal))
            #Enable above for LARGER gVal tie breaking, enable below for SMALLER gVal tie breaking
            #return ((self.gVal<other.gVal))
        else:
            return (self.fVal < other.fVal)
    def getNeighbor(self, direction, maze, actualMaze):
        #Check bounds
        newX, newY =  0, 0
        if(direction == 'r'):
            if(((self.x + 1) >= mazeMaxX)):
                return None
            #print("Checking right: ", end="")
            newX, newY = self.x + 1, self.y
        elif(direction == 'u'):
            if(((self.y - 1) < 0)):
                 return None
            #print("Checking up: ", end="")
            newX, newY = self.x, self.y - 1
        elif(direction == 'l'):
            if( ((self.x - 1) < 0)):
                return None
            #print("Checking left: ", end="")
            newX, newY = self.x - 1, self.y
        else:
            if( ((self.y + 1) >= mazeMaxY)):
                return None
            #print("Checking down: ", end="")
            newX, newY = self.x, self.y + 1
        
        nebState = maze[newY][newX]
        #if(nebState.isBlocked == 1):
        if(maze[newY][newX].isBlocked == 'y'):
            #print("(" + str(newX) + "," + str(newY) + ") is BLOCKED ")
            return None
        else:
            #print("(" + str(newX) + "," + str(newY) + ") is NOT BLOCKED")
            return nebState
class robot:
    def __init__(self, currentState):
        self.currentState = currentState
    def move(self, newState):
        self.currentState = newState
    def updateRobotMaze(self, actualMaze, maze):
            newXR, newYR = self.currentState.x + 1 , self.currentState.y
            #print("Checking right: ")
            if(newXR < mazeMaxX):
                if((actualMaze[newYR][(newXR)]) == 1):
                    #Right state is blocked -> update isBlocked to 1
                    #print("BLOCKED...updating map")
                    maze[newYR][newXR].isBlocked = 'y'
            
            newXU, newYU = self.currentState.x , self.currentState.y - 1
            #print("Checking up: ")
            if(newYU >= 0):
                if((actualMaze[newYU][newXU]) == 1):
                    #Up state is blocked -> update isBlocked to 1
                    #print("BLOCKED...updating map")
                    maze[newYU][newXU].isBlocked = 'y'
            newXL, newYL = self.currentState.x-1 , self.currentState.y
            #print("Checking left: ")
            if(newXL >=0):
                if((actualMaze[newYL][newXL]) == 1):
                    #Left state is blocked -> update isBlocked to 1
                    #print("BLOCKED...updating map")
                    maze[newYL][newXL].isBlocked = 'y'
            newXD, newYD  = self.currentState.x , self.currentState.y+1
            #print("Checking down: ")
            if(newYD < mazeMaxY):
                if((actualMaze[newYD][newXD]) == 1):
                    #Down state is blocked -> update isBlocked to 1
                    #print("BLOCKED...updating map")
                    maze[newYD][newXD].isBlocked = 'y'

def aStar(maze, counter, openList, closedList, goalState, expansionDict, actualMaze):
    #Main A* algo
    poppedCounter = 0
    print("Starting: " + str(counter) + "th iteration of A*")
    while(openList):
        #Get nextState w/ lowest fVal
        currentState = heapq.heappop(openList)
        currentState.isOpen = 'n'
        poppedCounter+=1
        #print("poppedCounter is: " + str(poppedCounter))
        if(currentState == goalState):
                    print("Found the goalState! " + goalState.getCoordinates())
                    return currentState
        #print("CurrentState is: " + currentState.getCoordinates())
        #Add nextState to closed list
        if((isInClosedList(currentState, closedList)) == 0):
            #print("Expanding state: " + currentState.getCoordinates())
            closedList[currentState.getCoordinates()] = currentState 
            #closedList.append(currentState)
            maze.expansions = maze.expansions + 1
        
        #Generate successor states from currentState
        for action in ["r", "u", "l", "d"]:
            #Check if there exists a succ(currentState, action):
            succState = currentState.getNeighbor(action, maze.map, actualMaze)
            if(succState==None):
                continue
            #Successor state exists
            if(isInClosedList(succState, closedList) == 1):
                continue

            #If neighbor has not been initialized by current A* call
            if(succState.search < counter):
                succState.gVal = sys.maxsize
                succState.fVal = succState.getFVal()
                succState.search = counter
                succState.isOpen = 'n'
            #prevSuccState = IsInOpenList(succState, openList)
            if(succState.isOpen == 'y'):
                if(succState.gVal > (currentState.gVal + 1)):
                    succState.gVal = (currentState.gVal + 1)
                    succState.fVal = succState.getFVal()
                    succState.parent = currentState
                    succState.search = counter
            else:
                #print("Next neighbor is: " + succState.getCoordinates())
                succState.gVal = (currentState.gVal + 1)
                succState.fVal = succState.getFVal()
                succState.parent = currentState
                succState.search = counter
                heapq.heappush(openList, succState)
                succState.isOpen = 'y'
                #heapq.heapify(openList)
    return None

def isInClosedList(state, closedList):
    if state.getCoordinates() in closedList:
        #state is in closedList -> return 1 for true
        return 1
    else:
        return 0

def IsInOpenList(state, openList):
    for s in openList:
        if(s == state):
            #state is in openList -> return 1 for true
            return s
    return None


def printClosedList(closedList):
    print("Closed list is: ", end="")
    for k in closedList.keys():
        print(k + " , ");

def aStarAdaptiveDriver(robot, fileName, maze, initialStartState, goalState, filePath):
    #Main A* driver
    actualMaze = np.loadtxt(fileName, delimiter = ' ').astype(int) 
    expansionDict = {}
    startState = initialStartState
    counter = 0
    currentMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]
    finalPath = []
    while(startState != goalState):
        counter+=1
        startState.gVal = 0
        startState.fVal = startState.getFVal()
        startState.search = counter
        goalState.search = counter
        goalState.gVal = sys.maxsize
        goalState.fVal = goalState.getFVal
        openList = []
        heapq.heapify(openList)
        closedList = {}
        heapq.heappush(openList, startState)
        startState.isOpen = 'y'
       #heapq.heapify(openList)
        oldExpansions = maze.expansions
        robot.updateRobotMaze(actualMaze, maze.map)
        resultState = aStar(maze, counter, openList, closedList, goalState, expansionDict, actualMaze)
        
        startState.isOpen = 'n'
        #Update hVals:
        for key in closedList.keys():
            closedList[key].hVal = goalState.gVal - closedList[key].gVal
            closedList[key].fVal = closedList[key].getFVal()
        

        expansionDict[counter] = maze.expansions - oldExpansions
        #printPathBackTrack(resultState)
        if(resultState == None):
            print("Target cannot be reached")
            return 0
        path = []
        currentState = resultState
        #iteration = 0
        while(currentState != startState):
            #print("Appending " + currentState.getCoordinates() + " to path")
            path.append(currentState)
            currentState = currentState.parent
            #iteration+=1
        #print("Path is: ", end="")
        #Execute path...
        while(path):
            nextMove = path.pop()
            #print("Popped out: " + nextMove.getCoordinates())
            
            #Check if next move is blocked
            if(actualMaze[nextMove.y][nextMove.x] == 1):
                #print("nextMove is BLOCKED")
                break
            
            #Path is valid so far...execute and update robot's vision:
            print("Moving robot from: " + robot.currentState.getCoordinates() + " to " + nextMove.getCoordinates())
            currentMaze[nextMove.y][nextMove.x] = 8
            robot.move(nextMove)
            finalPath.append(nextMove.getCoordinates())        
        
        if(((counter % 10 ) == 0 ) or (counter == 1)):
            visualizeMaze(currentMaze, maze, startState, counter, filePath)
        #Reset start to new robot's starting position
        startState = robot.currentState
        #update any action costs
    visualizeMaze(currentMaze, maze, startState, counter, filePath)
    print("Got to target in " + str(counter) + " iterations and " + str(maze.expansions) + " total expansions")
    print("Goal: " + goalState.getCoordinates())
    print("Start: " + initialStartState.getCoordinates())
    #print("Final Path is: " + initialStartState.getCoordinates() + ", ", end="")
    #for position in finalPath:
    #    print(position + ", ", end="")
    #print(actualMaze)
    #print("Expansions are: ")
    #for k in expansionDict:
    #    print( "Iteration: " + str(k) + " # Expansions: " + str(expansionDict[k]))

    return 1

def aStarDriver(robot, fileName, maze, initialStartState, goalState, filePath):
    #Main A* driver
    actualMaze = np.loadtxt(fileName, delimiter = ' ').astype(int) 
    expansionDict = {}
    startState = initialStartState
    counter = 0
    currentMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]
    finalPath = []
    while(startState != goalState):
        counter+=1
        startState.gVal = 0
        startState.fVal = startState.getFVal()
        startState.search = counter
        goalState.search = counter
        goalState.gVal = sys.maxsize
        goalState.fVal = goalState.getFVal
        openList = []
        heapq.heapify(openList)
        closedList = {}
        heapq.heappush(openList, startState)
        startState.isOpen = 'y'
        #heapq.heapify(openList)
        oldExpansions = maze.expansions
        robot.updateRobotMaze(actualMaze, maze.map)
        resultState = aStar(maze, counter, openList, closedList, goalState, expansionDict, actualMaze)
        
        startState.isOpen = 'n'

        expansionDict[counter] = maze.expansions - oldExpansions
        #printPathBackTrack(resultState)
        if(resultState == None):
            print("Target cannot be reached")
            return 0
        path = []
        currentState = resultState
        #iteration = 0
        while(currentState != startState):
            #print("Appending " + currentState.getCoordinates() + " to path")
            path.append(currentState)
            currentState = currentState.parent
            #iteration+=1
        #print("Path is: ", end="")
        #Execute path...
        while(path):
            nextMove = path.pop()
            #print("Popped out: " + nextMove.getCoordinates())
            
            #Check if next move is blocked
            if(actualMaze[nextMove.y][nextMove.x] == 1):
                #print("nextMove is BLOCKED")
                break
            
            #Path is valid so far...execute and update robot's vision:
            print("Moving robot from: " + robot.currentState.getCoordinates() + " to " + nextMove.getCoordinates())
            currentMaze[nextMove.y][nextMove.x] = 8
            robot.move(nextMove)
            finalPath.append(nextMove.getCoordinates())        
        
        if(((counter % 10 ) == 0 ) or (counter == 1)):
            visualizeMaze(currentMaze, maze, startState, counter, filePath)
        #Reset start to new robot's starting position
        startState = robot.currentState
        #update any action costs
    visualizeMaze(currentMaze, maze, startState, counter, filePath)
    print("Got to target in " + str(counter) + " iterations and " + str(maze.expansions) + " total expansions")
    print("Goal: " + goalState.getCoordinates())
    print("Start: " + initialStartState.getCoordinates())
    #print("Final Path is: " + initialStartState.getCoordinates() + ", ", end="")
    #for position in finalPath:
    #    print(position + ", ", end="")
    #print(actualMaze)
    #print("Expansions are: ")
    #for k in expansionDict:
    #    print( "Iteration: " + str(k) + " # Expansions: " + str(expansionDict[k]))

    return 1

def visualizeMaze(currentMaze, maze, startState, counter, filePath):
    #currentMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]

    for i in range(0, mazeMaxY):
        for j in range(0, mazeMaxX):
            if(maze.map[i][j].isBlocked == 'y'):
                currentMaze[i][j] = 1
            else:
                if(currentMaze[i][j] == 8):
                    continue
                currentMaze[i][j] = 0
    currentMaze[startState.y][startState.x] = 8
    currentMaze[maze.start[1]][maze.start[0]] = 4
    currentMaze[maze.goal[1]][maze.goal[0]] = 6

    colorMap = colors.ListedColormap(['white', 'black', '#0AEA33','#00D8FF', '#D206FF',])
    norm = colors.BoundaryNorm([0,1,3, 5, 7, 9], colorMap.N)
    plt.figure()
    plt.imshow(currentMaze, cmap=colorMap, norm=norm, interpolation='nearest')
    plt.xticks([])
    plt.yticks([])
    plt.savefig(filePath + "/" + "iterations/mazePic{0:d}.png".format(counter))
    plt.close()

def visualizeMazeBackward(currentMaze, maze, startState, counter, filePath):
    #currentMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]

    for i in range(0, mazeMaxY):
        for j in range(0, mazeMaxX):
            if(maze.map[i][j].isBlocked == 'y'):
                currentMaze[i][j] = 1
            else:
                if(currentMaze[i][j] == 8):
                    continue
                currentMaze[i][j] = 0
    currentMaze[startState.y][startState.x] = 8
    currentMaze[maze.start[1]][maze.start[0]] = 6
    currentMaze[maze.goal[1]][maze.goal[0]] = 4

    colorMap = colors.ListedColormap(['white', 'black', '#0AEA33','#00D8FF', '#D206FF',])
    norm = colors.BoundaryNorm([0,1,3, 5, 7, 9], colorMap.N)
    plt.figure()
    plt.imshow(currentMaze, cmap=colorMap, norm=norm, interpolation='nearest')
    plt.xticks([])
    plt.yticks([])
    plt.savefig(filePath + "/" + "iterations/mazePic{0:d}.png".format(counter))
    plt.close()


def aStarBackwardDriver(robot, fileName, maze, initialStartState, goalState, filePath):
    #Main A* driver
    actualMaze = np.loadtxt(fileName, delimiter = ' ').astype(int) 
    expansionDict = {}
    startState = goalState#initialStartState
    goalState = initialStartState
    counter = 0
    currentMaze = [ [0]*mazeMaxX for row in range(mazeMaxY)]
    finalPath = []
    while(startState != goalState):
        counter+=1
        startState.gVal = 0
        startState.fVal = startState.getFVal()
        startState.search = counter
        goalState.search = counter
        goalState.gVal = sys.maxsize
        goalState.fVal = goalState.getFVal
        openList = []
        heapq.heapify(openList)
        closedList = {}
        heapq.heappush(openList, startState)
        startState.isOpen = 'y'
        #heapq.heapify(openList)
        oldExpansions = maze.expansions
        robot.updateRobotMaze(actualMaze, maze.map)
        print("Planning start: " + startState.getCoordinates())
        print("goalState: " + goalState.getCoordinates())
        resultState = aStar(maze, counter, openList, closedList, goalState, expansionDict, actualMaze)
        
        startState.isOpen = 'n'

        expansionDict[counter] = maze.expansions - oldExpansions
        #printPathBackTrack(resultState)
        if(resultState == None):
            print("Target cannot be reached")
            return 0
        path = []
        currentState = resultState
        #iteration = 0
        while(currentState != startState):
            if(currentState == robot.currentState):
                currentState = currentState.parent
                if(currentState == None):
                    break
            #print("Appending " + currentState.getCoordinates() + " to path")
            #if(currentState.getCoordinates() == initialStartState.getCoordinates()):
                #continue
            path.append(currentState)
            currentState = currentState.parent
            if(currentState == None):
                break
            #iteration+=1
        #print("Path is: ", end="")
        #Execute path...
        for nextMove in path:
            if(nextMove == robot.currentState):
                continue
            #print("Next Move: " + nextMove.getCoordinates())
            
            #Check if next move is blocked
            if(actualMaze[nextMove.y][nextMove.x] == 1):
                #print("nextMove is BLOCKED")
                break
            
            #Path is valid so far...execute and update robot's vision:
            print("Moving robot from: " + robot.currentState.getCoordinates() + " to " + nextMove.getCoordinates())
            currentMaze[nextMove.y][nextMove.x] = 8
            robot.move(nextMove)
            finalPath.append(nextMove.getCoordinates())        
        
        if(((counter % 10 ) == 0 ) or (counter == 1)):
            visualizeMazeBackward(currentMaze, maze, startState, counter, filePath)
        #Reset start to new robot's starting position
        goalState = robot.currentState
        #startState = robot.currentState
        #update any action costs
    visualizeMazeBackward(currentMaze, maze, startState, counter, filePath)
    print("Got to target in " + str(counter) + " iterations and " + str(maze.expansions) + " total expansions")
    print("Goal: " + goalState.getCoordinates())
    print("Start: " + initialStartState.getCoordinates())

    return 1



#Main():
#maze1 = maze('testMaze.txt')

#maze1.isBackward = 0

#maze1.goal[0] = 20
#maze1.goal[1] = 10
#maze1.start[0] = 0
#maze1.start[1] = 0

#initialStartState = maze1.map[maze1.start[1]][maze1.start[0]]
#initialStartState.gVal = 0
#initialStartState.fVal = initialStartState.getFVal()
#myRobot = robot(initialStartState)
#goalState = maze1.map[maze1.goal[1]][maze1.goal[0]]
#filePath = "iterations"
#profile = cProfile.Profile()
#profile.enable()
#aStarDriver(myRobot, 'testMaze.txt', maze1, initialStartState, goalState, filePath)
#profile.disable()
#profile.print_stats()
