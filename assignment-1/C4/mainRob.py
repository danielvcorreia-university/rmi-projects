
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[1] > (len(maze) - 1) or node_position[1] < 0 or node_position[0] > (len(maze[len(maze)-1]) -1) or node_position[0] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[1]][node_position[0]] != 'X':
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Create the f, g, and h values
            child.g = current_node.g + 1

            index = None
            if child in open_list: index = open_list.index(child)

            if child not in closed_list and (index == None or child.g <= open_list[index].g) :
                child.h = sqrt( ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2) )
                child.f = child.g + child.h
                # Add the child to the open list
                open_list.append(child)

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.beaconsList = []
        self.x = 28.0
        self.y = 14.0
        self.outleft = 0.0
        self.outright = 0.0

        #for testing
        self.initialX = 0.0
        self.initialY = 0.0

        #testing motors adjust
        self.counter = 0

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap, minpathMap):
        self.labMap = labMap

    def printMap(self, outMap=None):
        for l in reversed(self.labMap):
            #print(''.join([str(l) for l in l]), file=outMap)
            line = ''
            for i in range(1, len(l)):
                line += l[i]
            print(''.join([line]), file=outMap)

    def fillMap(self, x, y, symbol):
        self.labMap[y][x] = symbol

    def calcTurnLeftDir(self, dir):
        if dir > -20 and dir < 20:
            return 90
        elif dir > 70 and dir < 110:
            return 180
        elif dir > 160 or dir < -160:
            return -90
        else: #if dir > -110 and dir < -70:
            return 0
        
    def calcTurnRightDir(self, dir):
        if dir > -20 and dir < 20:
            return -90
        elif dir > 70 and dir < 110:
            return 0
        elif dir > 160 or dir < -160:
            return 90
        else: #if dir > -110 and dir < -70:
            return 180
    
    def calcTurnAroundDir(self, dir):
        if dir > -20 and dir < 20:
            return 180
        elif dir > 70 and dir < 110:
            return -90
        elif dir > 160 or dir < -160:
            return 0
        else: #if dir > -110 and dir < -70:
            return 90

    def calcClosestDir(self,dir):
        if dir > -20 and dir < 20:
            return 0
        elif dir > 70 and dir < 110:
            return 90
        elif dir > 160 or dir < -160:
            return 180
        else: #if dir > -110 and dir < -70:
            return -90

    def fillWalls(self, x, y, dir, center, left, right):
        if dir > -20 and dir < 20:
            if center >= 1.4 and self.labMap[y][x+1] != 'X':
                self.fillMap(x+1,y,'|')
            if left >= 1.6 and self.labMap[y+1][x] != 'X':
                self.fillMap(x,y+1,'-')
            if right >= 1.6 and self.labMap[y-1][x] != 'X':
                self.fillMap(x,y-1,'-')

        elif dir > 70 and dir < 110:
            if center >= 1.4 and self.labMap[y+1][x] != 'X':
                self.fillMap(x,y+1,'-')
            if left >= 1.6 and self.labMap[y][x-1] != 'X':
                self.fillMap(x-1,y,'|')
            if right >= 1.6 and self.labMap[y][x+1] != 'X':
                self.fillMap(x+1,y,'|')

        elif dir > 160 or dir < -160:
            if center >= 1.4 and self.labMap[y][x-1] != 'X':
                self.fillMap(x-1,y,'|')
            if left >= 1.6 and self.labMap[y-1][x] != 'X':
                self.fillMap(x,y-1,'-')
            if right >= 1.6 and self.labMap[y+1][x] != 'X':
                self.fillMap(x,y+1,'-')

        else: #if dir > -110 and dir < -70:
            if center >= 1.4 and self.labMap[y-1][x] != 'X':
                self.fillMap(x,y-1,'-')
            if left >= 1.6 and self.labMap[y][x+1] != 'X':
                self.fillMap(x+1,y,'|')
            if right >= 1.6 and self.labMap[y][x-1] != 'X':
                self.fillMap(x-1,y,'|')

    def checkIfFrontOld(self, x, y, dir):
        if dir > -20 and dir < 20:
            if labMap[y][x+2] == 'X' and labMap[y][x+1] == 'X':
                return True
            return False

        elif dir > 70 and dir < 110:
            if labMap[y+2][x] == 'X' and labMap[y+1][x] == 'X':
                return True
            return False

        elif dir > 160 or dir < -160:
            if labMap[y][x-2] == 'X' and labMap[y][x-1] == 'X':
                return True
            return False

        else: #if dir > -110 and dir < -70:
            if labMap[y-2][x] == 'X' and labMap[y-1][x] == 'X':
                return True
            return False

    def fillIntermediateX(self, x, y, dir):
        if dir > -20 and dir < 20:
            if labMap[y][x+1] == ' ':
                self.fillMap(x+1,y,'X')

        elif dir > 70 and dir < 110:
            if labMap[y+1][x] == ' ':
                self.fillMap(x,y+1,'X')

        elif dir > 160 or dir < -160:
            if labMap[y][x-1] == ' ':
                self.fillMap(x-1,y,'X')

        else: #if dir > -110 and dir < -70:
            if labMap[y-1][x] == ' ':
                self.fillMap(x,y-1,'X')

    def getUnopenedX(self):
        saveCoords = []

        #Encontrar todos os 'X' com um ' ' adjacente
        for x in range (2, 54, 2):
            for y in range (2, 26, 2):
                if self.labMap[y][x] == 'X' and (self.labMap[y][x+1] == ' ' or self.labMap[y][x-1] == ' ' or self.labMap[y+1][x] == ' ' or self.labMap[y-1][x] == ' '):
                    saveCoords.append((x,y))

        #print("saveCoords: "+ str(saveCoords))
        return saveCoords
                
    def calcDirByCoords(self, currCoords, nextCoords):
        res = (nextCoords[0]-currCoords[0], nextCoords[1]-currCoords[1])

        if res == (2,0):
            return 0
        elif res == (-2,0):
            return 180
        elif res == (0,2):
            return 90
        else: # res == (0, -2)
            return -90

    def checkCorrectDir(self, dir, dirToRotate):
        if dir > -20 and dir < 20 and dirToRotate == 0:
            return True
        elif dir > 70 and dir < 110 and dirToRotate == 90:
            return True
        elif (dir > 160 or dir < -160) and dirToRotate == 180:
            return True
        elif dir > -110 and dir < -70 and dirToRotate == -90:
            return True
        else: 
            return False
    
    def checkCorrectDirAstar(self, dir, dirToRotate):
        if dir > -45 and dir < 45 and dirToRotate == 0:
            return True
        elif dir > 45 and dir < 135 and dirToRotate == 90:
            return True
        elif (dir > 135 or dir < -135) and dirToRotate == 180:
            return True
        elif dir > -135 and dir < -45 and dirToRotate == -90:
            return True
        else: 
            return False

    def checkTurnLeft(self, dir, dirToRotate):
        if dir > -45 and dir < 45:
            if dirToRotate == 90:
                return True
        elif dir > 45 and dir < 135:
            if dirToRotate == 180:
                return True
        elif (dir > 135 or dir < -135):
            if dirToRotate == -90:
                return True
        elif dir > -135 and dir < -45:
            if dirToRotate == 0:
                return True
        return False

    def getAstarPath(self, intrealX, intrealY):
        #Coordenadas de 'X' que ainda têm um ' ' adjacente
        saveCoords = self.getUnopenedX()
        if saveCoords == []:
            self.finished = True
            minpath = astar(self.labMap,(intrealX,intrealY), self.beaconsList[0])
            minpath = [v for i, v in enumerate(minpath) if i % 2 == 0]
            return minpath
            

        #Calcular o caminho mais curto para um 'X' através de a*
        minpath = None
        first = True
        for i, unopenedX in enumerate(saveCoords):
            newpath = astar(self.labMap,(intrealX,intrealY), unopenedX)
            if newpath != None:
                if first == True: 
                    minpath = newpath
                    first = False
                if len(newpath) < len(minpath): 
                    minpath = newpath

        minpath = [v for i, v in enumerate(minpath) if i % 2 == 0]
        return minpath

    # Python function to print permutations of a given list
    def permutation(self, lst):
        # If lst is empty then there are no permutations
        if len(lst) == 0:
            return []
        # If there is only one element in lst then, only
        # one permutation is possible
        if len(lst) == 1:
            return [lst]
        # Find the permutations for lst if there are
        # more than 1 characters
        l = [] # empty list that will store current permutation
        # Iterate the input(lst) and calculate the permutation
        for i in range(len(lst)):
            m = lst[i]
            # Extract lst[i] or m from the list.  remLst is
            # remaining list
            remLst = lst[:i] + lst[i+1:]
            # Generating all permutations where m is first
            # element
            for p in self.permutation(remLst):
                l.append([m] + p)
        return l

    def foundOptimalPath(self):
        myPath = None
        for index, perm in enumerate(self.permutation(self.beaconsList[1:])):
            minPath = [(28,14)]
            for i in range(0, len(perm)+1):
                path = None
                if i == 0:
                    path = astar(self.labMap,self.beaconsList[0], perm[i])
                elif i == len(perm):
                    path = astar(self.labMap, perm[i-1], self.beaconsList[0])
                else:
                    path = astar(self.labMap, perm[i-1], perm[i])
                if path == None:
                    break

                path = [v for i, v in enumerate(path) if i % 2 == 0]
                for node in path[1:]:
                    minPath.append(node)
            if index == 0 or len(minPath) < len(myPath):
                myPath = minPath
        print("myPath: "+str(myPath))
        return [True, myPath]

    def finishProgram(self,myPath):
        #Map file
        for beacon in self.beaconsList:
            self.fillMap(beacon[0],beacon[1],str(self.beaconsList.index(beacon)))
        self.printMap()
        outMap = open(mapfile,"w")
        self.printMap(outMap)
        outMap.close()

        #Path file
        outMap = open(pathfile,"w")
        for node in myPath:
            if (node in self.beaconsList) and (self.beaconsList[0] != node):
                s = str(node[0]-28) + " " + str(node[1]-14) + " #"+str(self.beaconsList.index(node))+"\n"
            else:
                s = str(node[0]-28) + " " + str(node[1]-14) + "\n"
            outMap.write(s)
        outMap.close()

        print("finish()")
        self.finish()
        exit(-1)

    def simulateNextCoords(self, inleft, inright):
        self.outleft = (inleft + self.outleft) / 2    # N(1,o^2)
        self.outright = (inright + self.outright) / 2    # N(1,o^2)

        lin = (self.outleft + self.outright) / 2
        theta = (self.measures.compass * 1.570796) / 90
        self.x = self.x + lin * cos(theta)
        self.y = self.y + lin * sin(theta)

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        action = "forward"
        value = None
        path = None
        self.finished = False

        #for testing
        first = True
        
        for i in range(int(self.nBeacons)):
            self.beaconsList.append(None)


        while True:
            self.readSensors()
            #for testing
            if first:
                self.initialX = self.measures.x
                self.initialY = self.measures.y
                first = False

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                [action, value, path] = self.wander(action, value, path)
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                [action, value, path] = self.wander(action, value, path)
            

    def wander(self, action, value, path):
        #print("real: ("+str(self.measures.x - self.initialX + 28)+","+str(self.measures.y - self.initialY + 14) + ") vs sim: (" + str(self.x) + "," + str(self.y) + ")\n")
        #print("sim: (" + str(self.x) + "," + str(self.y) + ")\n")
        #print("front: " + str(self.measures.irSensor[0]) + ", left: "+ str(self.measures.irSensor[1]) + ", right: "+str(self.measures.irSensor[2]))
        
        intrealX = int(self.x)
        intrealY = int(self.y)
        if intrealX % 2 != 0: intrealX += 1
        if intrealY % 2 != 0: intrealY += 1


        dir = self.measures.compass
        closestDir = self.calcClosestDir(dir)
            
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        leftWheel = 0.15
        rightWheel = 0.15
        dirToRotate = 0

        diffY = 0
        if self.y % 2 < 0.41 and self.y % 2 > 0.21 and (closestDir == 0 or closestDir == 180): diffY = -2 #se y=16.3 quero que diminua
        if self.y % 2 < 1.79 and self.y % 2 > 1.59 and (closestDir == 0 or closestDir == 180): diffY = 2  #se y=15.7 quero que aumente


        if action == "forward":
            #print("X: "+str(realX)+", Y: "+str(realY)+", dir: "+str(dir))
            #print("beacons: "+str(self.beaconsList))
            if self.measures.collision:
                #print('Turn around')
                self.simulateNextCoords(0.1,-0.1)
                self.driveMotors(0.1,-0.1)
                return ["turningRight", self.calcTurnAroundDir(dir), [(intrealX,intrealY)]]


            #if ((self.x % 2 < 0.21 or self.x % 2 > 1.79) and (self.y % 2 < 0.31 or self.y % 2 > 1.69)) or ((self.x % 2 < 0.31 or self.x % 2 > 1.69) and (self.y % 2 < 0.21 or self.y % 2 > 1.79)):
            if ((self.x % 2 <= 0.3 or self.x % 2 >= 1.7) and (self.y % 2 <= 0.3 or self.y % 2 >= 1.7)):   
                #Caso seja um sitio novo
                if labMap[intrealY][intrealX] == ' ':
                    #Escrever 'X' no local e escrever as paredes a volta
                    if self.measures.ground != -1:
                        self.beaconsList[self.measures.ground] = (intrealX, intrealY)


                    rob.fillMap(intrealX, intrealY, 'X')
                    rob.fillWalls(intrealX, intrealY, dir, self.measures.irSensor[center_id], self.measures.irSensor[left_id], self.measures.irSensor[right_id])
                    #self.cleanXBetweenWalls()
                    rob.printMap()

                    # CORRIGIR COORDENADAS COM SENSORES LATERAIS
                    if self.measures.irSensor[left_id] >= 2.5:
                        number = 1 / self.measures.irSensor[left_id] 
                        if closestDir == 0:
                            self.y = intrealY - (number - 0.4)
                        elif closestDir == 90:
                            self.x = intrealX + (number - 0.4)
                        elif closestDir == -90:
                            self.x = intrealX - (number - 0.4)
                        else: #180
                            self.y = intrealY + (number - 0.4)

                    elif self.measures.irSensor[right_id] >= 2.5:
                        number = 1 / self.measures.irSensor[right_id] 
                        if closestDir == 0:
                            self.y = intrealY + (number - 0.4)
                        elif closestDir == 90:
                            self.x = intrealX - (number - 0.4)
                        elif closestDir == -90:
                            self.x = intrealX + (number - 0.4)
                        else: #180
                            self.y = intrealY - (number - 0.4)

                    #print("center: "+str(self.measures.irSensor[center_id])+", left: "+str(self.measures.irSensor[left_id])+", right: "+str(self.measures.irSensor[right_id]))
                    #Caso tenha parede a frente
                    if self.measures.irSensor[center_id] >= 1.4:

                        print(self.measures.irSensor[center_id])
                        # CORRIGIR COORDENADAS COM SENSOR FRONTAL
                        number = 1 / self.measures.irSensor[center_id] 
                        if closestDir == 0:
                            self.x = intrealX - (number - 0.4)
                        elif closestDir == 90:
                            self.y = intrealY - (number - 0.4)
                        elif closestDir == -90:
                            self.y = intrealY + (number - 0.4)
                        else: #180
                            self.x = intrealX + (number - 0.4)

                        #Vira 180 graus caso tenha parede a esquerda E parede a direita
                        if self.measures.irSensor[right_id] >= 1.3 and  self.measures.irSensor[left_id] >= 1.3:
                            #print('Turn around')
                            self.simulateNextCoords(0.1,-0.1)
                            self.driveMotors(0.1,-0.1)
                            return ["turningRight", self.calcTurnAroundDir(dir), path]
                        else:
                            #Vira a direita caso nao tenha parede a direita
                            if self.measures.irSensor[right_id] <= 1.1:
                                #print('Rotate Right')
                                self.simulateNextCoords(0.1,-0.1)
                                self.driveMotors(0.1,-0.1)
                                return ["turningRight", self.calcTurnRightDir(dir), path]
                            #Vira a esquerda
                            else :
                                #print('Rotate left')
                                self.simulateNextCoords(-0.1,0.1)
                                self.driveMotors(-0.1,0.1)
                                return ["turningLeft", self.calcTurnLeftDir(dir), path]
                                

                    #Caso não tenha parede a frente, escolhe virar a direita quando pode
                    if self.measures.irSensor[right_id] <= 1.3:
                        #print('Rotate Right')
                        self.simulateNextCoords(0.1,-0.1)
                        self.driveMotors(0.1,-0.1)
                        return ["turningRight", self.calcTurnRightDir(dir), path]

                    #Segue em frente
                    # self.simulateNextCoords(leftWheel,rightWheel)
                    # self.driveMotors(leftWheel,rightWheel)
                    # return ["forward", None, path]
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return ["adjust", 2, path]

                #Caso esteja a seguir um path do a*
                elif path != None:
                    print("@@@")
                    newpath = path
                    if self.measures.irSensor[center_id] >= 0.9:
                        leftWheel = 0.1
                        rightWheel = 0.1

                    #Já chegou ao 'X' destino ou deu astar para si mesmo
                    if len(newpath) == 1: 
                        if self.finished == True and newpath[0] == self.beaconsList[0]:
                            myPath = self.foundOptimalPath()
                            self.finishProgram(myPath[1])

                        rob.fillWalls(intrealX, intrealY, dir, self.measures.irSensor[center_id], self.measures.irSensor[left_id], self.measures.irSensor[right_id])
                        #rob.printMap()
                        #Verificar se precisa de rodar para apontar para a célula ' '
                        dirs = [-90, 90, 180, 0]
                        index = 0
                        print("a")
                        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                            if self.labMap[intrealY+new_position[1]][intrealX+new_position[0]] == ' ':
                                if self.checkCorrectDirAstar(dir,dirs[index]) == False:
                                    if self.checkTurnLeft(dir, dirs[index]):
                                        self.simulateNextCoords(-0.15,-0.15)
                                        self.driveMotors(-0.15,-0.15)
                                        return ["turningLeft", dirs[index], None]
                                    self.simulateNextCoords(-0.15,-0.15)
                                    self.driveMotors(-0.15,-0.15)
                                    return ["turningRight", dirs[index], None]
                                self.simulateNextCoords(leftWheel,rightWheel)
                                self.driveMotors(leftWheel,rightWheel)
                                return ["forward", 1, None]
                            index += 1

                        # minpath = self.getAstarPath(intrealX, intrealY)
                        # print("LEN 0: " + str(minpath))

                        self.simulateNextCoords(leftWheel,rightWheel)
                        self.driveMotors(leftWheel,rightWheel)
                        return ["forward", 1, None]

                    elif (intrealX, intrealY) == path[1] and len(path) >= 2:
                        #Já chegou ao 'X' seguinte
                        newpath = path[1:]
                        print("newpath: " + str(newpath))
                        #Verifica se precisa de virar para ficar de frente para a próxima célula no path
                        dirToRotate = self.calcDirByCoords(path[0], path[1])

                        if self.checkCorrectDirAstar(dir,dirToRotate) == False:
                            if self.checkTurnLeft(dir, dirToRotate):
                                self.simulateNextCoords(-0.1,0.1)
                                self.driveMotors(-0.1,0.1)
                                return ["turningLeft", dirToRotate, newpath]
                            self.simulateNextCoords(0.1,-0.1)
                            self.driveMotors(0.1,-0.1)
                            return ["turningRight", dirToRotate, newpath]

                    else:
                        print("b")
                        #Estou no nó inicial, verificar direção para o caso em que o caminho a* não seja para a frente
                        #Verifica se precisa de virar para ficar de frente para a próxima célula no path
                        if (intrealX, intrealY) == path[0]:
                            dirToRotate = self.calcDirByCoords((intrealX, intrealY), path[1])
                        else:
                            dirToRotate = self.calcDirByCoords((intrealX, intrealY), path[0])
                        print(str(intrealX) + "," + str(intrealY))
                        print(path[0])
                        print(path[1])
                        print(dirToRotate)

                        if self.checkCorrectDirAstar(dir,dirToRotate) == False:
                            if self.checkTurnLeft(dir, dirToRotate):
                                self.simulateNextCoords(-0.1,0.1)
                                self.driveMotors(-0.1,0.1)
                                return ["turningLeft", dirToRotate, newpath]
                            self.simulateNextCoords(0.1,-0.1)
                            self.driveMotors(0.1,-0.1)
                            return ["turningRight", dirToRotate, newpath]

                    print("c")
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return ["forward", 1, newpath]



                #Caso a célula seguinte seja um sitio onde já tenha passado,
                #  tenho de ver os 'X' que ainda têm quadrados adjacentes por descobrir e aplicar a*
                elif self.checkIfFrontOld(intrealX, intrealY, dir):
                    rob.fillWalls(intrealX, intrealY, dir, self.measures.irSensor[center_id], self.measures.irSensor[left_id], self.measures.irSensor[right_id])
                    self.fillIntermediateX(intrealX,intrealY, dir)
                    #rob.printMap()

                    minpath = self.getAstarPath(intrealX, intrealY)
                    print("checkiffrontold " + str(minpath))

                    #Edge case: a* para si mesmo, por ter detetado old path a frente mas ter um ' ' adjacente
                    if len(minpath) == 1:
                        if self.finished == True and minpath[0] == self.beaconsList[0]:
                            myPath = self.foundOptimalPath()
                            self.finishProgram(myPath[1])
                            
                        rob.fillWalls(intrealX, intrealY, dir, self.measures.irSensor[center_id], self.measures.irSensor[left_id], self.measures.irSensor[right_id])
                        #rob.printMap()
                        #Verificar se precisa de rodar para apontar para a célula ' '
                        dirs = [-90, 90, 180, 0]
                        index = 0
                        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                            if self.labMap[intrealY+new_position[1]][intrealX+new_position[0]] == ' ':
                                if self.checkTurnLeft(dir, dirs[index]):
                                    self.simulateNextCoords(-0.15,-0.15)
                                    self.driveMotors(-0.15,-0.15)
                                    return ["turningLeft", dirs[index], None]
                                self.simulateNextCoords(-0.15,-0.15)
                                self.driveMotors(-0.15,-0.15)
                                return ["turningRight", dirs[index], None]
                            index += 1

                    self.simulateNextCoords(0.0,0.0)
                    self.driveMotors(0.0,0.0)
                    return ["forward", 1, minpath]
                
                #Para os ticks depois de escrever o 'X' em que ele ainda está na mesma célula, mas a seguinte está por descobrir (segue em frente)
                else:
                    rob.fillWalls(intrealX, intrealY, dir, self.measures.irSensor[center_id], self.measures.irSensor[left_id], self.measures.irSensor[right_id])
                    #Caso tenha parede a frente
                    if self.measures.irSensor[center_id] >= 1.4:
                        # CORRIGIR COORDENADAS COM SENSOR FRONTAL
                        number = 1 / self.measures.irSensor[center_id] 
                        if closestDir == 0:
                            self.x = intrealX - (number - 0.4)
                        elif closestDir == 90:
                            self.y = intrealY - (number - 0.4)
                        elif closestDir == -90:
                            self.y = intrealY + (number - 0.4)
                        else: #180
                            self.x = intrealX + (number - 0.4)

                        #Vira 180 graus caso tenha parede a esquerda E parede a direita
                        if self.measures.irSensor[right_id] >= 1.3 and  self.measures.irSensor[left_id] >= 1.3:
                            #print('Turn around')
                            self.simulateNextCoords(0.1,-0.1)
                            self.driveMotors(0.1,-0.1)
                            return ["turningRight", self.calcTurnAroundDir(dir), path]
                        else:
                            #Vira a esquerda caso nao tenha parede a esquerda
                            #if self.measures.irSensor[right_id] >= self.measures.irSensor[left_id]:
                            if self.measures.irSensor[right_id] <= 1.1:
                                #print('Rotate Right')
                                self.simulateNextCoords(0.1,-0.1)
                                self.driveMotors(0.1,-0.1)
                                return ["turningRight", self.calcTurnRightDir(dir), path]
                            #Vira a direita
                            else :
                                #print('Rotate left')
                                self.simulateNextCoords(-0.1,0.1)
                                self.driveMotors(-0.1,0.1)
                                return ["turningLeft", self.calcTurnLeftDir(dir), path]

                    self.fillIntermediateX(intrealX,intrealY, dir) # para ficar X de 1 em 1 no mapping.out
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return["forward", 1, path]

            #Não está no centro de nenhuma célula, continua em frente
            else:
                if self.measures.irSensor[center_id] >= 0.9:
                    leftWheel = 0.1
                    rightWheel = 0.1

                self.counter += 1
                
                #Caso tenha parede a frente
                if self.measures.irSensor[center_id] >= 1.7:
                    # CORRIGIR COORDENADAS COM SENSOR FRONTAL
                    number = 1 / self.measures.irSensor[center_id] 
                    if closestDir == 0:
                        self.x = intrealX - (number - 0.4)
                    elif closestDir == 90:
                        self.y = intrealY - (number - 0.4)
                    elif closestDir == -90:
                        self.y = intrealY + (number - 0.4)
                    else: #180
                        self.x = intrealX + (number - 0.4)

                    #Vira 180 graus caso tenha parede a esquerda E parede a direita
                    if self.measures.irSensor[right_id] >= 1.3 and  self.measures.irSensor[left_id] >= 1.3:
                        #print('Turn around')
                        self.simulateNextCoords(0.1,-0.1)
                        self.driveMotors(0.1,-0.1)
                        return ["turningRight", self.calcTurnAroundDir(dir), path]
                    else:
                        #Vira a esquerda caso nao tenha parede a esquerda
                        #if self.measures.irSensor[right_id] >= self.measures.irSensor[left_id]:
                        if self.measures.irSensor[right_id] <= 1.1:
                            #print('Rotate Right')
                            self.simulateNextCoords(0.1,-0.1)
                            self.driveMotors(0.1,-0.1)
                            return ["turningRight", self.calcTurnRightDir(dir), path]
                        #Vira a direita
                        else :
                            #print('Rotate left')
                            self.simulateNextCoords(-0.1,0.1)
                            self.driveMotors(-0.1,0.1)
                            return ["turningLeft", self.calcTurnLeftDir(dir), path]

                #if self.counter % 1 == 0:
                if abs(self.y - intrealY) >= 0.1:
                    if closestDir == 0 :
                        if self.y < intrealY:
                            leftWheel -= 0.01
                        elif self.y > intrealY:
                            rightWheel -= 0.01
                    elif closestDir == 180:
                        if self.y < intrealY:
                            rightWheel -= 0.01
                        elif self.y > intrealY:
                            leftWheel -= 0.01
                if abs(self.x - intrealX) >= 0.1:
                    if closestDir == 90:
                        if self.x < intrealX:
                            rightWheel -= 0.01
                        elif self.x > intrealX:
                            leftWheel -= 0.01
                    else: # -90
                        if self.x < intrealX:
                            leftWheel -= 0.01
                        elif self.x > intrealX:
                            rightWheel -= 0.01


                self.simulateNextCoords(leftWheel,rightWheel)
                self.driveMotors(leftWheel,rightWheel)
                return["forward", 1, path]
        
        elif action == "adjust":
            if value == 0:
                if self.checkIfFrontOld(intrealX, intrealY, dir):
                    self.simulateNextCoords(0.0,0.0)
                    self.driveMotors(0.0,0.0)
                else:
                    self.simulateNextCoords(0.15,0.15)
                    self.driveMotors(0.15,0.15)
                return ["forward", 1, path]

        
            #if value % 2 == 0:
            if closestDir == 180:
                if dir > 0:
                    #print("adjustLeft dir: "+str(dir)+", closest "+str(closestDir)+", diffY "+str(diffY))
                    leftWheel = -0.01
                    rightWheel = 0.01
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return [action, value-1, path]
                elif dir < 0:
                    #print("adjustRight dir: "+str(dir)+", closest "+str(closestDir)+", diffY "+str(diffY))
                    leftWheel = 0.01
                    rightWheel = -0.01
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return [action, value-1, path]
            else:
                if dir < (closestDir+diffY):
                    #print("adjustLeft dir: "+str(dir)+", closest "+str(closestDir)+", diffY "+str(diffY))
                    leftWheel = -0.01
                    rightWheel = 0.01
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return [action, value-1, path]
                elif dir > (closestDir+diffY):
                    #print("adjustRight dir: "+str(dir)+", closest "+str(closestDir)+", diffY "+str(diffY))
                    leftWheel = 0.01
                    rightWheel = -0.01
                    self.simulateNextCoords(leftWheel,rightWheel)
                    self.driveMotors(leftWheel,rightWheel)
                    return [action, value-1, path]
            
            self.simulateNextCoords(0.0,0.0)
            self.driveMotors(0.0,0.0)
            return [action, value-1, path]

        elif action == "turningLeft":
            self.counter = 0
            if self.checkCorrectDir(dir,value) == False:
                #if dir % 15 == 0:
                    #print("turnLeft dir: "+str(dir)+" to "+str(value))
                leftWheel = -0.1
                rightWheel = 0.1
                self.simulateNextCoords(leftWheel,rightWheel)
                self.driveMotors(leftWheel,rightWheel)
                return [action, value, path]
            else:
                self.simulateNextCoords(0.0,0.0)
                self.driveMotors(0.0,0.0)
                return ["adjust", 5, path]


        elif action == "turningRight":
            self.counter = 0
            if self.checkCorrectDir(dir,value) == False:
                #if dir % 15 == 0:
                    #print("turnRight dir: "+str(dir)+" to "+str(value))
                leftWheel = 0.1
                rightWheel = -0.1
                self.simulateNextCoords(leftWheel,rightWheel)
                self.driveMotors(leftWheel,rightWheel)
                return [action, value, path]
            else:
                self.simulateNextCoords(0.0,0.0)
                self.driveMotors(0.0,0.0)
                return ["adjust", 5, path]

        

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
mapfile = "myrob.map"
pathfile = "myrob.path"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        mapfile = sys.argv[i + 1] + ".map"
        pathfile = sys.argv[i + 1] + ".path"
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    labMap = [[' '] * (CELLCOLS*4) for i in range(CELLROWS*4) ]         # 56 a 28
    minpathMap = [['X'] * (CELLCOLS*4) for i in range(CELLROWS*4) ]         # 56 a 28
    rob.setMap(labMap, minpathMap)
    if mapc != None:
        rob.setMap(mapc.labMap, minpathMap)
        rob.printMap()
    
    rob.run()
