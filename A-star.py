import matplotlib.pyplot as plt
import math
from math import*

def checkActions(x,y,t,d,obstacleSpace):
    available = []
#st
n_x= x+d*math.cos(t)
n_y= y+d*math.sin(t)
if obstacleSpace[n_x][n_y] == 0:
        available.append([n_x,n_y,t+0])

#l30
n_x= x+d*math.cos(t+math.radians(30))
n_y= y+d*math.sin(t+math.radians(30))
if obstacleSpace[n_x][n_y] == 0:
        available.append([n_x,n_y,t+30])

#l60
n_x= x+d*math.cos(t+math.radians(60))
n_y= y+d*math.sin(t+math.radians(60))
if obstacleSpace[n_x][n_y] == 0:
        available.append([n_x,n_y,t+60])

#l-30
n_x= x+d*math.cos(t+math.radians(-30))
n_y= y+d*math.sin(t+math.radians(-30))
if obstacleSpace[n_x][n_y] == 0:
        available.append([n_x,n_y,t-30])

#l-60
if obstacleSpace[n_x][n_y] == 0:
        available.append([n_x,n_y,t-60])        
return available

def sortList(nodeList, index): 
    for i in range(0, len(nodeList)): 
        for j in range(0, len(nodeList)-i-1): 
            if (nodeList[j][index] > nodeList[j + 1][index]): 
                temp = nodeList[j] 
                nodeList[j]= nodeList[j + 1] 
                nodeList[j + 1]= temp 
    return nodeList 

def checkHalfPlane(X0, Y0, x, y):
    a = (X0[1] - Y0[1])/(X0[0] - Y0[0])
    b = X0[1] - a*X0[0]
    
    check = y - b - a*x
    
    if check <= 0:
        return 1
    return 0

def checkHalfPlane(X0, Y0, x, y):
    a = (X0[1] - Y0[1])/(X0[0] - Y0[0])
    b = X0[1] - a*X0[0]
    
    check = y - b - a*x
    
    if check <= 0:
        return 1
    return 0

# Check for obstcale (return 1 if obstacle, 0 if not)
def checkObs(x,y):
#     Circle
    circle = (x-225)**2 + (y-150)**2 -(25**2)
    if circle <=0:
        return 1
#     Elipse
    ellipse= ((x-150)/40)**2 + ((y-100)/20)**2 -1
    if ellipse<=0:
            return 1
    
#     Rectangle
    X=[95,30]
    Y=[95-(cos(pi/6)*75), 30+(75*sin(pi/6))]
    Z=[95+(10*sin(pi/6)), 30+(10*cos(pi/6))]
    W=[Z[0]-(cos(pi/6)*75), Z[1]+(75*sin(pi/6))]
    
    if checkHalfPlane(X, Y, x, y)==0 and checkHalfPlane(X, Z, x, y)==0 and checkHalfPlane(Y, W, x, y)==1 and checkHalfPlane(Z, W, x, y)==1:
            return 1

    
#    Rhombus
    A = [225,10]
    B = [250,25]
    C = [225,40]
    D = [200,25]
    
    if checkHalfPlane(A, B, x, y)==0 and checkHalfPlane(B, C, x, y)==1 and checkHalfPlane(C, D, x, y)==1 and checkHalfPlane(A, D, x, y)==0:
            return 1

        
#     Polygon
    P1 = [20, 120]
    P2 = [25, 185]
    P3 = [75, 185]
    P4 = [100, 150]
    P5 = [75, 120]
    P6 = [50, 150]
# Check Triange 1 (P1, P2, P6)
    if checkHalfPlane(P1, P2, x, y)==1 and checkHalfPlane(P2, P6, x, y)==1 and checkHalfPlane(P1, P6, x, y)==0:
            return 1
# Check Triange 2 (P2, P3, P6)
    if checkHalfPlane(P3, P2, x, y)==1 and checkHalfPlane(P2, P6, x, y)==0 and checkHalfPlane(P3, P6, x, y)==0:
            return 1
# Check Triange 3 (P4, P3, P6)
    if checkHalfPlane(P3, P4, x, y)==1 and checkHalfPlane(P4, P6, x, y)==0 and checkHalfPlane(P3, P6, x, y)==1:
            return 1
# Check Triange 3 (P4, P5, P6)
    if checkHalfPlane(P5, P4, x, y)==0 and checkHalfPlane(P4, P6, x, y)==1 and checkHalfPlane(P5, P6, x, y)==0:
            return 1
    
    return 0

# Generate Map (including obstacles)
def generateMap():
    mapList = []
    rowList = []
    for x in range(300):
        for y in range(200):
            if checkObs(x, y) == 1:
                rowList.append(1)
            else:
                rowList.append(0)

        
        mapList.append(rowList)
        rowList = []
        
    return mapList

    # Generate Map + Obstacle Clearance (Radius and Clearance of Agent)
def generateObstacleSpace(r, c, obstacleMap):
    circleList = []
    rowList = []
    obstacleSpace = copy.deepcopy(obstacleMap)
    
    for x in range(2*(r+c)+1):
        for y in range(2*(r+c)+1):
            circle = (x-(r+c))**2 + (y-(r+c))**2 - (r+c+0.5)**2
            if circle <= 0:
                rowList.append(1)
            else:
                rowList.append(0)
            
        circleList.append(rowList)
        rowList = []
        
    for row in range(len(obstacleMap)):
        for column in range(len(obstacleMap[row])):
            
            if obstacleMap[row][column] == 1:
                
                for inColumn in range(len(circleList)):
                    for inRow in range(len(circleList)):
                        if row+inRow-(r+c) < len(obstacleMap) and column+inColumn-(r+c) < len(obstacleMap[row+inRow-(r+c)]):
                            if obstacleMap[row+inRow-(r+c)][column+inColumn-(r+c)] == 0 and circleList[inRow][inColumn]:
                                obstacleSpace[row+inRow-(r+c)][column+inColumn-(r+c)] = 2
                            
            if row <= r+c-1:
                if obstacleSpace[row][column] == 0:
                    obstacleSpace[row][column] = 2
                
            if row >= len(obstacleMap)-(r+c):
                if obstacleSpace[row][column] == 0:
                    obstacleSpace[row][column] = 2
                
            if column <= r+c-1:
                if obstacleSpace[row][column] == 0:
                    obstacleSpace[row][column] = 2
                
            if column >= len(obstacleMap[row])-(r+c):
                if obstacleSpace[row][column] == 0:
                    obstacleSpace[row][column] = 2
 
    return obstacleSpace

def astar(startNode, goalNode, obstacles)
    nodes = []
    nextNodes=[]
    Q=[]
    Vn=[]
    #   Variable to check if there is a new node with a lowest cost    
    newLowerCost = 1;
    run = 0
    while len(Q)>0:
        if newLowerCost:
            sortList(Q,1)