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
