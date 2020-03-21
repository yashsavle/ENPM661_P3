# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 23:26:26 2020

@author: prana
"""
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math
from halfplane import*
import datetime
from datetime import datetime as dtime
#import imutils
import time



def inObstacle(x,y,d):
    #print("in obstacle")
    #if int(Y)==115:
        #return True
    #if int(Y)==95:
        #return True
    
    #if int(X)==155:
        #return True
        
    
    if (d*d)-(x*x)>0:
        #print("x not valid")
        return True
        
    if (d*d)-(y*y)>0:
        #print("y not valid")
        return True
        
    if x+d+1>300    or   y+d+1>200:
        #print("x,y out od endzone")
        return True
        
    for i in range(12):
        x1=x+(d*np.cos(2*3.14159/12*(i)))
        y1=y+(d*np.sin(2*3.14159/12*(i)))
        if checkPoints([x1,y1])=="inside":
            #print("yes")
            #print(str([x1,y1])+" out of range")
            return True
    return False
#    if checkPoints([X,Y])=="inside":
#        #print("yes")
#        return True
    
    return False
    
#    if maze[int(Y)][int(X)]==1:
#        return False
#    else:
#        return True
    
    
def dist(current,parent):
    
    dist=np.sqrt(np.square(current[0]-parent[0])+np.square(current[1]-parent[1]))
    return dist
#empty space=0
#visited=3
 
    
def isGoal(current,goal,threshold):
    if current[0]>=goal[0]-threshold   and   current[0]<=goal[0]+threshold  and  current[1]>=goal[1]-threshold   and   current[1]<=goal[1]+threshold:
        return True
    return False

def maze_solver_Astar():
    checkPoints([100,100])
    plt.axis([0, 300, 0, 200])
    print("Sol of Astar")
    #maze=createMaze2()
    #maze=mazeMaker("final")
    frame= np.zeros( (200,300,3), np.uint8 )
    starttime=dtime.now()
                
#    for i in range(300):
#        for j in range(200):
#            if inObstacle(i,j,0.1)==True:
#                frame[j][i]=(0,255,0)
#    cv2.imshow("frame",frame) 
    #cv2.waitKey(0)
#    
    startx=input("Enter x of start co-ordinates:")
    starty=input("Enter y of start co-ordinates:")
    startz=input("Enter z of start co-ordinates:")
    start=[int(startx),200-int(starty),int(startz)]
    
    #start=[50,200-30,60]
    goalx=input("Enter x of Goal co-ordinates:")
    goaly=input("Enter y of Goal co-ordinates:")
    #goalz=input("Enter z of Goal co-ordinates:")
    goal=[int(goalx),200-int(goaly),0]
    #goal=[150,200-150,0]
    #goal=[90,200-50,0]
    robot_diameter=input("Enter robot radius:")
    robot_diameter=2*int(robot_diameter)
    
    clearance=input("Enter clearance value:")
    clearance=int(clearance)
    #robot_diameter=4
    
    robot_diameter=robot_diameter+(2*clearance)    
    out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 600, (300,200))

    if inObstacle(start[0],start[1],robot_diameter)==True:
        print("start in obstacle or outside boundary")
        return
    if inObstacle(goal[0],goal[1],robot_diameter)==True:
        print("goal in obstacle or outside boundary")
        return
    theta=30
    threshold=0.5
    step_size=input("Enter step size:")
    step_size=int(step_size)
    action_set=[]
    if goal[1]-start[1]>0:
        action_set=[int(180/theta),int(360/theta)]
    else:
        action_set=[int(0),int(180/theta)]
#    for i in range(len(maze)):
#        for j in range(len(maze[0])):
#            if maze[i][j]=="O" :
#                start=[i,j]
#            if maze[i][j]=="X" :
#                goal=[i,j]
    maze_size=[300,200,int(360/theta)]
    
    ms1=int(maze_size[0]/threshold)
    ms2=int(maze_size[1]/threshold)
    ms3=int(360/theta)
    
    my_maze=np.zeros((ms1,ms2,ms3))
    my_maze_visited=np.zeros((ms1,ms2))
    
    
    
    s1=int(start[0]/threshold)
    s2=int(start[1]/threshold)
    s3=int(start[2]/theta)
    g1=int(goal[0]/threshold)
    g2=int(goal[1]/threshold)
    g3=int(goal[2]/theta)
    my_maze[s1][s2][s3]=5
    my_maze[g1][g2][g3]=6
    
    

    
    parent=np.zeros((ms1,ms2,ms3,3))              #if error look here at ms2 it was full range rather than /theta
    
    X=np.zeros((ms1))
    Y=np.zeros((ms2))
    #Z=np.zeros((ms3))
    H=np.zeros((ms1,ms2,ms3))
    d_sin = {angle:np.sin(np.deg2rad(angle)) for angle in range(0,361)}
    
    d_cos = {angle:np.cos(np.deg2rad(angle)) for angle in range(0,361)}
    #d_cos = {angle:np.cos(np.deg2rad(angle)) for angle in (0,30,60,90,120,150,180,210,240,270,300,330,360)}
    #print(d_sin[330])
    

#    cv2.circle(frame, (95,170), 5, (0,0,255))
#    cv2.circle(frame, (36,125), 5, (0,0,255))
#    cv2.circle(frame, (100,162), 5, (0,0,255))
    
    for i in range(ms1):
        for j in range(ms2):
            
                [X[i],Y[j]]=[i,j]
                #temp=np.sqrt(np.square(X[i]*threshold-goal[0])+np.square(Y[j]*threshold-goal[1]))
                temp=(abs(X[i]*threshold-goal[0])+abs(Y[j]*threshold-goal[1]))
                
                H[i][j]=temp
    
    
    g=np.full((ms1,ms2,ms3),float('inf'))
    f=np.full((ms1,ms2,ms3),float('inf'))
    
    
    
    
    g[s1][s2][s3]=0
    f[s1][s2][s3]=H[s1][s2][s3]
    numExpanded=0
    expanded=[]
    expanded.append(start)
    
    
    current=start
    goal_current=start
    prev_current=start
    
    print("started to solve")
    while 1:

        
        min_dist=float('inf')

        
        for c in expanded:
            if min_dist>f[int(c[0]/threshold)][int(c[1]/threshold)][int(c[2]/theta)]:
                min_dist=f[int(c[0]/threshold)][int(c[1]/threshold)][int(c[2]/theta)]
                current=c
            
            
        #print("current")
        #print(current)            
        #cv2.circle(frame, (int(current[0]),int(current[1])), robot_diameter+1, (0,0,255))
        #cv2.circle(frame, (int(prev_current[0]),int(prev_current[1])), robot_diameter, (0,0,0))

        c1=int(current[0]/threshold)
        c2=int(current[1]/threshold)
        c3=int(current[2]/theta)
        #print(current)
        if isGoal(current,goal,(robot_diameter+step_size)/2)==True  or min_dist==float('inf'):
            goal_current=current
            print("goal reached")
            break
        
        
        #my_maze[c1][c2][c3]=3             #visited
        my_maze_visited[c1][c2]=1
        
        #plt.plot(int(current[0]),int(current[1]),'yo',markersize=1)
        frame[int(current[1])][int(current[0])]=(255,0,0)           #Mark blue
        expanded.remove(current)
        f[c1][c2][c3]=float('inf')
        
        
        adjacent=[]
        for i in range(0,int(360/theta)):
            angle=int(current[2]+(theta*i))
            if angle>359:
                #print(angle)
                angle=angle-360
            X=current[0]+(step_size*d_cos[int(angle)])
            Y=current[1]+(step_size*d_sin[int(angle)])
            
            t=theta*i+current[2]
            if t>359:
                t=t-360
            if  X<300   and X>0   and Y>0   and Y<200 and inObstacle(X,Y,robot_diameter/2)==False  and my_maze_visited[int(X/threshold)][int(Y/threshold)]!=1  :#and     my_maze[int(X/threshold)][int(Y/threshold)][int(t/theta)]!=3  and    my_maze[int(X/threshold)][int(Y/threshold)][int(t/theta)]!=4    :
                adjacent.append([X,Y,t])
                #cv2.line(frame,(int(X),int(Y)),(int(current[0]),int(current[1])), (255,0,0), 1)
    
            #else:
                #adjacent.append(current)

        temp=0
        
        for n in range(len(adjacent)):
            adj1=int(adjacent[n][0]/threshold)
            adj2=int(adjacent[n][1]/threshold)
            adj3=int(adjacent[n][2]/theta)
            
            if (my_maze [adj1] [adj2][adj3] ==0   or   my_maze [adj1] [adj2][adj3] ==4     or  my_maze[adj1][adj2][adj3]==6)   :
                if  g[adj1][adj2][adj3] > g[c1][c2][c3]+dist(current,adjacent[n]):
                    g[adj1][adj2][adj3]=g[c1][c2][c3]+dist(current,adjacent[n])
                    f[adj1][adj2][adj3]=g[adj1][adj2][adj3]+H[adj1][adj2][adj3]
                    #print("ASTAR")
                    parent[adj1][adj2][adj3] =current
                    temp=n
                    #cv2.line(frame,(int(X),int(Y)),(int(current[0]),int(current[1])), (255,0,0), 2)
    
                    #plt.plot(int(adjacent[n][0]),int(adjacent[n][1]),'bo',markersize=1)
                    frame[int(adjacent[n][1])][int(adjacent[n][0])]=(255,255,255)
                    if my_maze_visited[adj1][adj2]!=1  and  my_maze[adj1][adj2][adj3]!=4  and  my_maze[adj1][adj2][adj3]!=3  :
                        expanded.append(adjacent[n])
                
                    my_maze[adj1][adj2][adj3]=4        #explored
                
                
                
                
        numExpanded=numExpanded+1
        #print(adjacent)
        #current=adjacent[temp]
        cv2.imshow("frame",frame)
        out.write(frame)

        prev_current=current
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
          
        
        #print(parent)
        
    print(np.shape(parent))
    route=[]
    if f[int(goal_current[0]/threshold)][int(goal_current[1]/threshold)][int(goal_current[2]/theta)]==float('inf'):
        route=[]
        print("route cannot be found")
    else:
        route=[goal_current]
        #print(([route[len(route)-1][0]][route[len(route)-1][1]][0]))
        #print(int(parent[route[len(route)-1][0]][route[len(route)-1][1]][1]))
        a=int(route[len(route)-1][0]/threshold)
        b=int(route[len(route)-1][1]/threshold)
        c=int(route[len(route)-1][2]/theta)
        #print(parent[int(a/threshold)][int(b/threshold)][int(c/theta)])
        while parent[a][b][c][0]!=0  or  parent[a][b][c][1]!=0   or    parent[a][b][c][2]!=0:
            a=int(route[len(route)-1][0]/threshold)
            b=int(route[len(route)-1][1]/threshold)
            c=int(route[len(route)-1][2]/theta)
            #print(parent[int(a/threshold)][int(b/threshold)][int(c/theta)])
            #print([int(a/threshold),int(b/threshold),int(c/theta)])
            c1=parent[a][b][c][0]
            c2=parent[a][b][c][1]
            c3=parent[a][b][c][2]
            #print([c1,c2,c3])
            #plt.plot(c1,c2,'ro',markersize=1)
            frame[int(c2)][int(c1)]=(0,255,0)
            route.append([c1,c2,c3])
           
    #print(route)
    
    for p in route:
        print(p)
        
    for p in range(len(route)-2-1,-1,-1):
        cv2.line(frame,(int(route[p][0]),int(route[p][1])),(int(route[p+1][0]),int(route[p+1][1])), (0,0,255), 2)
        out.write(frame)

    print(numExpanded)
    endtime=dtime.now()
    print([starttime,endtime])
    
    cv2.imshow("frame",frame)
    out.write(frame)
    out.release()
    cv2.waitKey(0)   
    cv2.destroyAllWindows() 
    
    
#    for i in route:
#        maze[int(i[0])][int(i[1])]="="
#    maze[start[0]][start[1]]="O"
#    maze[goal[0]][goal[1]]="X"
    #print(maze)
   
maze_solver_Astar()