import numpy as np


def getIntercept(x1,y1,x2,y2):
    m=(y2-y1)/(x2-x1)

    b=y1-m*x1
    return b

def getSlope(x1,y1,x2,y2):
    return (y2-y1/(x2-x1))
def checkHalfPlane(X0, Y0, x, y):
    a = (X0[1] - Y0[1])/(X0[0] - Y0[0])
    b = X0[1] - a*X0[0]
    
    check = y - b - a*x
    
    if check <= 0:
        return 1
    return 0

def checkCurves(point):
    x_dash=(1-np.square((point[0]-150)/40))
    if x_dash>0:
        y1=100-20*np.sqrt(x_dash)
        y2=100+20*np.sqrt(x_dash)
        if point[0]>=110   and   point[0]<=190   and point[1]>=y1   and point[1]<=y2:
            #print()
            return "inside"
    
    
    #for circle
    x_dash=225-np.square(point[0]-225)
    if x_dash>0:
        y1=50-np.sqrt(x_dash)
        y2=50+np.sqrt(x_dash)
        if point[0]>=200  and  point[0]<=250 and   point[1]>=y1   and  point[1]<=y2:
            return "inside"
    return "outside"