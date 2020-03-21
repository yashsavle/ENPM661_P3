import numpy as np


def getIntercept(x1,y1,x2,y2):
    m=(y2-y1)/(x2-x1)

    b=y1-m*x1
    return b

def getSlope(x1,y1,x2,y2):
    return (y2-y1/(x2-x1))
