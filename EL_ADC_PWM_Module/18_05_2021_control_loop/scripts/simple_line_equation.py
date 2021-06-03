#!/usr/bin/python3
pointA = (4.0,1)
pointB = (5.0,750)

dX = pointB[0] - pointA[0]
dY = pointB[1] - pointA[1]
m = dY/dX
c = pointA[1] - m*pointA[0]

print("y = {} * x + {}".format(m,c))