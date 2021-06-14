import math

def getAngleAndDirection(point1,point2):
    p1x = point1[0]
    p1y = point1[1]
    p2x = point2[0]
    p2y = point2[1]

    xDef = p2x - p1x
    yDef = p2y - p1y

    angle = math.degrees(math.atan2(yDef,xDef))

    if angle >= 0 and angle<= 90:
        quad = 1
    elif angle >90 and angle<= 180:
        quad = 2
    elif angle >180 and angle<= 270:
        quad = 3
    else:
        quad = 4

    return angle , quad


if __name__ == '__main__':
    point1 = [0,0]
    point2 = [1,-1]

    print(getAngleAndDirection(point1,point2))