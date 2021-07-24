import pygame
import math

prevCenterX = 0
prevCenterY = 0
prevMouseX = 0
prevMouseY = 0
prevLOSX = 0
prevLOSY = 0

#Color de las lineas raycast
lineColor = pygame.Color(237,76,103)

# método para determinar todos los puntos que forman un círculo en un espacio 2d.
# parámetros son (coord x, coord y, radio, radianes rotados). devuelve una matriz de puntos 
def find_cicle(x, y, radius, radiansRotated):
    radiansRotated = -radiansRotated + math.pi/2
    rangeArr = [radiansRotated + math.pi/4, radiansRotated + math.pi/6,
    radiansRotated + math.pi/16, radiansRotated - math.pi/16,
    radiansRotated + math.pi/8, radiansRotated - math.pi/8,radiansRotated - math.pi/6, radiansRotated - (math.pi/4)]
    returnArr = []
    for index in rangeArr:
        return_X = y + (radius * math.cos(index))
        return_Y = x + (radius * math.sin(index))
        return_X = int(round(return_X))
        return_Y = int(round(return_Y))
        returnArr.append([return_X, return_Y])
    return returnArr


# Dibujar las lineas raycast, puede hacer un simil con la vision del player
def draw_line_of_sight(mouse_X, mouse_Y, center_X, center_Y,screen,obstacleList,displayWidth,displayHeight):
    global prevMouseX
    global prevMouseY
    global prevCenterX
    global prevCenterY
    global prevLOSX
    global prevLOSY

    if(mouse_X == prevMouseX and mouse_Y == prevMouseY and center_X == prevCenterX and center_Y == prevCenterY):
        pygame.draw.line(screen, lineColor, (prevCenterX, prevCenterY), (prevLOSX, prevLOSY), 3)
        return

    lineThickness = 1

    prevMouseX = mouse_X
    prevMouseY = mouse_Y
    prevCenterX = center_X
    prevCenterY = center_Y
    noCollisionFound = True
    threshold = 11
    index = 0

    if mouse_X <= 0:
        mouse_X = 1
    if  mouse_Y <= 0:
        mouse_Y = 1

    #Pendiente del caycast(vision del player)
    slope = (center_Y - mouse_Y) / (center_X - mouse_X)

    if mouse_X > center_X:
        while noCollisionFound:
            yCoord = (slope * index) + center_Y
            if check_obstacle_collision(index + center_X, yCoord,obstacleList):
                pygame.draw.line(screen, lineColor, (center_X, center_Y), (index + center_X, yCoord), lineThickness)
                prevLOSX = index + center_X
                prevLOSY = yCoord
                noCollisionFound = False
            elif check_screen_collision(index + center_X, yCoord,displayWidth,displayHeight):
                pygame.draw.line(screen, lineColor, (center_X, center_Y), (index + center_X, yCoord), lineThickness)
                prevLOSX = index + center_X
                prevLOSY = yCoord
                noCollisionFound = False
            else :
                index = index + 1

    else:
        while noCollisionFound:
            yCoord = (-slope * index) + center_Y
            # print(check_obstacle_collision(center_X - index, yCoord,obstacleList))
            if check_obstacle_collision(center_X - index, yCoord,obstacleList):
                pygame.draw.line(screen, lineColor, (center_X, center_Y), (center_X - index, yCoord), lineThickness)
                prevLOSX = center_X - index
                prevLOSY = yCoord
                noCollisionFound = False
            elif check_screen_collision(center_X - index, yCoord,displayWidth,displayHeight):
                pygame.draw.line(screen, lineColor, (center_X, center_Y), (center_X -index, yCoord), lineThickness)
                prevLOSX = center_X - index
                prevLOSY = yCoord
                noCollisionFound = False
            else :
                index = index + 1


#Las 2 funciones siguientes limitan las lineas y evita que traspacen un bloque o dimensión de la pantalla
#Dado un punto, devuelve True si cumple con las dimensiones del obstáculo
def check_obstacle_collision(los_X, los_Y,obstacleList):
    for box in obstacleList:
        if(los_X >= box.x and los_X <= box.x + box.width):

            if(los_Y >= box.y and los_Y <= box.y + box.height):
                return True
    else:
        return False

#Dado un punto, devuelve verdadero si ha salido de los limites de la pantalla
def check_screen_collision(los_X, los_Y,displayWidth,displayHeight):
    if(los_X <= 0 or los_X >= displayWidth):
        return True
    if(los_Y <= 0 or los_Y >= displayHeight):
        return True
    else :
        return False


# dado un punto (centro de playerImg) y (int) d grado en radianes, creará un círculo a su alrededor y devolverá un punto a lo largo del
#circunferencia del círculo (45 grados + d)
def draw_cone_line_of_sight(center_X, center_Y, circlePointList,screen,obstacleList,displayWidth,displayHeight):
    for circleCoordPair in circlePointList:
        x = circleCoordPair[1]
        y = circleCoordPair[0]
        draw_line_of_sight(x, y, center_X, center_Y,screen,obstacleList,displayWidth,displayHeight)
