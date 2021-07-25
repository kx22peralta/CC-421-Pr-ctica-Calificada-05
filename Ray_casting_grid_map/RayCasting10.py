import pygame
import math


pygame.init()
clock = pygame.time.Clock()
obstacleList = []

# construir ventana de visualización
displayWidth = 800
displayHeight = 650

prevCenterX = 0
prevCenterY = 0
prevMouseX = 0
prevMouseY = 0
prevLOSX = 0
prevLOSY = 0

screen = pygame.display.set_mode((displayWidth,displayHeight))
pygame.display.set_caption('RayCasting in Python')
background_color = (255,255,255)
black = pygame.Color(0,0,0)

playerImg = pygame.image.load('PeaShooterResizeOpen.png')

obstacleImg = pygame.image.load('Obstacle.png')


playerImg = pygame.transform.scale(playerImg, (31, 31))


def player(x,y):
    screen.blit(playerImg, (x,y))

# construir clase para mantener las coordenadas de obstáculos 
class Rectangle:
    def __init__(self, _x, _y, _width, _height):
        self.x = _x
        self.y = _y
        self.width = _width
        self.height = _height
    def __eq__(self, other):
        if isinstance(other, Rectangle):
            if(self.x == other.x and self.y == other.y and self.height == other.height and self.width == other.width):
                return True
            else:
                return False

# construir obstrucción
def square_obstacle(obs_x, obs_y, obs_width, obs_height, _obstacleImg):
    current = Rectangle(obs_x, obs_y, obs_width, obs_height)

    if(len(obstacleList) == 0):
        obstacleList.append(current)
    # comprobar si hay cajas duplicadas
    duplicate_found = False
    for box in obstacleList:
        if current.__eq__(box):
            duplicate_found = True
        else:
            duplicate_found = False

    if duplicate_found == False:
        obstacleList.append(current)
        print(str(len(obstacleList)) + str(" ")+str(obs_x) +str(" ")+ str(obs_y)+str(" ")+ str(obs_width)+str(" ")+ str(obs_height))
# lee de C:\Users\frank\Documents\GitHub\CC-421-Practica-Calificada-05\Ray_casting_grid_map\Arena1.txt para completar el mapa de obstáculos y crear un mapa
# la primera línea leída contiene coordenadas que representan el tamaño de ventana deseado
# cada línea a partir de entonces contiene un obstáculo cuadrado "x posición inicial, y posición inicial, ancho, alto"
# NO incluya paréntesis para el tamaño de la ventana u obstáculos 
def scan_obstacle_file(file_path):
    file = open(file_path, "r")
    if file.mode == 'r':
        file_line = file.readlines()
        read_window_size(file_line[0])

        for line in file_line[1:]:
            # ahora solo separe x, y, ancho, alto 
            # y agregue cada línea como un parámetro de obstáculo cuadrado 
            value = line.split(',')
            square_obstacle(int(value[0]), int(value[1]), int(value[2]), int(value[3]), obstacleImg)

# divide la cadena por, dividiendo screenWidth y screenHeight y luego establece los valores
def read_window_size(first_line):
    list = first_line.split(',')
    global displayWidth
    global displayHeight
    displayWidth = int(list[0])
    displayHeight = int(list[1])
    screen = pygame.display.set_mode((displayWidth,displayHeight))

def find_rotation_degrees(mouse_X, mouse_Y, center_X, center_Y, radians):
    degree = (radians * (180 / 3.1415) * -1) + 90
    return degree

# método para determinar cuántos radianes rotar la imagen del jugador
def find_rotation_radians(mouse_X, mouse_Y, center_X, center_Y):
    radians = math.atan2(mouse_Y - center_Y, mouse_X - center_X)
    return radians


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
def draw_line_of_sight(mouse_X, mouse_Y, center_X, center_Y):
    global prevMouseX
    global prevMouseY
    global prevCenterX
    global prevCenterY
    global prevLOSX
    global prevLOSY

    if(mouse_X == prevMouseX and mouse_Y == prevMouseY and center_X == prevCenterX and center_Y == prevCenterY):
        pygame.draw.line(screen, black, (prevCenterX, prevCenterY), (prevLOSX, prevLOSY), 3)
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
            if check_obstacle_collision(index + center_X, yCoord):
                pygame.draw.line(screen, black, (center_X, center_Y), (index + center_X, yCoord), lineThickness)
                prevLOSX = index + center_X
                prevLOSY = yCoord
                noCollisionFound = False
            elif check_screen_collision(index + center_X, yCoord):
                pygame.draw.line(screen, black, (center_X, center_Y), (index + center_X, yCoord), lineThickness)
                prevLOSX = index + center_X
                prevLOSY = yCoord
                noCollisionFound = False
            else :
                index = index + 1

    else:
        while noCollisionFound:
            yCoord = (-slope * index) + center_Y
            if check_obstacle_collision(center_X - index, yCoord):
                pygame.draw.line(screen, black, (center_X, center_Y), (center_X - index, yCoord), lineThickness)
                prevLOSX = center_X - index
                prevLOSY = yCoord
                noCollisionFound = False
            elif check_screen_collision(center_X - index, yCoord):
                pygame.draw.line(screen, black, (center_X, center_Y), (center_X -index, yCoord), lineThickness)
                prevLOSX = center_X - index
                prevLOSY = yCoord
                noCollisionFound = False
            else :
                index = index + 1


#Las 2 funciones siguientes limitan las lineas y evita que traspacen un bloque o dimensión de la pantalla
#Dado un punto, devuelve True si cumple con las dimensiones del obstáculo
def check_obstacle_collision(los_X, los_Y):
    for box in obstacleList:
        if(los_X >= box.x and los_X <= box.x + box.width):
            if(los_Y >= box.y and los_Y <= box.y + box.height):
                return True
    else:
        return False

#Dado un punto, devuelve verdadero si ha salido de los limites de la pantalla
def check_screen_collision(los_X, los_Y):
    if(los_X <= 0 or los_X >= displayWidth):
        return True
    if(los_Y <= 0 or los_Y >= displayHeight):
        return True
    else :
        return False


# dado un punto (centro de playerImg) y (int) d grado en radianes, creará un círculo a su alrededor y devolverá un punto a lo largo del
#circunferencia del círculo (45 grados + d)
def draw_cone_line_of_sight(center_X, center_Y, circlePointList):
    for circleCoordPair in circlePointList:
        x = circleCoordPair[1]
        y = circleCoordPair[0]
        draw_line_of_sight(x, y, center_X, center_Y)


#Carga el juego y evita que se cierre(loop), aqui se comprueban y gestionan los eventos y acciones
def game_loop():
    x = (displayWidth * .45)
    y = (displayHeight * .8)

    x_change = 0
    y_change = 0

    # Crea el mapa a traves del documento
    scan_obstacle_file("Arena1.txt")

    #Variable que hace referncia a que el juego está en marcha
    running = True
    #Acciones mientras corre el juego(movimiento y accion de salir)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                key = pygame.key.get_pressed()
                if key[pygame.K_LEFT] or key[pygame.K_a]:
                    x_change = - 5
                if key[pygame.K_RIGHT] or key[pygame.K_d]:
                    x_change = + 5
                if key[pygame.K_UP] or key[pygame.K_w]:
                    y_change = - 5
                if key[pygame.K_DOWN] or key[pygame.K_s]:
                    y_change = + 5
                if key[pygame.K_ESCAPE]:
                    pygame.quit()
            if event.type == pygame.KEYUP:
                if key[pygame.K_LEFT] or key[pygame.K_RIGHT] or key[pygame.K_a] or key[pygame.K_d]:
                    x_change = 0
                if key[pygame.K_UP] or key[pygame.K_DOWN] or key[pygame.K_s] or key[pygame.K_w]:
                    y_change = 0

        #Variación de los ejes para el movimiento del player
        x += x_change
        y += y_change

        # Se definen los límites para que el player no salga de la ventana
        if x >= displayWidth - playerImg.get_width() or x < 0:
            x -= x_change
        if y >= displayHeight - playerImg.get_height() or y < 0:
            y -= y_change

  # Se definen los limites del player con los obstaculos
        playerHeight = playerImg.get_height()
        playerWidth = playerImg.get_width()
        for box in obstacleList:
            if y < box.y + box.height and y + playerHeight > box.y:
                if x > box.x and x < box.x + box.width or x + playerWidth > box.x and x + playerWidth < box.x + box.width:
                    y -= y_change
                    x -= x_change

    # Ajustes finales en cuanto al screen y los obstáculos
        screen.fill(background_color)
        for box in obstacleList:
            obstacleImgScaled = pygame.transform.scale(obstacleImg, (box.width, box.height))
            screen.blit(obstacleImgScaled,(box.x, box.y))

        # Se rota al player según el mouse
        mouseX = pygame.mouse.get_pos()[0]
        mouseY = pygame.mouse.get_pos()[1]

        centerX = x + (playerImg.get_width()/2)
        centerY = y + (playerImg.get_height()/2)


        radians = find_rotation_radians(mouseX, mouseY, centerX, centerY)
        degrees = find_rotation_degrees(mouseX, mouseY, centerX, centerY, radians)
        draw_line_of_sight(mouseX, mouseY, centerX, centerY)
        circleArr = find_cicle(centerX, centerY, 100, radians)
        draw_cone_line_of_sight(centerX, centerY, circleArr)
        playerImgRotated = pygame.transform.rotate(playerImg, degrees)
        screen.blit(playerImgRotated,(x,y))


        clock.tick(50)
        pygame.display.update()

game_loop()
pygame.quit()