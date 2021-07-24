import pygame
import Raycasting
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

#Color del piso
background_color = (210,218,226)

playerImg = pygame.image.load('../PeaShooterResizeOpen.png')

obstacleImg = pygame.image.load('../Obstacle.png')


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

def find_rotation_degrees(mouse_X, mouse_Y, center_X, center_Y, radians):
    degree = (radians * (180 / 3.1415) * -1) + 90
    return degree

# método para determinar cuántos radianes rotar la imagen del jugador
def find_rotation_radians(mouse_X, mouse_Y, center_X, center_Y):
    radians = math.atan2(mouse_Y - center_Y, mouse_X - center_X)
    return radians

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

def scan_obstacle_file(file_path,obstacleImg):
    file = open(file_path, "r")
    if file.mode == 'r':
        file_line = file.readlines()
        read_window_size(file_line[0])

        for line in file_line[1:]:
            # ahora solo separe x, y, ancho, alto
            # y agregue cada línea como un parámetro de obstáculo cuadrado 
            value = line.split(',')
            square_obstacle(int(value[0]), int(value[1]), int(value[2]), int(value[3]), obstacleImg)


def read_window_size(first_line):
    list = first_line.split(',')
    global displayWidth
    global displayHeight
    displayWidth = int(list[0])
    displayHeight = int(list[1])
    screen = pygame.display.set_mode((displayWidth,displayHeight))



x = (displayWidth * .45)
y = (displayHeight * .8)

x_change = 0
y_change = 0

# Crea el mapa a traves del documento
scan_obstacle_file("../Arena1.txt",obstacleImg)

#Variable que hace referencia a que el juego está en marcha
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


    # print(obstacleList)
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
    Raycasting.draw_line_of_sight(mouseX, mouseY, centerX, centerY,screen,obstacleList,displayWidth,displayHeight)
    circleArr = Raycasting.find_cicle(centerX, centerY, 100, radians)
    Raycasting.draw_cone_line_of_sight(centerX, centerY, circleArr,screen,obstacleList,displayWidth,displayHeight)
    playerImgRotated = pygame.transform.rotate(playerImg, degrees)
    screen.blit(playerImgRotated,(x,y))
    clock.tick(50)
    pygame.display.update()


pygame.quit()