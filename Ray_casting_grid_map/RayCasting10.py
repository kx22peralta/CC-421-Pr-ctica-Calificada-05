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


# playerImg = pygame.image.load('C:\Users\frank\Desktop\Ray_casting_grid_map\PeaShooterResizeOpen.png')

# obstacleImg = pygame.image.load('C:\Users\frank\Desktop\Ray_casting_grid_map\Obstacle.png')

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

#Carga el juego y evita que se cierre(loop), aqui se comprueban y gestionan los eventos y acciones
def game_loop():
    x = (displayWidth * .45)
    y = (displayHeight * .8)

    x_change = 0
    y_change = 0

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

        clock.tick(50)
        pygame.display.update()

game_loop()
pygame.quit()