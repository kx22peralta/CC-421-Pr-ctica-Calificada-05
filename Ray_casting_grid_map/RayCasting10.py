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


playerImg = pygame.image.load('C:\Users\frank\Desktop\Ray_casting_grid_map\PeaShooterResizeOpen.png')

obstacleImg = pygame.image.load('C:\Users\frank\Desktop\Ray_casting_grid_map\Obstacle.png')

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


