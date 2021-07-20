import pygame
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((400,600))
pygame.display.set_caption("Dynamic window approach")

auto = pygame.image.load("auto.png")
r= 1
while True:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
    r+=1
    screen.fill('black')
    screen.blit(auto,  ( r, r)  )
    pygame.display.update()
    clock.tick(60)