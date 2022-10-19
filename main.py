import pygame

pygame.init()

screen = pygame.display.set_mode([800,800])

running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255,255,255))
    screenWidth = screen.get_width()

    pygame.draw.rect(screen,(20,20,125),(int(screenWidth/2) - 300,int(screenWidth/2) - 300,550,550))
    pygame.draw.rect(screen,(100,100,100),(500,20,50,50))
    pygame.display.flip()

pygame.quit()