import pygame

pygame.init()

screen = pygame.display.set_mode([800,800])

running = True

class Boat:
    def __init__(self,rectSize,startPos,Bscreen,Bscolor,atualPos = 0):
        self.rectSize = rectSize
        self.startPos = startPos
        self.atualPos = startPos
        self.screen = Bscreen
        self.color = Bscolor
    def drawBoat(self):
        pygame.draw.rect(self.screen,self.color,(self.startPos[0],self.startPos[1],self.rectSize[0],self.rectSize[1]))
    

barco = Boat((20,20),(550,550),screen,(255,255,255))

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255,255,255))
    rectSize = 50
    screenWidth = screen.get_width()

    pygame.draw.rect(screen,(20,20,125),(int(screenWidth/2) - 300,int(screenWidth/2) - 300,rectSize*11,rectSize*11))
    pygame.draw.rect(screen,(100,100,100),(500,20,rectSize,rectSize))
    pygame.draw.rect(screen,(100,100,100),(20,400,rectSize,rectSize))
    pygame.draw.circle(screen,(0,255,0),(20+rectSize/2,400+rectSize/2),20)
    pygame.draw.circle(screen,(0,255,0),(500+rectSize/2,20+rectSize/2),20)
    barco.drawBoat()

    pygame.display.flip()

pygame.quit()