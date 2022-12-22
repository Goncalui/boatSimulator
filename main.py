import math
import pygame

from getDistance import getDistance

pygame.init()

screen = pygame.display.set_mode([800,1000]) #16x20

running = True

class Boat:
    def __init__(self,rectSize,startPos,Bscreen,Bscolor,atualPos = 0):
        self.rectSize = rectSize
        self.startPos = startPos
        self.atualPos = startPos
        self.screen = Bscreen
        self.color = Bscolor
    def drawBoat(self,color=0):
        if(color):
            pygame.draw.rect(self.screen,color,(self.startPos[0],self.startPos[1],self.rectSize[0],self.rectSize[1]))
        else:
            pygame.draw.rect(self.screen,self.color,(self.startPos[0],self.startPos[1],self.rectSize[0],self.rectSize[1]))
    

barco = Boat((20,20),(400,200),screen,(255,0,0))
rectSize = 50
rectSize2 = rectSize/4
screenWidth = screen.get_width()

tela = 0
atuali = 0

while running:
    atuali+=1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONUP:
            if(math.sqrt((x-(500+rectSize/4))**2 + (y-(50+rectSize/4))**2) <= 5):
                if(tela != 1):
                    tela = 1
                else:
                    tela = 0
            elif(math.sqrt((x-(50+rectSize/4))**2 + (y-(400+rectSize/4))**2) <= 5):
                if(tela != 2):
                    tela = 2
                else:
                    tela = 0

    x = pygame.mouse.get_pos()[0]
    y = pygame.mouse.get_pos()[1]

    screen.fill((255,255,255))

    if(tela == 0):
        pygame.draw.rect(screen,(20,20,125),(int(screenWidth/2) - 300,int(screenWidth/2) - 300,rectSize*11,rectSize*11))
        pygame.draw.rect(screen,(100,100,100),(500,50,rectSize/2,rectSize/2))
        pygame.draw.rect(screen,(100,100,100),(50,400,rectSize/2,rectSize/2))
        pygame.draw.circle(screen,(0,255,0),(50+rectSize/4,400+rectSize/4),20/4)
        pygame.draw.circle(screen,(0,255,0),(500+rectSize/4,50+rectSize/4),20/4)
        up = 2000

        pygame.draw.circle(screen, (0, 255, 0),[50+rectSize/4,400+rectSize/4],atuali%up, 1+((1 + atuali//up)*up- atuali)//100)
        pygame.draw.circle(screen, (0, 255, 0),[500+rectSize/4,50+rectSize/4],atuali%up, 1+((1 + atuali//up)*up- atuali)//100)
        barco.drawBoat()

    elif(tela == 1):
        
        for j in range(20*4):
            for i in range(16*4):
                color = (255 - (math.sqrt(((500+rectSize/4)-(rectSize2*i))**2 + ((50+rectSize/4)-(rectSize2*j))**2+0.001)//20)*4.5,0 + (math.sqrt(((500+rectSize/4)-(rectSize2*i))**2 + ((50+rectSize/4)-(rectSize2*j))**2+0.001)//20)*4.5,0)

                pygame.draw.rect(screen,(0,0,0),(rectSize2*i,rectSize2*j,rectSize2,rectSize2))
                pygame.draw.rect(screen,color,(rectSize2*i,rectSize2*j,rectSize2-1/4,rectSize2-1/4))
            
                pygame.draw.rect(screen,(100,100,100),(500,50,rectSize/2,rectSize/2))
                pygame.draw.circle(screen,(0,255,0),(500+rectSize/4,50+rectSize/4),20/4)
                
                #pygame.display.flip()
        
        barco.drawBoat(color = (0,255,0))
        font1 = pygame.font.SysFont('chalkduster.ttf',20)
        img1 = font1.render(str(getDistance(-44-((math.sqrt(((500+rectSize/4)-(barco.atualPos[0]))**2 + ((50+rectSize/4)-(barco.atualPos[1]))**2+0.001)//20)*4.5)//6)),True,(0,0,255))
        screen.blit(img1, (20, 50))

    elif(tela == 2):
        for i in range(16*4):
            for j in range(20*4):
                color = (255 - (math.sqrt(((50+rectSize/4)-(rectSize2*i))**2 + ((400+rectSize/4)-(rectSize2*j))**2+0.001)//20)*4.5, 0+ (math.sqrt(((50+rectSize/4)-(rectSize2*i))**2 + ((400+rectSize/4)-(rectSize2*j))**2+0.001)//20)*4.5,0)

                pygame.draw.rect(screen,(0,0,0),(rectSize2*i,rectSize2*j,rectSize2,rectSize2))
                pygame.draw.rect(screen,color,(rectSize2*i,rectSize2*j,rectSize2-1/4,rectSize2-1/4))

                pygame.draw.rect(screen,(100,100,100),(50,400,rectSize/2,rectSize/2))
                pygame.draw.circle(screen,(0,255,0),(50+rectSize/4,400+rectSize/4),20/4)
                
                #pygame.display.flip()
        
        barco.drawBoat(color = (0,255,0))
        font1 = pygame.font.SysFont('chalkduster.ttf',20)
        img1 = font1.render(str(getDistance(-44-((math.sqrt(((50+rectSize/4)-(barco.atualPos[0]))**2 + ((400+rectSize/4)-(barco.atualPos[1]))**2+0.001)//20)*4.5)//6)),True,(0,0,255))
        screen.blit(img1, (20, 50))
    
    pygame.display.flip()

pygame.quit()
