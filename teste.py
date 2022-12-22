import pygame
import time

from matplotlib import pyplot as plt
import matplotlib.backends.backend_tkagg

import os
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (800,0)

pygame.init()

screen_width = 800
screen_height = 600

screen = pygame.display.set_mode((screen_width, screen_height))


def draw_shapes(square1_x, square1_y, square2_x, square2_y, square3_x, square3_y, circle_x, circle_y):
  # Desenhe o primeiro quadrado na posição (square1_x, square1_y)
  pygame.draw.rect(screen, (0, 220, 150), (square1_x, square1_y, 25, 25))
  
  # Desenhe o segundo quadrado na posição (square2_x, square2_y)
  pygame.draw.rect(screen, (0, 255, 0), (square2_x, square2_y, 25, 25))
  
  # Desenhe o terceiro quadrado na posição (square3_x, square3_y)
  pygame.draw.rect(screen, (0, 200, 150), (square3_x, square3_y, 25, 25))
  
  # Desenhe o círculo na posição (circle_x, circle_y)
  pygame.draw.circle(screen, (0, 0, 255), (circle_x, circle_y), 10)

circle_x = 100+10
circle_y = 100+10
circle_xVel = 0
circle_yVel = 0
Kp = 0.025
final = (710,110)

# Crie uma lista para armazenar as teclas pressionadas
keys_down = []
distance = []
# Cria o gráfico
fig, ax = plt.subplots()

mngr = plt.get_current_fig_manager()
geom = mngr.window.geometry()
x,y,dx,dy = geom.getRect()
mngr.window.setGeometry(x,y,640, 545)
# Cria os eixos do gráfico
x, y = [], []

tempo = time.time()

running = True
ax.plot()
while running:
    screen.fill((255, 255, 255))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Verifique se a tecla W foi pressionada
        if event.type == pygame.KEYDOWN and event.key == pygame.K_w:
            keys_down.append(pygame.K_w)
        # Verifique se a tecla A foi pressionada
        if event.type == pygame.KEYDOWN and event.key == pygame.K_a:
            keys_down.append(pygame.K_a)
        # Verifique se a tecla S foi pressionada
        if event.type == pygame.KEYDOWN and event.key == pygame.K_s:
            keys_down.append(pygame.K_s)
        # Verifique se a tecla D foi pressionada
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            keys_down.append(pygame.K_d)
        # Verifique se a tecla W foi solta
        if event.type == pygame.KEYUP and event.key == pygame.K_w:
            keys_down.remove(pygame.K_w)
        # Verifique se a tecla A foi solta
        if event.type == pygame.KEYUP and event.key == pygame.K_a:
            keys_down.remove(pygame.K_a)
        # Verifique se a tecla S foi solta
        if event.type == pygame.KEYUP and event.key == pygame.K_s:
            keys_down.remove(pygame.K_s)
        # Verifique se a tecla D foi solta
        if event.type == pygame.KEYUP and event.key == pygame.K_d:
            keys_down.remove(pygame.K_d)

    # Atualiza os eixos do gráfico
    ax.clear()
    ax.plot(distance)
    # Adiciona um título ao gráfico
    ax.set_title("Controle Proporcional Derivativo")

    # Adiciona um título ao eixo x
    ax.set_xlabel("Tempo")

    # Adiciona um título ao eixo y
    ax.set_ylabel("Distancia para a chegada")

    # Exibe o gráfico
    plt.pause(0.00001)

    if(time.time() - tempo < 10):

        circle_y += 1
        
        distance.append( (((circle_x-final[0])**2) +((circle_y-final[1])**2 ))**(0.5) ) 

    else:

        velocidadeDoMotor = -Kp * (posiçãoAtual-final[0])
        

        if pygame.K_w in keys_down:
            circle_y -= circle_yVel
        if pygame.K_a in keys_down:
            circle_x -= circle_xVel
        if pygame.K_s in keys_down:
            circle_y += circle_yVel
        if pygame.K_d in keys_down:
            circle_x += circle_xVel

        
        circle_x += circle_xVel
        circle_y += circle_yVel
        distance.append( (((circle_x-final[0])**2) +((circle_y-final[1])**2 ))**(0.5) ) 



    draw_shapes(100, 100, 700, 100, 100, 400, circle_x, circle_y)

    pygame.display.flip()
    pygame.time.Clock().tick(30)

pygame.quit()