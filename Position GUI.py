import pygame
import rospy
from time import sleep
import math
import sys
from std_msgs.msg import String

WindowWidth = 800 # width of the program's window, in pixels
WindowHeight = 600 # height in pixels
CenterX = int(WindowWidth / 2)
CenterY = int(WindowHeight / 2)

FPS = 60

yPos = 0

PERIOD_INCREMENTS = 500.0
AMPLITUDE = 100

pygame.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)


white = (255,255,255)
black = (0,0,0)

red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

Display = pygame.display.set_mode((WindowWidth,WindowHeight))
Display.fill(black)

#pixAr = pygame.PixelArray(Display)
#pixAr[10][20] = green


pygame.draw.polygon(Display, green, ((400,600),(0,0),(800,0)))

step = 0

textsurface = myfont.render("no person detected", False, (0, 0, 255))

def name_callback(name_data):
    global yPos
    global textsurface
    name_data = name_data.data
    nameAndPos = name_data.split("=")
    name, position = nameAndPos[0], nameAndPos[1]
    textsurface = myfont.render(name, False, (0, 0, 255))
    pos_tuple = tuple(map(int, position.split(',')))
    yPos = (pos_tuple[0]/float(1280))*800
    print (yPos)
    
    #1280, 720
    


def startNode():
    rospy.init_node('read_names', anonymous=False)  
    rospy.Subscriber('/names', String, name_callback) 


startNode()

while True:
    
    # Allows the x button on window to quit the script
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    Display.fill(black)
    pygame.draw.polygon(Display, green, ((400,600),(0,0),(800,0)))
    #yPos = 4*(100+(-1 * math.sin(step) * AMPLITUDE))
    if yPos > 775:
        yPos = 775
    elif yPos < 25:
        yPos = 25


    pygame.draw.polygon(Display, red, ((400,600),(yPos-25,0),(yPos+25,0)))

    Display.blit(textsurface,(yPos,200))

    pygame.display.update()

    sleep(0.01)
    step += 0.01
    step %= 2 * math.pi