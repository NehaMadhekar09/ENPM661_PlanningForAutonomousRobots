import pygame
import numpy as np
 

pygame.init()
canvas=pygame.display.set_mode((600,250))
 

# For Rectangular Obstacles
# pygame.draw.rect(canvas, (255,255,255), pygame.Rect(95, 0, 60, 105))
pygame.draw.rect(canvas, (255,255,255), pygame.Rect(100, 0, 50, 100).inflate(10,10))
pygame.draw.rect(canvas, (80,208,255), pygame.Rect(100, 0, 50, 100))
pygame.draw.rect(canvas, (255,0,0), pygame.Rect(100, 0, 50, 100),2)

# pygame.draw.rect(canvas, (255,255,255), pygame.Rect(95, 145, 60, 105))
pygame.draw.rect(canvas, (255,255,255), pygame.Rect(100, 150, 50, 100).inflate(10,10))
pygame.draw.rect(canvas, (80,208,255), pygame.Rect(100, 150, 50, 100))
pygame.draw.rect(canvas, (255,0,0), pygame.Rect(100, 150, 50, 100),2)

# For Hexagonal Obstacle
centre_hex=(300,125)
side=75
hex_points=[]
hex_points_inflated=[]

line_offset=5
offset_for_vertex=line_offset/np.cos(np.deg2rad(30))
inflated_side= side + offset_for_vertex
print(inflated_side)

for i in range(6):
    x=centre_hex[0]+side*np.cos(np.deg2rad(60*i + 30))
    y=centre_hex[1]+side*np.sin(np.deg2rad(60*i + 30))
    hex_points.append((x,y))
    x_i=centre_hex[0]+inflated_side*np.cos(np.deg2rad(60*i + 30))
    y_i=centre_hex[1]+inflated_side*np.sin(np.deg2rad(60*i + 30))
    hex_points_inflated.append((x_i,y_i))

pygame.draw.polygon(canvas, (255,255,255), (hex_points_inflated[0],hex_points_inflated[1],hex_points_inflated[2],hex_points_inflated[3],hex_points_inflated[4],hex_points_inflated[5]))
pygame.draw.polygon(canvas, (80,208,255), (hex_points[0],hex_points[1],hex_points[2],hex_points[3],hex_points[4],hex_points[5]))
pygame.draw.polygon(canvas, (255,0,0), (hex_points[0],hex_points[1],hex_points[2],hex_points[3],hex_points[4],hex_points[5]),2)


# For Triangular Obstacle
y_offset=5*np.sqrt(3)
x_offset=5/np.sin(np.deg2rad(60))
pygame.draw.polygon(canvas, (255,255,255), ((455,15-y_offset),(455,235+y_offset),(515,125)))
pygame.draw.polygon(canvas, (80,208,255), ((460,25),(460,225),(510,125)))
pygame.draw.polygon(canvas, (80,208,255), ((460,25),(460,225),(510,125)),2)

pygame.display.flip()

running = True
  
# game loop
while running:
    
# for loop through the event queue  
    for event in pygame.event.get():
      
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False