#================================================================================================================================
# Github link of the code repository:
# https://github.com/NehaMadhekar09/ENPM661_PlanningForAutonomousRobots/commits?author=NehaMadhekar09
#================================================================================================================================
import pygame
import numpy as np
import heapq as hq
from collections import deque
from pygame import gfxdraw
import time

pygame.init()
canvas=pygame.display.set_mode((600,250))
 
# For Rectangular Obstacles
pygame.draw.rect(canvas, (255,255,255), pygame.Rect(100, 0, 50, 100).inflate(10,10))
pygame.draw.rect(canvas, (80,208,255), pygame.Rect(100, 0, 50, 100))
pygame.draw.rect(canvas, (255,0,0), pygame.Rect(100, 0, 50, 100),2)

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
x_off=10/np.sqrt(3)
y_off=10+(20/np.sqrt(3))

pygame.draw.polygon(canvas, (255,255,255), ((460-5,25-y_off),(460-5,225+y_off),(510+x_off,125)))
pygame.draw.polygon(canvas, (80,208,255), ((460,25),(460,225),(510,125)))
pygame.draw.polygon(canvas, (255,0,0), ((460,25),(460,225),(510,125)),2)

pygame.display.flip()
#================================================================================================================================
# Functions used for Dijkstras Algorithm

# Checks whether the left move is posiible  
def IsLeftMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==0):
        new_node_xy=(current_node[0][0]-1,current_node[0][1])
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return [True,new_node_xy] 
    return[False]

# Checks whether the right move is posiible  
def IsRightMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==599):
        new_node_xy=(current_node[0][0]+1,current_node[0][1])
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

# Checks whether the up move is posiible  
def IsUpMovePossible(current_node,closed_queue):
    if not(current_node[0][1]==0):
        new_node_xy=(current_node[0][0],current_node[0][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

# Checks whether the down move is posiible  
def IsDownMovePossible(current_node,closed_queue):
    if not(current_node[0][1]==249):
        new_node_xy=(current_node[0][0],current_node[0][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]
    
# Checks whether the up-right move is posiible  
def IsUpRightMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==599 or current_node[0][1]==0):
        new_node_xy=(current_node[0][0]+1,current_node[0][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

# Checks whether the up-left move is posiible
def IsUpLeftMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==0 or current_node[0][1]==0):
        new_node_xy=(current_node[0][0]-1,current_node[0][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

# Checks whether the down-left move is posiible
def IsDownLeftMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==0 or current_node[0][1]==249):
        new_node_xy=(current_node[0][0]-1,current_node[0][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

# Checks whether the down-right move is posiible
def IsDownRightMovePossible(current_node,closed_queue):
    if not(current_node[0][0]==599 or current_node[0][1]==249):
        new_node_xy=(current_node[0][0]+1,current_node[0][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not new_node_xy in closed_queue:
                return[True,new_node_xy]
    return[False]

def UpdateOpenList(current_node,new_node_xy,step_cost,open_queue,index):
    cost_to_come=current_node[1][0] + step_cost
    if new_node_xy in open_queue:
        if(cost_to_come < open_queue[new_node_xy][0]):
            id=open_queue[new_node_xy][1]
            open_queue[new_node_xy]=(cost_to_come, id, current_node[1][1],new_node_xy)
    else:
        open_queue[new_node_xy]=(cost_to_come,index,current_node[1][1])
        new_node=(cost_to_come,index,current_node[1][1],new_node_xy)
    new_node=open_queue[new_node_xy]
    return new_node

# This function generates path by backtracking.
def BacktrackPath(closed_queue,goal_xy,goal_parent_index):
    path=deque()
    path.appendleft(goal_xy)
    parent_index=goal_parent_index
    index=1
    while(index!=0):
        for key,value in closed_queue.items():
            if parent_index==value[1]:
                path.appendleft(key)
                parent_index=value[2]
                index=value[1]
    return path

# Dijkstras Algorithm
def DijkstrasAlgorithm(start_node_xy, goal_node_xy):
    open_Q = {}
    closed_Q={}
    visited_nodes=[]
    
    # creating tuple with cost to come, index, parent node index and adding to the dictionary with key value as node coordinates
    open_Q[start_node_xy]=(0,0,0)
    visited_nodes.append(start_node_xy)

    index=1
    # Run for maximum 1 million iterations
    while index < 1000000:
        open_Q=dict(sorted(open_Q.items(),key=lambda x:x[1][0],reverse = True))
        current_node=open_Q.popitem()
        closed_Q[current_node[0]]=current_node[1]
        
        right_move=IsRightMovePossible(current_node,closed_Q)
        if(right_move[0]):
            new_node_xy=right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        up_right_move=IsUpRightMovePossible(current_node,closed_Q)
        if(up_right_move[0]):
            new_node_xy=up_right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        up_left_move=IsUpLeftMovePossible(current_node,closed_Q)
        if(up_left_move[0]):
            new_node_xy=up_left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break
        
        up_move=IsUpMovePossible(current_node,closed_Q)
        if(up_move[0]):
            new_node_xy=up_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        down_left_move=IsDownLeftMovePossible(current_node,closed_Q)
        if(down_left_move[0]):
            new_node_xy=down_left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        down_right_move=IsDownRightMovePossible(current_node,closed_Q)
        if(down_right_move[0]):
            new_node_xy=down_right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        left_move=IsLeftMovePossible(current_node,closed_Q)
        if(left_move[0]):
            new_node_xy=left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break
        
        down_move=IsDownMovePossible(current_node,closed_Q)
        if(down_move[0]):
            new_node_xy=down_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            visited_nodes.append(new_node_xy)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        if(len(open_Q)==0):
            print("Failed to find solution")
            return

    # Backtrach path
    path=BacktrackPath(closed_Q,goal_node_xy,goal_parent_index)

    return path, visited_nodes


#================================================================================================================================

# Get correct inputs from user
while True:
    start_x=int(input("Enter x coordinate of the start node: "))
    start_y=int(input("Enter y coordinate of the start node: "))
    if (0<=start_x<600 and 0<=start_y<250):
        # Check whether the inputs are not in collision space
        if (canvas.get_at((start_x, start_y))[:3]==(0,0,0)):
            break
        print("The start node lie in the obstacle space. Enter the values again")
    else:
        print("The start node is not within the canvas range. Enter the values again")


while True:
    goal_x=int(input("Enter x coordinate of the goal node: "))
    goal_y=int(input("Enter y coordinate of the goal node: "))
    if (0<=goal_x<600 and 0<=goal_y<250):
        # Check whether the inputs are not in collision space
        if (canvas.get_at((goal_x, goal_y))[:3]==(0,0,0)):
            break
        print("The goal node lie in the obstacle space. Enter the values again")
    else:
        print("The goal node is not within the canvas range. Enter the values again")

    
print("Dijkstras in progress...")

# Convert start and goal node to correct coordinate system
start_node_xy=(start_x,249-start_y)
goal_node_xy=(goal_x,249-goal_y)

start = time.time()

# Run algorithm
path,visited_nodes=DijkstrasAlgorithm(start_node_xy,goal_node_xy)

end = time.time()
print("Done. Total time taken in seconds: ",end - start)

# Display the animation if user says yes
Visualization = input('Would you like to start visualization? Type "y" if yes, type "n" if no: ').lower()
if Visualization.startswith('y'):
    # Show explored nodes
    for i in range (len(visited_nodes)):
        gfxdraw.pixel(canvas, visited_nodes[i][0], visited_nodes[i][1], (0,0,255))
        pygame.display.flip()

    # Show optimal path
    for i in range (len(path)):
        pygame.draw.circle(canvas, (255,255,0), path[i], 3)
        pygame.display.flip()


running = True
  
while running:
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False