import pygame
import numpy as np
import heapq as hq
from collections import deque

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
# print(inflated_side)

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
# //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

# Checks whether the left move is posiible  
def IsLeftMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==0):
        new_node_xy=(current_node[3][0]-1,current_node[3][1])
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return [True,new_node_xy] 
    return[False]

# Checks whether the right move is posiible  
def IsRightMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==600):
        new_node_xy=(current_node[3][0]+1,current_node[3][1])
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

# Checks whether the up move is posiible  
def IsUpMovePossible(current_node,closed_queue):
    if not(current_node[3][1]==0):
        new_node_xy=(current_node[3][0],current_node[3][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

# Checks whether the down move is posiible  
def IsDownMovePossible(current_node,closed_queue):
    if not(current_node[3][1]==250):
        new_node_xy=(current_node[3][0],current_node[3][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]
    
# Checks whether the up-right move is posiible  
def IsUpRightMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==600 or current_node[3][1]==0):
        new_node_xy=(current_node[3][0]+1,current_node[3][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

# Checks whether the up-left move is posiible
def IsUpLeftMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==0 or current_node[3][1]==0):
        new_node_xy=(current_node[3][0]-1,current_node[3][1]-1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

# Checks whether the down-left move is posiible
def IsDownLeftMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==0 or current_node[3][1]==250):
        new_node_xy=(current_node[3][0]-1,current_node[3][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

# Checks whether the down-right move is posiible
def IsDownRightMovePossible(current_node,closed_queue):
    if not(current_node[3][0]==600 or current_node[3][1]==250):
        new_node_xy=(current_node[3][0]+1,current_node[3][1]+1)
        if canvas.get_at((new_node_xy[0], new_node_xy[1]))[:3]==(0,0,0):
            if not(new_node_xy in list(zip(*closed_queue))[3]):
                return[True,new_node_xy]
    return[False]

def UpdateOpenList(current_node,new_node_xy,step_cost,open_queue,index):
    cost_to_come=current_node[0] + step_cost
    is_present=False
    for i in range(len(open_queue)):
        if open_queue[i][3]==new_node_xy:
            is_present=True
            if(cost_to_come < open_queue[i][0]):
                id=open_queue[i][1]
                open_queue[i]=(cost_to_come, id, current_node[1],new_node_xy)
            new_node=open_queue[i]    
    if not is_present:
        new_node=(cost_to_come,index,current_node[1],new_node_xy)
        hq.heappush(open_queue,new_node)
    hq.heapify(open_queue)
    return new_node

# This function generates path by backtracking.
def BacktrackPath(closed_queue,goal_xy,goal_parent_index):
    path=deque()
    path.appendleft(goal_xy)
    parent_index=goal_parent_index
    index=1
    while(index!=0):
        for i in range(len(closed_queue)):
            if(closed_queue[i][1]==parent_index):
                path.appendleft(closed_queue[i][3])
                parent_index=closed_queue[i][2]
                index=closed_queue[i][1]
    return path

# Dijkstras Algorithm

def DijkstrasAlgorithm(start_node_xy, goal_node_xy):
    open_Q = []
    closed_Q=[]
    
    # creating tuple with cost to come, index, parent node index and coordinate values (x,y)
    node=(0, 0, 0, start_node_xy)  
    hq.heappush(open_Q, node)
    hq.heapify(open_Q)
    hq.heapify(closed_Q)

    index=1
    # Run for maximum 1 million iterations
    while index < 1000000:
        current_node=hq.heappop(open_Q)
        hq.heappush(closed_Q,current_node)
        hq.heapify(closed_Q)
        hq.heapify(open_Q)

        right_move=IsRightMovePossible(current_node,closed_Q)
        if(right_move[0]):
            new_node_xy=right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        up_right_move=IsUpRightMovePossible(current_node,closed_Q)
        if(up_right_move[0]):
            new_node_xy=up_right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        up_left_move=IsUpLeftMovePossible(current_node,closed_Q)
        if(up_left_move[0]):
            new_node_xy=up_left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break
        
        up_move=IsUpMovePossible(current_node,closed_Q)
        if(up_move[0]):
            new_node_xy=up_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        down_left_move=IsDownLeftMovePossible(current_node,closed_Q)
        if(down_left_move[0]):
            new_node_xy=down_left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        down_right_move=IsDownRightMovePossible(current_node,closed_Q)
        if(down_right_move[0]):
            new_node_xy=down_right_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1.4,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        left_move=IsLeftMovePossible(current_node,closed_Q)
        if(left_move[0]):
            new_node_xy=left_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break
        
        down_move=IsDownMovePossible(current_node,closed_Q)
        if(down_move[0]):
            new_node_xy=down_move[1]
            new_node=UpdateOpenList(current_node,new_node_xy,1,open_Q,index)
            index+=1
            if(new_node_xy==goal_node_xy):
                goal_parent_index=new_node[2]
                break

        if(len(open_Q)==0):
            print("Failed to find solution")
            return


    path=BacktrackPath(closed_Q,goal_node_xy,goal_parent_index)

    return path


path=DijkstrasAlgorithm((90,90),(160,90))
print(path)


running = True
  
while running:
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False