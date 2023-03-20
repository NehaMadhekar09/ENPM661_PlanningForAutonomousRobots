#================================================================================================================================
# Github link of the code repository:
# https://github.com/NehaMadhekar09/ENPM661_PlanningForAutonomousRobots/commits?author=NehaMadhekar09
#================================================================================================================================
import pygame
import numpy as np
from collections import deque
from shapely.geometry import LineString
import time


def FindIntersectionOfTwoLines(line1,line2):
    if(line1[1][0] - line1[0][0]==0):
        m2 = (line2[1][1]- line2[0][1]) / (line2[1][0]- line2[0][0])
        c2 = line2[0][1] - m2 * line2[0][0]
        intersection_x=line1[0][0]
        intersection_y=m2* intersection_x + c2
    elif(line2[1][0] - line2[0][0]==0):
        m1 = (line1[1][1]- line1[0][1]) / (line1[1][0]- line1[0][0])
        c1 = line1[0][1] - m1 * line1[0][0]
        intersection_x=line2[0][0]
        intersection_y = m1 * intersection_x + c1
    else:
        m1 = (line1[1][1]- line1[0][1]) / (line1[1][0]- line1[0][0])
        m2 = (line2[1][1]- line2[0][1]) / (line2[1][0]- line2[0][0])
        c1 = line1[0][1] - m1 * line1[0][0]
        c2 = line2[0][1] - m2 * line2[0][0]
        intersection_x = (c2 - c1) / (m1 - m2)
        intersection_y = m1 * intersection_x + c1

    return (intersection_x,intersection_y)


def CreateObstacles(canvas,clerarance,robot_radius):
    distance= clerarance+robot_radius

    # For Border clerance
    canvas.fill((255,255,255))
    pygame.draw.rect(canvas, (0,0,0), pygame.Rect(distance, distance, 600-2*distance, 250-2*distance))


    # For Rectangular Obstacles
    pygame.draw.rect(canvas, (255,255,255), pygame.Rect(100, 0, 50, 100).inflate(2*distance,2*distance))
    pygame.draw.rect(canvas, (80,208,255), pygame.Rect(100, 0, 50, 100))
    pygame.draw.rect(canvas, (255,0,0), pygame.Rect(100, 0, 50, 100),2)

    pygame.draw.rect(canvas, (255,255,255), pygame.Rect(100, 150, 50, 100).inflate(2*distance,2*distance))
    pygame.draw.rect(canvas, (80,208,255), pygame.Rect(100, 150, 50, 100))
    pygame.draw.rect(canvas, (255,0,0), pygame.Rect(100, 150, 50, 100),2)

    # For Hexagonal Obstacle
    centre_hex=(300,125)
    side=75
    hex_points=[]
    hex_points_inflated=[]

    offset_for_vertex=distance/np.cos(np.deg2rad(30))
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

    line1 = LineString([(460, 25), (460, 225)])
    line2 = LineString([(460, 225), (510, 125)])
    line3 = LineString([(510, 125), (460, 25)])

    line1_offset = line1.parallel_offset(-distance)
    line2_offset = line2.parallel_offset(-distance)
    line3_offset = line3.parallel_offset(-distance)

    intersection1=FindIntersectionOfTwoLines(list(line1_offset.coords),list(line2_offset.coords))
    intersection2=FindIntersectionOfTwoLines(list(line2_offset.coords),list(line3_offset.coords))
    intersection3=FindIntersectionOfTwoLines(list(line3_offset.coords),list(line1_offset.coords))

    pygame.draw.polygon(canvas, (255,255,255), ((intersection1[0],intersection1[1]),(intersection2[0],intersection2[1]),(intersection3[0],intersection3[1])))
    pygame.draw.polygon(canvas, (80,208,255), ((460,25),(460,225),(510,125)))
    pygame.draw.polygon(canvas, (255,0,0), ((460,25),(460,225),(510,125)),2)

    pygame.display.flip()

#================================================================================================================================
# Functions used for A* Algorithm

def IsAngleActionPossible(current_node,closed_queue,closed_Q_matrix,L,angle):
    new_angle=current_node[1][5][2] + angle
    if new_angle < 0:
        new_angle=360+new_angle
    if new_angle >= 360:
        new_angle=new_angle-360
    new_node_xytheta=(current_node[1][5][0]+L*np.cos(np.deg2rad(new_angle)),current_node[1][5][1]+L*np.sin(np.deg2rad(new_angle)),new_angle)
    x=new_node_xytheta[0]
    y=new_node_xytheta[1]
    if 0<=x<600 and 0<=y<250:
        if canvas.get_at((int(new_node_xytheta[0]), int(new_node_xytheta[1])))[:3]==(0,0,0):
            new_node_xythetaD=DiscretizeNodeCoordinates(new_node_xytheta)
            if closed_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]==0:
                return [True,new_node_xythetaD,new_node_xytheta] 
    return[False]

def ComputeDistance(node_xytheta,goal_node_xytheta):
    return np.sqrt((goal_node_xytheta[0]-node_xytheta[0])*(goal_node_xytheta[0]-node_xytheta[0])+(goal_node_xytheta[1]-node_xytheta[1])*(goal_node_xytheta[1]-node_xytheta[1]))
    

def UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,action_cost,open_queue,open_matrix,index,goal_node_xytheta):
    cost_to_come=current_node[1][1] + action_cost
    if open_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]==1:
        if(cost_to_come < open_queue[new_node_xythetaD][1]):
            id=open_queue[new_node_xythetaD][3]
            cost_to_go=open_queue[new_node_xythetaD][2]
            total_cost=cost_to_come+cost_to_go
            open_queue[new_node_xythetaD]=(total_cost,cost_to_come,cost_to_go, id, current_node[1][3],new_node_xytheta)
    else:
        cost_to_go=ComputeDistance(new_node_xytheta,goal_node_xytheta)
        total_cost=cost_to_come+cost_to_go
        open_queue[new_node_xythetaD]=(total_cost,cost_to_come,cost_to_go,index,current_node[1][3],new_node_xytheta)
    new_node=open_queue[new_node_xythetaD]
    return new_node

# This function generates path by backtracking.
def BacktrackPath(closed_queue,goal_xy_theta,goal_parent_index):
    path=deque()
    path.appendleft(goal_xy_theta)
    parent_index=goal_parent_index
    index=1
    while(index!=0):
        for key,value in closed_queue.items():
            if parent_index==value[3]:
                path.appendleft(value[5])
                parent_index=value[4]
                index=value[3]
    return path

def DiscretizeNodeCoordinates(actual_coordinates,thresholds=(0.5,0.5,30)):
    discretized_coordinates=[]
    for i in range(len(actual_coordinates)):
        remainder=actual_coordinates[i]%thresholds[i]
        answer=int(actual_coordinates[i]/thresholds[i])
        if(remainder <= thresholds[i]/2):
            discretized_coordinates.append(answer*thresholds[i])
        else:
            discretized_coordinates.append((answer+1)*thresholds[i])
    return (discretized_coordinates[0],discretized_coordinates[1],discretized_coordinates[2])


def IsGoalReached(node_xytheta, goal_node_xytheta,goal_threshold):
    goal_reached=False
    distance=ComputeDistance(node_xytheta,goal_node_xytheta)
    if distance<goal_threshold and abs(goal_node_xytheta[2]-node_xytheta[2]) <= 30:
        goal_reached=True
    return goal_reached

# A* Algorithm
def A_Star_Algorithm(start_node_xytheta, goal_node_xytheta):
    visited_nodes=[]
    open_Q = {}
    open_Q_matrix=np.zeros(shape=(500,1200,12))
    closed_Q={}
    closed_Q_matrix=np.zeros(shape=(500,1200,12))
    
    
    # creating tuple with total cost,cost to come,cost to go,node index, parent node index and tuple with actual position(x,y,theta)
    # Adding it to the dictionary with key value as descritized node coordinates(x_d,y_d,theta_d)
    start_node_xythetaD=DiscretizeNodeCoordinates(start_node_xytheta)
    open_Q[start_node_xythetaD]=(0,0,0,0,0,start_node_xytheta)
    open_Q_matrix[int(start_node_xythetaD[1]/0.5)][int(start_node_xythetaD[0]/0.5)][int(start_node_xythetaD[2]/30)]=1
    visited_nodes.append((0,start_node_xytheta))

    L=10
    goal_threshold=1.5
    index=1
    # Run for maximum 10000 iterations
    isConverged=True
    while True:
        if index > 10000:
            print("Failed to reach the goal.Kindly check whether the goal is in reachable area. Max Iterations Reached.")
            isConverged=False
            break
        open_Q=dict(sorted(open_Q.items(),key=lambda x:x[1][0],reverse = True))
        current_node=open_Q.popitem()
        # open_Q_matrix[int(current_node[0][1]/0.5)][int(current_node[0][0]/0.5)][int(current_node[0][2]/30)]=0
        closed_Q[current_node[0]]=current_node[1]
        closed_Q_matrix[int(current_node[0][1]/0.5)][int(current_node[0][0]/0.5)][int(current_node[0][2]/30)]=1
        
        Action_0=IsAngleActionPossible(current_node,closed_Q,closed_Q_matrix,L,0)
        if(Action_0[0]):
            new_node_xythetaD=Action_0[1]
            new_node_xytheta=Action_0[2]
            new_node=UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,1,open_Q,open_Q_matrix,index,goal_node_xytheta)
            open_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta))
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xytheta,goal_threshold):
                goal_parent_index=current_node[1][3]
                break

        Action_30=IsAngleActionPossible(current_node,closed_Q,closed_Q_matrix,L,30)
        if(Action_30[0]):
            new_node_xythetaD=Action_30[1]
            new_node_xytheta=Action_30[2]
            new_node=UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,1,open_Q,open_Q_matrix,index,goal_node_xytheta)
            open_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta))
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xytheta,goal_threshold):
                goal_parent_index=current_node[1][3]
                break

        Action_Neg30=IsAngleActionPossible(current_node,closed_Q,closed_Q_matrix,L,-30)
        if(Action_Neg30[0]):
            new_node_xythetaD=Action_Neg30[1]
            new_node_xytheta=Action_Neg30[2]
            new_node=UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,1,open_Q,open_Q_matrix,index,goal_node_xytheta)
            open_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta))
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xytheta,goal_threshold):
                goal_parent_index=current_node[1][3]
                break

        Action_60=IsAngleActionPossible(current_node,closed_Q,closed_Q_matrix,L,60)
        if(Action_60[0]):
            new_node_xythetaD=Action_60[1]
            new_node_xytheta=Action_60[2]
            new_node=UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,1,open_Q,open_Q_matrix,index,goal_node_xytheta)
            open_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta))
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xytheta,goal_threshold):
                goal_parent_index=current_node[1][3]
                break

        Action_Neg60=IsAngleActionPossible(current_node,closed_Q,closed_Q_matrix,L,-60)
        if(Action_Neg60[0]):
            new_node_xythetaD=Action_Neg60[1]
            new_node_xytheta=Action_Neg60[2]
            new_node=UpdateOpenList(current_node,new_node_xythetaD,new_node_xytheta,1,open_Q,open_Q_matrix,index,goal_node_xytheta)
            open_Q_matrix[int(new_node_xythetaD[1]/0.5)][int(new_node_xythetaD[0]/0.5)][int(new_node_xythetaD[2]/30)]=1
            visited_nodes.append((current_node[1][3],new_node_xytheta))
            index+=1
            if IsGoalReached(new_node_xytheta,goal_node_xytheta,goal_threshold):
                goal_parent_index=current_node[1][3]
                break
            
        if(len(open_Q)==0):
            print("Failed to find solution")
            isConverged=False
            break

    if(isConverged):
        # Backtrach path
        path=BacktrackPath(closed_Q,goal_node_xytheta,goal_parent_index)
        return [isConverged,path, visited_nodes]

    return [isConverged]


pygame.init()
canvas=pygame.display.set_mode((600,250))

CreateObstacles(canvas,5,5)

start=(11,11,0)
goal=(500,50,30)
L=10


start_time = time.time()
result=A_Star_Algorithm(start,goal)

end_time = time.time()
print("Total time taken in seconds: ",end_time - start_time)

if result[0]:
    path=result[1]
    visited_nodes=result[2]
    for i in range(1,len(visited_nodes)):
        parent_index=visited_nodes[i][0]
        pygame.draw.line(canvas, (0, 0, 255),
                [visited_nodes[parent_index][1][0], visited_nodes[parent_index][1][1]],
                [visited_nodes[i][1][0], visited_nodes[i][1][1]], 1)
        pygame.display.flip()
    
    # Show optimal path
    for i in range (len(path)-1):
        pygame.draw.circle(canvas, (255,255,0), (path[i][0],path[i][1]),1)
        pygame.draw.line(canvas, (255,255,0),
                [path[i][0], path[i][1]],
                [path[i+1][0], path[i+1][1]], 1)
        pygame.display.flip()

pygame.draw.circle(canvas, (255,0,0), (start[0],start[1]),5,3)
pygame.draw.circle(canvas, (255,0,0), (goal[0],goal[1]),5,3)
pygame.display.flip()


running = True
  
while running:
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False


#================================================================================================================================

# # Get correct inputs from user
# while True:
#     start_x=int(input("Enter x coordinate of the start node: "))
#     start_y=int(input("Enter y coordinate of the start node: "))
#     if (0<=start_x<600 and 0<=start_y<250):
#         # Check whether the inputs are not in collision space
#         if (canvas.get_at((start_x, start_y))[:3]==(0,0,0)):
#             break
#         print("The start node lie in the obstacle space. Enter the values again")
#     else:
#         print("The start node is not within the canvas range. Enter the values again")


# while True:
#     goal_x=int(input("Enter x coordinate of the goal node: "))
#     goal_y=int(input("Enter y coordinate of the goal node: "))
#     if (0<=goal_x<600 and 0<=goal_y<250):
#         # Check whether the inputs are not in collision space
#         if (canvas.get_at((goal_x, goal_y))[:3]==(0,0,0)):
#             break
#         print("The goal node lie in the obstacle space. Enter the values again")
#     else:
#         print("The goal node is not within the canvas range. Enter the values again")

    
# print("Dijkstras in progress...")

# # Convert start and goal node to correct coordinate system
# start_node_xy=(start_x,249-start_y)
# goal_node_xy=(goal_x,249-goal_y)

# start = time.time()

# # Run algorithm
# path,visited_nodes=DijkstrasAlgorithm(start_node_xy,goal_node_xy)

# end = time.time()
# print("Done. Total time taken in seconds: ",end - start)

# # Display the animation if user says yes
# Visualization = input('Would you like to start visualization? Type "y" if yes, type "n" if no: ').lower()
# if Visualization.startswith('y'):
#     # Show explored nodes
#     for i in range (len(visited_nodes)):
#         gfxdraw.pixel(canvas, visited_nodes[i][0], visited_nodes[i][1], (0,0,255))
#         pygame.display.flip()

#     # Show optimal path
#     for i in range (len(path)):
#         pygame.draw.circle(canvas, (255,255,0), path[i], 3)
#         pygame.display.flip()


# running = True
  
# while running:
#     for event in pygame.event.get():
#         # Check for QUIT event      
#         if event.type == pygame.QUIT:
#             running = False


#================================================================================================================================