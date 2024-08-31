import sys
import pygame
import heapq

#initialize Pygame
pygame.init()

#Define constants
Tile_size=50 #Tile Size(as per user needs)
grid_size=10 #Grid Size(as per user needs)
instruction_height=80 #height of instruction box(as per user needs)
screen_width=Tile_size*grid_size
screen_height=Tile_size*grid_size+instruction_height
WHITE=(255,255,255)
BLACK=(0,0,0)
GREEN=(0,128,0)
YELLOW=(255,255,0)
GREY=(200,200,200)
RED=(255,0,0)

#Screen Display
screen=pygame.display.set_mode((screen_width,screen_height))
pygame.display.set_caption("Path Planning and Obstacle Avoidance")

#Bg colour for the environment 
grid=[[WHITE for _ in range(grid_size)] for _ in range(grid_size)]

#Draw grid lines
def draw_grid():
    for x in range(grid_size):
        for y in range(grid_size):
            rect = pygame.Rect(x*Tile_size,y*Tile_size,Tile_size,Tile_size)
            pygame.draw.rect(screen,grid[x][y],rect)
            pygame.draw.rect(screen,BLACK,rect,1) 

#Function to wrap text in instruction box
def wrap_text(text,font,max_width):
    words =text.split(' ')
    lines =[]
    current_line=''
    for word in words:
        test_line=current_line+ word + ' '
        if font.size(test_line)[0] <= max_width:
            current_line= test_line
        else:
            lines.append(current_line)
            current_line= word + ' '
    lines.append(current_line)
    return lines

#Drawing instruction area
def draw_instruction_area(instructions):
    instruction_area_rect=pygame.Rect(0,screen_height-instruction_height, screen_width,instruction_height)
    pygame.draw.rect(screen,GREY,instruction_area_rect)
    
    font = pygame.font.Font(None,36)
    #Wrap text within the instruction box width
    wrapped_text=wrap_text(instructions,font,screen_width-20)  
    for i,line in enumerate(wrapped_text):
        text = font.render(line,True,BLACK)
        screen.blit(text,(10,screen_height-instruction_height+ 10+i*20))  

#Dijikstra Algorithm
def dijkstra(start, goal):
    directions=[(0,1),(1,0),(0,-1),(-1,0)]
    #priorityQueue
    pq=[(0,start)]
    distances={start:0}
    previous={start:None}

    while pq:
        current_distance,current_node=heapq.heappop(pq)

        if current_node==goal:
            path=[]
            while current_node:
                path.append(current_node)
                current_node=previous[current_node]
            path.reverse()
            return path

        for direction in directions:
            neighbor=(current_node[0]+direction[0],current_node[1]+direction[1])
            if 0 <=neighbor[0]<grid_size and 0<= neighbor[1] < grid_size and grid[neighbor[0]][neighbor[1]] != BLACK:
                distance = current_distance + 1
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
    return None

#A* algorithm
def heuristic(a,b):
    return abs(a[0]- b[0])+abs(a[1] - b[1])

def a_star(start,goal):
    directions=[(0,1),(1,0),(0,-1),(-1,0)]
    pq=[(0,start)]
    g_scores={start: 0}
    f_scores={start:heuristic(start, goal)}
    previous={start: None}
    
    while pq:
        current_f_score,current_node=heapq.heappop(pq)
        if current_node == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = previous[current_node]
            path.reverse()
            return path
        
        for direction in directions:
            neighbor=(current_node[0]+direction[0],current_node[1]+direction[1])
            if 0<=neighbor[0]<grid_size and 0<=neighbor[1]<grid_size and grid[neighbor[0]][neighbor[1]]!= BLACK:
                g_score = g_scores[current_node]+1
                f_score = g_score+ heuristic(neighbor, goal)
                if neighbor not in g_scores or g_score < g_scores[neighbor]:
                    g_scores[neighbor]= g_score
                    f_scores[neighbor]= f_score
                    previous[neighbor]= current_node
                    heapq.heappush(pq, (f_score, neighbor))
    return None

#Main LOOP
running=True

#State Variables
instructions="Welcome to Omni World,A Real time situation anaylsis software.Lets begin by setting the obstacles\n"
set_obstacles=True
set_goal=False
set_source=False
goal=False
source=False
path=None

while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT: #if user clicks to cross button to quit
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN: #detects mouse click
            mouse_x,mouse_y=pygame.mouse.get_pos()
            grid_x=mouse_x //Tile_size
            grid_y=mouse_y //Tile_size
            
            #to ignore clicks on instruction box
            if grid_y >= grid_size:  
                continue

            if set_obstacles:
                grid[grid_x][grid_y]=BLACK
            elif set_goal:
                if not goal:
                    if grid[grid_x][grid_y]!=BLACK:
                        grid[grid_x][grid_y]=GREEN
                        goal=(grid_x,grid_y)
                        instructions = "Goal is set.Click '0' to start setting the source point."
                    else:
                        instructions="\nYou Cannot Set the GOAL node on the OBSTACLE!!!! "
                else:
                    instructions="Goal is already set:( "
            elif set_source:
                if not source:
                    if grid[grid_x][grid_y] != BLACK and grid[grid_x][grid_y] != GREEN:
                        grid[grid_x][grid_y] = YELLOW
                        source=(grid_x, grid_y)
                        instructions="Source is set.Press 'D' for Dijkstra or 'A' for A* to find the path."
                    else:
                        instructions="\nYou Cannot Set the SOURCE node on the OBSTACLE Or GOAL node!!!! "
                else:
                    instructions="Source is already set."

        
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_0:
                if set_obstacles:
                    set_obstacles=False
                    set_goal=True
                    instructions="Now set the goal point by clicking on a tile."
                elif set_goal and goal:
                    set_goal=False
                    set_source=True
                    instructions="Now set the source point by clicking on a tile."
            elif event.key==pygame.K_d and source and goal:
                path=dijkstra(source,goal)
                if path:
                    instructions="Path found using Dijkstra.Press 'M' to move the robot."
                else:
                    instructions="No path found using Dijkstra."
            elif event.key == pygame.K_a and source and goal:
                path = a_star(source,goal)
                if path:
                    instructions="Path found using A*. Press 'M' to move the robot."
                else:
                    instructions="No path found using A*."
            elif event.key==pygame.K_m and path:
                instructions="Robot is moving along the path."
                for step in path:
                    grid[step[0]][step[1]]=RED
                    draw_grid()
                    draw_instruction_area(instructions)
                    pygame.display.flip()
                    pygame.time.wait(500)  # Wait for 0.5 second to simulate movement
                instructions = "Robot has reached the goal."
                path = None  # Reset path after movement
                        
    draw_grid()
    draw_instruction_area(instructions)

    pygame.display.flip()

'''
    Developed by:-Manas Nair
    Contact:- nairwork1209@gmail.com
    SASTRA Deemed University
'''