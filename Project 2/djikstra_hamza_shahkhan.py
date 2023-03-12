# %%
import cv2
import numpy as np
from math import sin, cos, pi
import heapq as hq
import time

# %%
def regularPoly(n,a,b,r):
    points = [(a,b+r)]
    theta = pi/2
    dTheta = 2*pi/n

    for i in range(1,n):
        theta += dTheta
        points.append((a + r*cos(theta), b + r*sin(theta)))
    points = np.array(points)
    points = np.round(points,0)
    points = points.astype(int)
    points = points.reshape(-1,1,2)

    return points

canvas = np.zeros([250,600,3], dtype = 'uint8')

color = (255,255,255)
bcolor = (100,100,100)

cv2.rectangle(canvas,(95,0),(155,105),bcolor,-1)
cv2.rectangle(canvas,(95,145),(155,250),bcolor,-1)
cv2.rectangle(canvas,(100,0),(150,100),color,-1)
cv2.rectangle(canvas,(100,150),(150,250),color,-1)

tri_pts = np.array([[460,25],[460,225],[510,125]])
tri_pts = tri_pts.reshape((-1,1,2))
trib_pts = np.array([[455,4],[455,246],[515,125]])
trib_pts = trib_pts.reshape((-1,1,2))
cv2.fillPoly(canvas,[trib_pts],(bcolor))
cv2.fillPoly(canvas,[tri_pts],(color))

hex_pts = regularPoly(6,300,125,75)
bloat = regularPoly(6,300,125,81)
cv2.fillPoly(canvas,[bloat],bcolor)
cv2.fillPoly(canvas,[hex_pts],color)

cv2.imshow('SS',canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()

map = canvas.copy()
row, col,_ = map.shape
obstacle_space = []
for j in range(row):
    for i in range(col):
        if map[j,i].all() > 0:
            obstacle_space.append((i,j))
if (25,25) in obstacle_space:
    print('dd')

# %%
#Functions to move key/0 in the 3x3 Matrix
def move_up(CurrentNode):
  i,j = CurrentNode
  Cost = 1
  if i>0 :
    NewNode = (i-1,j)
    return NewNode,Cost

def move_down(CurrentNode):
  
  i,j = CurrentNode
  Cost = 1
  if i<250 :
    NewNode = (i+1,j)
    return NewNode,Cost

def move_left(CurrentNode):
  i,j = CurrentNode
  Cost = 1
  if j>0:
    NewNode =  (i,j-1)
    return NewNode,Cost

def move_right(CurrentNode):
  i,j = CurrentNode
  Cost = 1
  if j<600:
    NewNode = (i,j+1)
    return NewNode,Cost

def move_top_right(CurrentNode):
  i,j = CurrentNode
  Cost = 1.4
  if (i>0 and j<600):
    NewNode  = (i-1,j+1)
    return NewNode,Cost

def move_top_left(CurrentNode):
  i,j = CurrentNode 
  Cost = 1.4
  if (i>0 and j>0):
    NewNode  = (i-1,j-1)
    return NewNode,Cost

def move_bottom_right(CurrentNode):
  i,j = CurrentNode
  Cost = 1.4
  if (i<250 and j<600):
    NewNode  = (i+1,j+1)
    return NewNode,Cost

def move_bottom_left(CurrentNode):
  i,j = CurrentNode
  Cost = 1.4
  if (i<250 and j>0):
    NewNode  = (i+1,j-1)
    return NewNode,Cost



# %%
def generate_path(goal_node,start_node,bk_dict):
  '''
  Returns List of Parent Nodes
  Parameters:
  Input:
  goal_node = int
    posiiton of goal state achieved (last node)
  bk_dict = dictionary
    dictionary containing all the child-parent node indices
  Output:
  p = list
    Returns list of parent nodes
  '''
  q=goal_node
  p= []
  while(q != start_node):
    p.append(q)
    q = bk_dict[q]
  p.append(start_node)
  p.reverse()
  return p


# %%

def animate_path(path,map_):
    map_img = map_.copy()
    color = (0,0,255)
    out = cv2.VideoWriter('path_animation.avi',cv2.VideoWriter_fourcc(*'MJPG'), 60, (map_img.shape[1],map_img.shape[0]))
 
    for node in range(0,len(path)-1):
        f = cv2.line(map_img,path[node],path[node+1],color,1)
        f = cv2.flip(f,0)
        out.write(f)
        cv2.imshow('Final Path',f)
        cv2.waitKey(1)
    out.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows() 
    
    print('Video File created')


def animate_search(visited_nodes,parent_dict,map_):
    map_img = map_.copy()
    out = cv2.VideoWriter('search_animation.avi',cv2.VideoWriter_fourcc(*'MJPG'), 60, (map_img.shape[1],map_img.shape[0]))
    for node in visited_nodes:
        f = cv2.circle(map_img,node,0,(255,0,0),-1)
        f = cv2.flip(f,0)
        out.write(f)
        cv2.imshow('Djikstra Search',f)
        cv2.waitKey(1)
    out.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print('Video file created')

def animate_djikstra(visited_nodes,path,map_):
    color = (255,0,0)
    path_color = (0,0,255)
    map_img = map_.copy()
    #out = cv2.VideoWriter('djikstra_animation.avi',cv2.VideoWriter_fourcc(*'MJPG'), 60, (map_img.shape[1],map_img.shape[0]))
    for node in visited_nodes:
        f = cv2.circle(map_img,node,0,color,-1)
        f = cv2.flip(f,0)
        #out.write(f)
        cv2.imshow('Djikstra Search',f)
        cv2.waitKey(1)
    
    for node in range(0,len(path)-1):
        f = cv2.circle(map_img,path[0],2,(0,255,0),-1)
        f = cv2.circle(map_img,path[-1],2,(0,255,0),-1)
        f = cv2.line(map_img,path[node],path[node+1],path_color,2)
        f = cv2.flip(f,0)
        #out.write(f)
        cv2.imshow('Djikstra Search',f)
        cv2.waitKey(1)

    #out.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print('Video file created')
    


# %%
def djikstra(start_node,goal_node):
    print('\nDjikstra Algorithm started')
    actionset =[move_up,move_down,move_left,move_right,move_top_left,move_top_right,move_bottom_left,move_bottom_right]
    parent = {}
    parent[start_node] = None

    Q = []
    hq.heappush(Q,(0,start_node))
    hq.heapify(Q)

    closed_list = []
    closed_list.append(start_node)

    flag = 0

    C2C = {}
    C2C[start_node] = 0

    while (flag!=1):

        _,currentNode = hq.heappop(Q)
        
        if currentNode == goal_node:
            path = generate_path(goal_node,start_node,parent)
            print('\nGoal Node reached')
            print('\nFinal Cost to Come: ',C2C[goal_node])
            #animate_path(path,canvas)
            flag = 1
            break

        for action in actionset:
            if action(currentNode) is not None:
                new_node, Cost = action(currentNode)
                if new_node not in obstacle_space:
                    if new_node not in closed_list:
                        temp_cost = C2C[currentNode] + Cost
                        if new_node not in C2C or temp_cost < C2C[new_node]:
                            parent[new_node] = currentNode
                            C2C[new_node] = temp_cost
                            closed_list.append(new_node)
                            hq.heappush(Q,(temp_cost,new_node))
                    elif new_node in closed_list:
                        if temp_cost < C2C[new_node]:
                            parent[new_node] = currentNode
                            C2C[new_node] = temp_cost

    cv2.destroyAllWindows()
    return path,closed_list,parent
                        

# %%
while True:
    sn = (input("Enter start node (x,y) as 'x y': ").split())
    start_node = (int(sn[0]),int(sn[1]))
    if (start_node in obstacle_space):
        print('\n Start node is in obstacle space, enter again')
        continue
    gn = (input("Enter goal node (x,y) as 'x y': ").split())
    goal_node = (int(gn[0]),int(gn[1]))
    if (goal_node in obstacle_space):
            print('\nGoal node is in obstacle space, enter again')
            continue
    print('Entered Start node is: ', start_node)
    print('Entered Goal node is: ', goal_node)
    break

start_time = time.time()
f_path,closedlist,parents = djikstra(start_node,goal_node)
#animate_path(f_path,canvas)
end_time = time.time()
print('\nExecution Time: ', end_time - start_time, "s")
#animate_djikstra(closedlist,f_path,canvas)

# %%
animate_path(f_path,canvas)

# %%
animate_djikstra(closedlist,f_path,canvas)

# %%


# %%
len(f_path)

# %%



