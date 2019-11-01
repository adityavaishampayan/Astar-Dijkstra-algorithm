#importing required libraries
import heapq
import matplotlib.pyplot as plt
import numpy as np
import math
import time

start_time = time.time()

startx =int(input("please enter start point x coordinate: "))
starty =int(input("please enter start point y coordinate: "))

goalx =int(input("please enter goal point x coordinate: "))
goaly =int(input("please enter goal point y coordinate: "))

res = 1
rr = int(input("enter robot radius: "))
clearance = int(input("enter clearance for the robot: "))

 
start = (round(startx/res),round(starty/res))
goal = (round(goalx/res),round(goaly/res))
rr = rr/res
clearance = clearance/res
extra = rr + clearance

plt.plot(start[0], start[1], "Dr")
plt.plot(goal[0], goal[1], "Dr")

obstacle = np.zeros(shape=(int(151/res),int(251/res)))

plotx = []
ploty = []

####################################WALLS######################################
ox = []
oy = []

for i in range(round(250/res)):
        ox.append(i)
        oy.append(0)
        obstacle[0][i] = 1
    
        ox.append(i)
        oy.append(int(150/res))
        obstacle[int(149/res)][i] = 1

for i in range(round(150/res)):
        ox.append(0)
        oy.append(i)
        obstacle[i][0] = 1
      
        ox.append(int(250/res))
        oy.append(i)
        obstacle[i][int(249/res)] = 1

#####################################WALLS2####################################

Wx = []
Wy = []

for x in range(round(251/res)):
    for y in range(round(151/res)):
        
        f1 = y - extra
        if f1<0:
            Wx.append((x))
            Wy.append((y))
            
        f2 = x - extra
        if f2<0:
            Wx.append((x))
            Wy.append((y))
            
        f3 = y - (150 - extra)
        if f3>0:
            Wx.append((x))
            Wy.append((y))
            
        f4 = x - (250 - extra)
        if f4>0:
            Wx.append((x))
            Wy.append((y))
                      

plt.scatter(Wx,Wy)
walls = zip(Wx,Wy)

for i in walls:
    obstacle[i[1]][i[0]] = 1


##############################RECTANGLE########################################
points1 = []
points2 = []
points3 = []
points4 = []

for x in range(round(251/res)):
    for y in range(round(151/res)):
        f1 = -y + (67.5/res) - extra
        if f1<=0:
            points1.append((x,y))

for x in range(round(251/res)):
    for y in range(round(151/res)):           
        f1 = y - (112.5/res) - extra
        if f1<=0:
            points2.append((x,y))

for x in range(round(251/res)):
    for y in range(round(151/res)):
        f1 = -x + (50/res) - extra
        if f1<=0:
            points3.append((x,y))
            
for x in range(round(251/res)):
    for y in range(round(151/res)):
        f1 = x - (100/res) - extra
        if f1<=0:
            points4.append((x,y))

rectangle = list(set(points1) & set(points2) & set(points3) & set(points4))            

for i in rectangle:
    obstacle[i[1]][i[0]] = 1
       
xs = [x[0] for x in rectangle]
ys = [x[1] for x in rectangle]
plt.scatter(xs,ys)

################################CIRCLE#########################################

circle = []
for x in range(round(251/res)):
    for y in range(round(151/res)):
        f5 = (x - round(190/res))**2 + (y - round(130/res))**2 - ((15/res) + extra)**2
        if f5<=0:
            circle.append((x,y))

xcircle = [x[0] for x in circle]
ycircle = [x[1] for x in circle]

for i in circle:
    obstacle[i[1]][i[0]] = 1
plt.scatter(xcircle,ycircle)

#############################ELLIPSE###########################################
ellipse = []
major_ax = (15/res) + extra
minor_ax = (6/res) + extra

for x in range(round(251/res)):
    for y in range(round(151/res)):
        f6 = (((x - (140/res))**2)/major_ax**2) + (((y - (120/res))**2)/minor_ax**2) -1
        
        if f6<=0:
            ellipse.append((x,y))

xellipse = [x[0] for x in ellipse]
yellipse = [x[1] for x in ellipse]
plt.scatter(xellipse,yellipse)

for i in ellipse:
    obstacle[i[1]][i[0]] = 1

#####################################POLYGON###################################
shapeA1 = []
shapeA2 = []
shapeA3 = []

shapeB1 = []
shapeB2 = []
shapeB3 = []
shapeB4 = []

shapeC1 = []
shapeC2 = []
shapeC3 = []
shapeC4 = []

for x in range(round(251/res)):
    for y in range(round(151/res)):

################################TRIANGLE1######################################
        f1 = 38*x + 23*y - (extra + 192.04)*44.42
        if f1<=0:
            shapeA1.append((x,y))
               
        f2 = -y + 52 
        if f2<=0:
            shapeA2.append((x,y))
            
        f3 = -38*x + 7*y + (-extra + 150.88)*38.64
        if f3<=0:
            shapeA3.append((x,y))
            
#################################TRIANGLE2#####################################
        f4 = 2*x + 19*y - (extra + 68.77)*19.10 
        if f4<=0:
            shapeB1.append((x,y))   
                       
        f5 =  -41*x -25*y  + (135.88 - extra)*48.02
        if f5<=0:
            shapeB2.append((x,y))
                        
        f6 = 37*x -13*y  - (136.55 )*39.22
        if f6<=0:
            shapeB3.append((x,y))
        
            
################################QUADRILATERAL##################################                
        f7 = y - 52
        if f7<=0:
            shapeC1.append((x,y))
            
            
        f8 = 37*x - 20*y - (145.06 + extra)*42.05
        if f8<=0:
            shapeC2.append((x,y))
            
        f9 = -y + 15 - extra
        if f9<=0:
            shapeC3.append((x,y))
        
        f10 = -37*x + 13*y + (136.55)*39.22
        if f10<=0:
            shapeC4.append((x,y))
                    
poly1 = list(set(shapeA1) & set(shapeA2) & set(shapeA3))
poly2 = list(set(shapeB1) & set(shapeB2) & set(shapeB3))
poly3 = list(set(shapeC1) & set(shapeC2) & set(shapeC3) & set(shapeC4))

xpoly1 = [x[0] for x in poly1]
ypoly1 = [x[1] for x in poly1]

xpoly2 = [x[0] for x in poly2]
ypoly2 = [x[1] for x in poly2]

xpoly3 = [x[0] for x in poly3]
ypoly3 = [x[1] for x in poly3]

final_poly = list(set(poly1) | set(poly2) | set(poly3))
xpolyf = [x[0] for x in final_poly]
ypolyf = [x[1] for x in final_poly]

final_obstacle_space = set(set(rectangle) or set(circle) or set(ellipse) or set(final_poly) or set(walls) or set(zip(ox,oy)) )

for i in final_poly:
    obstacle[i[1]][i[0]] = 1
    
plt.scatter(xpolyf,ypolyf)


obstacle_t = obstacle.T 
obs = []
for i in range(round(250/res)):
    obs.append(obstacle_t[i])
    
plt.scatter(ox,oy)   

def get_motion_model():
    steps = [[1,0,1],
             [0,1,1],
             [-1,0,1],
             [0,-1,1],
             [1,1,math.sqrt(2)],
             [1,-1,math.sqrt(2)],
             [-1,-1,math.sqrt(2)],
             [-1,1,math.sqrt(2)]]
    return steps

def retrace(clist):
    backtrack = []
    l = len(clist)
    current_pos = clist[l-1][1]
    backtrack.append(current_pos)
    parent = clist[l-1][2]
    while parent != None: 
        for i in range(l):
            X = clist[i]
            if X[1] == parent:
                parent = X[2]
                current_pos = X[1]
                backtrack.append(current_pos)
    return backtrack[::-1]  
  
   
def dijkstra_algorithm(start,goal, obstacle):
    
    goal_vertex = (0,goal,None)
    start_vertex = (0,start,None)
    
    open_list = []
    closed_list = []
   
    motion = get_motion_model()
       
    heapq.heappush(open_list,(start_vertex))
    obstacle[start_vertex[1][0]][start_vertex[1][1]] = 1

    while len(open_list)>0:
        current_node = heapq.heappop(open_list)
        heapq.heappush(closed_list,current_node)
        plotx.append(current_node[1][0])
        ploty.append(current_node[1][1])
        
        if len(ploty)%1000 == 0:
            plt.plot(goal[0], goal[1], "Dr")
            plt.plot(plotx,ploty, '.y')
            plt.plot(goal[0], goal[1], "Dr")
            plt.pause(0.001)
                
        if current_node[1] == goal_vertex[1] :
            print('goal coordinates found')
            final_path = retrace(closed_list)
            return final_path                                        
        
        neighbors = []
        
        for new_position in motion:
            
            # Get node position
            node_position = (current_node[1][0] + new_position[0],
                             current_node[1][1] + new_position[1])
            node_position_cost = current_node[0] + new_position[2]
            
            node_heuristic_cost = round((current_node[1][0] - goal_vertex[1][0])**2 + (current_node[1][1] - goal_vertex[1][1])**2)
            final_cost =node_position_cost + node_heuristic_cost
            
            node_parent = current_node[1]
            
            minx = 0
            miny = 0
            maxy = (len(obstacle) - 1)
            maxx = (len(obstacle[0]) -1)
            
            
            # Make sure within range
            if node_position[0] > maxy:
                continue
            
            if node_position[0] < miny:
                continue
            
            if node_position[1] > maxx:
                continue
            
            if node_position[1] < minx:
                continue
            
            # Make sure walkable terrain
            if obstacle[node_position[0]][node_position[1]] != 0:
                continue
            
            #Creating cost_map
            obstacle[node_position[0]][node_position[1]] = 1
            
            new_node = (final_cost,node_position,node_parent)
                
            neighbors.append(new_node)
        
            heapq.heappush(open_list,(new_node))


if start in (zip(ox,oy) or final_obstacle_space):
    print("start node in obstacle space")

elif goal in (zip(ox,oy) or final_obstacle_space):
    print("goal node in obstacle space")

else:
    path = dijkstra_algorithm(start,goal, obs)
    plt.plot(plotx,ploty, '.y')
    if path!= None:
        scatterx = [x[0] for x in path]
        scattery = [x[1] for x in path]
        plt.plot(scatterx,scattery,color = 'c',linewidth = 4)
        elapsed_time = time.time() - start_time
        print("time elapsed: ", elapsed_time)
    else:
        print("path not found")
    