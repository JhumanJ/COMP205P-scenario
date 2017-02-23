#Imports
import ast, math, os, time, random
import matplotlib.pyplot as plt

# -------------------- ROBOT PATH GENERATION -----------------------

#checks if the robot has already been woken up to see if this node still has to be travelled to

def checkInPath(listOfAwakenedRobots, position, length):
    if listOfAwakenedRobots[position] != -1:
        return True
    return False


def addToSolution(listOfPaths, shortestEdge):

    newList = []
    num = len(listOfPaths)-1

    for i in range(num):
        newList.append(listOfPaths[i])

    for i in range(num, num + len(shortestEdge)):
        newList.append(shortestEdge[i-num])

    return newList


def algorithm(matrix):

    paths = matrix
    # Array that stores the indexes of the points of the awaken robots
    listOfAwakenedRobots = []

    # list of the solutions
    listOfPaths = []
    for i in range(len(paths)):
        listOfPaths.append([])

    # All points are initiated to index
    for i in range(len(paths)):
        listOfAwakenedRobots.append(-1)

    listOfAwakenedRobots[0] = 0

    print("Len paths: "+str(len(paths)))
    counter = 0

    for row in range(len(paths)-1): #It covers all the nodes in the graph not counting the return to start node
        print("Algorithm Step: "+str(counter)+"/"+str(len(paths)))
        counter = counter + 1
        shortestEdge = [(0,0),(0,0)]
        botNumber = -1
        my_from = -1

        for awakenBot in range(len(listOfAwakenedRobots)):

            if listOfAwakenedRobots[awakenBot] != -1:

                currentNode = listOfAwakenedRobots[awakenBot]

                for column in range(len(paths)):

                    if checkInPath(listOfAwakenedRobots, column, len(paths)) == False:

                        if shortestEdge == [(0,0),(0,0)] and matrix[currentNode][column] !=0: #Initialisation of the shortest distance
                            shortestEdge = matrix[currentNode][column]
                            botNumber = awakenBot
                            my_from = column

                        if calc_dist(matrix[currentNode][column]) < calc_dist(shortestEdge): #It checks which edge is the shortest
                            shortestEdge = matrix[currentNode][column]
                            botNumber = awakenBot
                            my_from = column

        listOfAwakenedRobots[botNumber] = my_from
        listOfAwakenedRobots[my_from] = my_from
        # print ("Len:"+str(len(listOfPaths)) + " index: " + str(botNumber))
        listOfPaths[botNumber] = addToSolution(listOfPaths[botNumber], shortestEdge)

    return listOfPaths

def calc_dist(my_list):
    my_sum = 0
    for i in range(len(my_list)-1):
        distance = math.sqrt( ((my_list[i][0]-my_list[i+1][0])**2)+((my_list[i][1]-my_list[i+1][1])**2) )
        my_sum = my_sum + distance
    return my_sum

# --------------------------- GENERATE MATRIX ------------

def onSegment((xi,yi),(xj,yj),(xk,yk)):
    return  (xi <= xk or xj <= xk) and (xk <= xi or xk <= xj) and (yi <= yk or yj <= yk) and (yk <= yi or yk <= yj)
def det(a, b):
    return a[0] * b[1] - a[1] * b[0]
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    div = det(xdiff, ydiff)
    if div == 0:
       return False
    else:
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        if ((line1[0][0]<=x<=line1[1][0] or line1[1][0]<=x<=line1[0][0]) and (line2[0][0]<=x<=line2[1][0] or line2[1][0]<=x<=line2[0][0]) and (line1[1][1]<=y<=line1[0][1] or line1[0][1]<=y<=line1[1][1]) and (line2[0][1]<=y<=line2[1][1] or line2[1][1]<=y<=line2[0][1])):
            return (x,y)
        else:
            return False

#find next corder of obstacle
def next_obstacle_point(point,obstacle):
    if not point in obstacle:
        return False

    index = obstacle.index(point)

    index = index + 1
    if index == len(obstacle):
        index = 0

    return obstacle[index]

def prev_obstacle_point(point,obstacle):
    if not point in obstacle:
        return False

    index = obstacle.index(point)

    index = index - 1
    if index == -1:
        index = len(obstacle)-1

    return obstacle[index]

#check if segment intersect with obstacle
def does_intersect(x,y,obstacle):
    for i in range(0,len(obstacle)-1):
        my_intersect = line_intersection((x,y),(obstacle[i],obstacle[i+1]))
        if my_intersect != False:
            return True
        # print("Intersection of "+str(start_point)+str(destination_point)+" "+str(obstacle[i])+str(obstacle[i+1])+" -> "+str(my_intersect))
    if line_intersection((x,y),(obstacle[0],obstacle[len(obstacle)-1])) != False:
            return True
    return False

# return minimum
def min_intersect(x,y,obstacle):
    min_inter = 100000000
    min_point = ()

    count = 0
    for i in range(0,len(obstacle)-1):
        my_intersect = line_intersection((x,y),(obstacle[i],obstacle[i+1]))
        if my_intersect != False:
            if calc_dist([x,my_intersect])<min_inter:
                min_inter = calc_dist([x,my_intersect])
                min_point = my_intersect
                count = i

    my_intersect = line_intersection((x,y),(obstacle[0],obstacle[len(obstacle)-1]))
    if my_intersect != False:

        if calc_dist([x,my_intersect])<min_inter:
            min_inter = calc_dist([x,my_intersect])
            min_point = my_intersect
            count = len(obstacle)-1

    return [min_point,count]

#return shortest distance to obstacle
# def dist_to_obstacle(point,destination,obstacle):
#     distance = 0
#     # my_intersect =
#         dist = calc_dist([point,obstacle[i]])
#         if distance==0:
#             distance=dist
#         elif dist<distance:
#             distance=dist
#
#     return distance

#Check if path between point and obstacle is doable
def can_reach_obstacle_point(point,obstacle_point,obstacle):
    my_obstacle = obstacle[:]
    my_obstacle.pop(my_obstacle.index(obstacle_point))
    for my_point in my_obstacle:
        if line_intersection((point,obstacle_point),(my_point,next_obstacle_point(my_point,my_obstacle)))!=False:
            return False
    return True


def path_intersection(path,point_to_add):
    for i in range(0,len(path)-3):
        # print("Intersect of "+str((path[i],path[i+1],path[len(path)-2],point_to_add)))
        my_intersect = line_intersection((path[i],path[i+1]),(path[len(path)-2],point_to_add))
        if my_intersect != False:
            return True
        # print("Intersection of "+str(start_point)+str(destination_point)+" "+str(obstacle[i])+str(obstacle[i+1])+" -> "+str(my_intersect))
    return False


#take a path between two points and expanding to avoid obstacles (version right)
def checkPathAlternate(mypath, obstacles):

    # print("New Obstacle")
    path = mypath[:]

    # print(path)

    start_point = path[len(path)-2]
    destination_point = path[len(path)-1]

    old_obstacle=[]
    if len(path)>2:
        #retrieve previous obstacle
        old_obstacle = obstacles[len(obstacles)-1]
        obstacles.pop(len(obstacles)-1)

    obstacle_to_avoid = []
    obstacle_dist = 0
    intersection = []

    my_random = random.randint(0, 1)

    #find obstacle to avoid
    for obstacle in obstacles:

        if (does_intersect(start_point,destination_point,obstacle)):
            if obstacle_to_avoid == []:
                obstacle_to_avoid = obstacle
                intersection = min_intersect(start_point,destination_point,obstacle)
                obstacle_dist = calc_dist([start_point,intersection[0]])

                # print("\n\nObstacle intialized:"+str(obstacle)+" Does intersect:"+str(does_intersect(start_point,destination_point,obstacle))+" with a dist of "+ str(dist_to_obstacle(start_point,obstacle)) +"\n\n")

            temp_inter = min_intersect(start_point,destination_point,obstacle)
            if calc_dist([start_point,temp_inter[0]]) < obstacle_dist:
                # print("\n\nObstacle updated( "+str(obstacle[0])+" ) (old distance ="+str(obstacle_dist)+") new = "+str(dist_to_obstacle(start_point,obstacle)))
                obstacle_to_avoid = obstacle
                intersection = temp_inter
                obstacle_dist = calc_dist([start_point,intersection[0]])



    if obstacle_to_avoid ==[]:
        return path

    #if the point found make the path cross keep turning aroung previous obstacle
    if old_obstacle !=[] and path_intersection(path,intersection[0]):
        if my_random:
            next_point = prev_obstacle_point(path[len(path)-2],old_obstacle)
        else:
            next_point = next_obstacle_point(path[len(path)-2],old_obstacle)

        #while cross path
        # temp_count = 0
        while path_intersection(path,intersection[0]) :
            # print('cross path')
            path.insert(len(path)-1,next_point)
            if my_random:
                next_point = prev_obstacle_point(next_point,old_obstacle)
            else:
                next_point = next_obstacle_point(next_point,old_obstacle)
            # temp_count = temp_count +1
            # # if temp_count > len(old_obstacle):
            # #     return path
        #whiile point isnt reachable
        while (not can_reach_obstacle_point(destination_point,path[len(path)-2],old_obstacle)):
            # print('not reachable')
            path.insert(len(path)-1,next_point)

            if my_random:
                next_point = prev_obstacle_point(next_point,old_obstacle)
            else:
                next_point = next_obstacle_point(next_point,old_obstacle)

    #first we connect to the point of intersection
    path.insert(len(path)-1,intersection[0])
    #then we join the corner

    if my_random:
        path.insert(len(path)-1,prev_obstacle_point(obstacle_to_avoid[intersection[1]],obstacle_to_avoid))
    else:
        path.insert(len(path)-1,next_obstacle_point(obstacle_to_avoid[intersection[1]],obstacle_to_avoid))

    if my_random:
        next_point = prev_obstacle_point(path[len(path)-2],obstacle_to_avoid)
    else:
        next_point = next_obstacle_point(path[len(path)-2],obstacle_to_avoid)

    #we follow wall while it can't reach obstacle
    while (not can_reach_obstacle_point(destination_point,path[len(path)-2],obstacle_to_avoid)):
        # print('cant reach')
        path.insert(len(path)-1,next_point)
        if my_random:
            next_point = prev_obstacle_point(next_point,obstacle_to_avoid)
        else:
            next_point = next_obstacle_point(next_point,obstacle_to_avoid)


    #obstacle has been passed know we need to do the same thing for the next obstacle
    remaining_obstacles = obstacles[:]
    remaining_obstacles.remove(obstacle_to_avoid)
    remaining_obstacles.append(obstacle_to_avoid)

    # print("remaining_obstacles:"+str(remaining_obstacles))
    # print("Path:"+str(path))

    return checkPath(path, remaining_obstacles)



def checkPathRight(mypath, obstacles):

    # print("New Obstacle")
    path = mypath[:]

    # print(path)

    start_point = path[len(path)-2]
    destination_point = path[len(path)-1]

    old_obstacle=[]
    if len(path)>2:
        #retrieve previous obstacle
        old_obstacle = obstacles[len(obstacles)-1]
        obstacles.pop(len(obstacles)-1)

    obstacle_to_avoid = []
    obstacle_dist = 0
    intersection = []

    #find obstacle to avoid
    for obstacle in obstacles:

        if (does_intersect(start_point,destination_point,obstacle)):
            if obstacle_to_avoid == []:
                obstacle_to_avoid = obstacle
                intersection = min_intersect(start_point,destination_point,obstacle)
                obstacle_dist = calc_dist([start_point,intersection[0]])

                # print("\n\nObstacle intialized:"+str(obstacle)+" Does intersect:"+str(does_intersect(start_point,destination_point,obstacle))+" with a dist of "+ str(dist_to_obstacle(start_point,obstacle)) +"\n\n")

            temp_inter = min_intersect(start_point,destination_point,obstacle)
            if calc_dist([start_point,temp_inter[0]]) < obstacle_dist:
                # print("\n\nObstacle updated( "+str(obstacle[0])+" ) (old distance ="+str(obstacle_dist)+") new = "+str(dist_to_obstacle(start_point,obstacle)))
                obstacle_to_avoid = obstacle
                intersection = temp_inter
                obstacle_dist = calc_dist([start_point,intersection[0]])



    if obstacle_to_avoid ==[]:
        return path

    #if the point found make the path cross keep turning aroung previous obstacle
    if old_obstacle !=[] and path_intersection(path,intersection[0]):
        next_point = prev_obstacle_point(path[len(path)-2],old_obstacle)


        # temp_count = 0
        #while cross path
        while path_intersection(path,intersection[0]) :
            print('cross path')
            path.insert(len(path)-1,next_point)
            next_point = prev_obstacle_point(next_point,old_obstacle)
            # temp_count = temp_count +1
            # if temp_count > len(old_obstacle):
            #     return path
        #whiile point isnt reachable
        while (not can_reach_obstacle_point(destination_point,path[len(path)-2],old_obstacle)):
            # print('not reachable')
            path.insert(len(path)-1,next_point)
            next_point = prev_obstacle_point(next_point,old_obstacle)

    #first we connect to the point of intersection
    path.insert(len(path)-1,intersection[0])
    #then we join the corner
    path.insert(len(path)-1,prev_obstacle_point(obstacle_to_avoid[intersection[1]],obstacle_to_avoid))
    next_point = prev_obstacle_point(path[len(path)-2],obstacle_to_avoid)

    #we follow wall while it can't reach obstacle
    while (not can_reach_obstacle_point(destination_point,path[len(path)-2],obstacle_to_avoid)):
        # print('cant reach')
        path.insert(len(path)-1,next_point)
        next_point = prev_obstacle_point(next_point,obstacle_to_avoid)

    #obstacle has been passed know we need to do the same thing for the next obstacle
    remaining_obstacles = obstacles[:]
    remaining_obstacles.remove(obstacle_to_avoid)
    remaining_obstacles.append(obstacle_to_avoid)

    # print("remaining_obstacles:"+str(remaining_obstacles))
    # print("Path:"+str(path))

    return checkPath(path, remaining_obstacles)


def checkPath(mypath, obstacles):

    # print("New Obstacle")
    path = mypath[:]

    # print(path)

    start_point = path[len(path)-2]
    destination_point = path[len(path)-1]

    old_obstacle=[]
    if len(path)>2:
        #retrieve previous obstacle
        old_obstacle = obstacles[len(obstacles)-1]
        obstacles.pop(len(obstacles)-1)

    obstacle_to_avoid = []
    obstacle_dist = 0
    intersection = []

    #find obstacle to avoid
    for obstacle in obstacles:

        if (does_intersect(start_point,destination_point,obstacle)):
            if obstacle_to_avoid == []:
                obstacle_to_avoid = obstacle
                intersection = min_intersect(start_point,destination_point,obstacle)
                obstacle_dist = calc_dist([start_point,intersection[0]])

                # print("\n\nObstacle intialized:"+str(obstacle)+" Does intersect:"+str(does_intersect(start_point,destination_point,obstacle))+" with a dist of "+ str(dist_to_obstacle(start_point,obstacle)) +"\n\n")

            temp_inter = min_intersect(start_point,destination_point,obstacle)
            if calc_dist([start_point,temp_inter[0]]) < obstacle_dist:
                # print("\n\nObstacle updated( "+str(obstacle[0])+" ) (old distance ="+str(obstacle_dist)+") new = "+str(dist_to_obstacle(start_point,obstacle)))
                obstacle_to_avoid = obstacle
                intersection = temp_inter
                obstacle_dist = calc_dist([start_point,intersection[0]])



    if obstacle_to_avoid ==[]:
        return path

    #if the point found make the path cross keep turning aroung previous obstacle
    if old_obstacle !=[] and path_intersection(path,intersection[0]):
        next_point = next_obstacle_point(path[len(path)-2],old_obstacle)

        #while cross path
        # temp_count = 0
        while path_intersection(path,intersection[0]) :
            print('cross path')
            path.insert(len(path)-1,next_point)
            next_point = next_obstacle_point(next_point,old_obstacle)
            # temp_count = temp_count +1
            # if temp_count > len(old_obstacle):
            #     return path
        #whiile point isnt reachable
        while (not can_reach_obstacle_point(destination_point,path[len(path)-2],old_obstacle)):
            print('not reachable')
            path.insert(len(path)-1,next_point)
            next_point = next_obstacle_point(next_point,old_obstacle)

    #first we connect to the point of intersection
    path.insert(len(path)-1,intersection[0])
    #then we join the corner
    path.insert(len(path)-1,next_obstacle_point(obstacle_to_avoid[intersection[1]],obstacle_to_avoid))
    next_point = next_obstacle_point(path[len(path)-2],obstacle_to_avoid)

    #we follow wall while it can't reach obstacle
    while (not can_reach_obstacle_point(destination_point,path[len(path)-2],obstacle_to_avoid)):
        # print('cant reach')
        path.insert(len(path)-1,next_point)
        next_point = next_obstacle_point(next_point,obstacle_to_avoid)

    #obstacle has been passed know we need to do the same thing for the next obstacle
    remaining_obstacles = obstacles[:]
    remaining_obstacles.remove(obstacle_to_avoid)
    remaining_obstacles.append(obstacle_to_avoid)

    # print("remaining_obstacles:"+str(remaining_obstacles))
    # print("Path:"+str(path))

    return checkPath(path, remaining_obstacles)

def main():
    # open input data
    f = open('robots.mat.txt','r')

    #---------------prepare output data in txt file---------------
    outputr = open('output.txt','r')
    lines = outputr.readlines()
    outputr.close()

    outputw = open('output.txt','w')
    #delete all line
    outputw.seek(0)
    outputw.truncate()
    # re write code and password
    outputw.write(lines[0])
    outputw.write(lines[1])

    first = 14
    last = 14
    for jump in range(1,first):
        text = f.readline()

    for problem in range (first,last+1):
        start_time = time.time()
        text = f.readline().partition("\n")[0]
        text = text.partition(": ")[2]
        text = text.partition("#")
        #Read file
        points = text[0]
        temp_obstacles = text[2].split(";")

        #Parse Points
        try:
            points = list(ast.literal_eval(points))
        except(SyntaxError,ValueError):
            pass

        #Parse obstacles
        obstacles = []
        for item in temp_obstacles:
            temp_item=()
            try:
                temp_item = list(ast.literal_eval(item))
            except(SyntaxError,ValueError):
                pass
            if temp_item != ():
                obstacles.append(temp_item)


        # Display info
        print('\nGraph '+str(problem)+'.')
        # print("Points ("+str(len(points))+"): " + str(points))
        # print("Obstacles ("+str(len(obstacles))+"): " + str(obstacles))
        # print("MATRIX:\n")


        # ------------ Matrix of paths ------------
        paths = []

        for i in range(0,len(points)):
            tempList = []
            for j in range(0,len(points)):
                if i == j:
                    tempList.append([()])
                else:
                    tempTuple = [points[i],points[j]]
                    tempList.append(tempTuple)
            paths.append(tempList)

            # print(tempList)
        # print "\n\n"
        # print(paths)
        # print "\n\n"


        # Intersections handler / checkpath
        status = (len(points)**2)/2
        counter = 0

        for index_i in range(0,len(points)):
            for index_j in range(index_i+1,len(points)):
                if paths[index_i][index_j]!=[()]:
                    # For this path find closest intersection
                    paths[index_i][index_j] = checkPathRight(paths[index_i][index_j],obstacles)

                    # tempalternate = checkPathAlternate(paths[index_i][index_j],obstacles)
                    # tempRight = checkPathRight(paths[index_i][index_j],obstacles)
                    # tempLeft = checkPath(paths[index_i][index_j],obstacles)
                    #
                    # if calc_dist(tempalternate)<calc_dist(tempRight) and calc_dist(tempalternate)<calc_dist(tempLeft):
                    #     paths[index_i][index_j] = tempalternate
                    #
                    # elif calc_dist(tempRight)<calc_dist(tempalternate) and calc_dist(tempRight)<calc_dist(tempLeft):
                    #     paths[index_i][index_j] = tempRight
                    #
                    # else:
                    #     paths[index_i][index_j] = tempLeft

                    paths[index_j][index_i] = paths[index_i][index_j][:]
                    paths[index_j][index_i].reverse()

                    print("Optimize paths: "+str(counter)+"/"+str(status))
                    counter = counter + 1

        #     print(paths[index_i])
        # print "\n\n"

        print("--Optimized matrix execution time:" + str(time.time() - start_time))


        #----------------------Vizualization------------------------
        fig=plt.figure()
        ax=fig.add_subplot(111)

        # ----Obstacles
        if len(obstacles)>0:
            for obstacle in obstacles:
                x=[]
                y=[]
                for i in range(0,len(obstacle)):
                    x.append(obstacle[i][0])
                    y.append(obstacle[i][1])
                x.append(obstacle[0][0])
                y.append(obstacle[0][1])
                plt.plot(x,y,'k')

        # ----Points
        for point in points:
            plt.scatter(point[0],point[1])
        plt.title("Graph "+str(problem))


        # ----Paths
        result_path = algorithm(paths)
        # print(result_path)

        for my_path in result_path:
            x=[]
            y=[]
            for i in range(0,len(my_path)):
                x.append(my_path[i][0])
                y.append(my_path[i][1])
            plt.plot(x,y)



        plt.savefig('graph/'+str(problem)+'.png')

        # =--------Clear path
        for item in result_path:
            if item==[]:
                result_path.remove(item)
            for indx in range(len(item)-2):
                if item[indx]==item[indx+1]:
                    item.pop(indx)

        #----------write line
        outputw.write(str(problem)+": ")
        for item in result_path:
            for indx in range(len(item)):
                outputw.write(str(item[indx]))
                if (indx == (len(item)-1)):
                        outputw.write(";")
                else:
                    outputw.write(",")
        outputw.seek(-1, os.SEEK_END)
        outputw.truncate()
        outputw.write("\n")

        print("--Total execution time: " + str(time.time() - start_time) + "sec.")


    #close input
    outputw.close()
    f.close()

main()
