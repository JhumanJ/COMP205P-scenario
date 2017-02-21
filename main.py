#Imports
import ast, math
import matplotlib.pyplot as plt
import itertools

# from openpyxl import Workbook


# -------------------- ROBOT PATH GENERATION -----------------------

#Open File
def calc_dist(my_list):
    my_sum = 0
    for i in range(len(my_list)-1):
        distance = math.sqrt( ((my_list[i][0]-my_list[i+1][0])**2)+((my_list[i][1]-my_list[i+1][1])**2) )
        my_sum = my_sum + distance
    return my_sum

def checkInPath(listOfAwakenedRobots, position, length):
    for k in range(length):
        if listOfAwakenedRobots[k] != -1:
            if position == k:
                return True
	return False


def addToSolution(listOfPaths, shortestEdge, botNumber):

	newList = []

	for i in range(len(listOfPaths[botNumber])-1):
		newList[i] = listOfPaths[botNumber][i]

	for i in range(len(listOfPaths[botNumber])-1,len(listOfPaths[botNumber])-1 + len(shortestEdge)):
		newList[i] = shortestEdge[i-len(listOfPaths[botNumber])-1]

	return newList


def algorithm(matrix):

    paths = matrix
	# Array that stores the indexes of the points of the awaken robots
    listOfAwakenedRobots = [len(paths)]

	# list of the solutions
    listOfPaths = []

	# All points are initiated to index
    for i in range(len(paths)):
        listOfAwakenedRobots[i] = -1

    listOfAwakenedRobots[0] = 0

    for row in range(len(paths)-1): #It covers all the nodes in the graph not counting the return to start node

        shortestEdge = [(0,0)(0,0)]
        botNumber = -1
        my_from = -1

        for awakenBot in range(listOfAwakenedRobots):
            if listOfAwakenedRobots[awakenBot] != -1:
                currentNode = listOfAwakenedRobots[awakenBot]

                for column in range(len(paths)):    #It looks at all unvisited point from the current point it is visiting

	               if checkInPath(listOfAwakenedRobots, column, len(paths)):

                        if shortestEdge == [(0,0)(0,0)] and matrix[currentNode][column] !=0: #Initialisation of the shortest distance
                            shortestEdge = matrix[currentNode][column]
                            botNumber = awakenBot
                            my_from = column

                        if calc_dist(matrix[currentNode][j]) < calc_dist(shortestEdge): #It checks which edge is the shortest
                            shortestEdge = matrix[currentNode][column]
                            botNumber = awakenBot
                            my_from = column

		listOfAwakenedRobots[botNumber] = botNumber
        listOfAwakenedRobots[my_from] = botNumber
        listOfPaths = addToSolution(listOfPaths, shortestEdge, botNumber)

	return listOfPaths

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

def next_obstacle_point(point,obstacle):
    if not point in obstacle:
        return False

    index = obstacle.index(point)
    if index == len(obstacle):
        index = 0
    else:
        index = index + 1
    return obstacle[index]

def checkPath(path, obstacles):

    start_point = path[len(path)-2]
    destination_point = path[len(path)-1]


    # Todo vars
    closest_inter_point = ()
    intersect_segment = ()
    intersection_obstacle = []

    #init todo var
    for obstacle in obstacles:
        for i in range(0,len(obstacle)-1):
            my_intersect = line_intersection((start_point,destination_point),(obstacle[i],obstacle[i+1]))
            if my_intersect != False:
                if closest_inter_point == ():
                    closest_inter_point = my_intersect
                    intersect_segment = (obstacle[i],obstacle[i+1])
                    intersection_obstacle = obstacle
                elif calc_dist([start_point,closest_inter_point])>calc_dist([start_point,my_intersect]):
                    closest_inter_point = my_intersect
                    intersect_segment = (obstacle[i],obstacle[i+1])
                    intersection_obstacle = obstacle
            # print("Intersection of "+str(start_point)+str(destination_point)+" "+str(obstacle[i])+str(obstacle[i+1])+" -> "+str(my_intersect))
        my_intersect = line_intersection((start_point,destination_point),(obstacle[0],obstacle[len(obstacle)-1]))
        if my_intersect != False:
            if closest_inter_point == ():
                closest_inter_point = my_intersect
                intersect_segment = (obstacle[i],obstacle[i+1])
                intersection_obstacle = obstacle
            elif calc_dist([start_point,closest_inter_point])>calc_dist([start_point,my_intersect]):
                closest_inter_point = my_intersect
                intersect_segment = (obstacle[i],obstacle[i+1])
                intersection_obstacle = obstacle
        # print("Intersection of "+str(start_point)+str(destination_point)+" "+str(obstacle[0])+str(obstacle[len(obstacle)-1])+" -> "+str(my_intersect))
        # print("---")

    if intersection_obstacle != []:
        #inersection with an obstacle

        #find closest point of obstacle
        closest_point_of_obstacle = ()
        for point in intersection_obstacle:
            if closest_point_of_obstacle == ():
                closest_point_of_obstacle = point
            elif calc_dist([start_point,closest_point_of_obstacle])>calc_dist([start_point,point]):
                closest_point_of_obstacle = point

        new_path = path
        new_path.insert(len(new_path)-1,closest_point_of_obstacle)


        # Si il y a une intersection entre le dernier segment et l'obstacle continuer de contourner
        still_inter = True
        while still_inter:
            still_inter = False
            for i in range(0,len(intersection_obstacle)-2):

                if line_intersection((new_path[len(new_path)-2],new_path[len(new_path)-1]),(intersection_obstacle[i],intersection_obstacle[i+1])) != False:
                    still_inter = True
                    print(str(intersection_obstacle[i])+" "+str(intersection_obstacle[i+1]))


            if line_intersection((new_path[len(new_path)-2],new_path[len(new_path)-1]),(intersection_obstacle[0],intersection_obstacle[len(intersection_obstacle)-1])) != False:
                still_inter = True
                print(str(intersection_obstacle[i])+" "+str(intersection_obstacle[i+1]))


            new_path.insert(len(new_path)-1,next_obstacle_point(closest_point_of_obstacle,intersection_obstacle))

        return new_path

    return path

    #  /find closest Intersection

def main():
    f = open('robots.mat.txt','r')

    # Excel code
    # column_number = 1
    # wb = Workbook()
    # ws = wb.active
    # ws.title = "Data"

    for problem in range (1,31):
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

        # Parse obstacles
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
        print('\nGraph '+str(problem)+':')
        print("Points ("+str(len(points))+"): " + str(points))
        print("Obstacles ("+str(len(obstacles))+"): " + str(obstacles))
        print("MATRIX:\n")


        # Matrix of paths
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

        # Intersections
        for index_i in range(0,len(points)):
            for index_j in range(index_i,len(points)):
                if index_i != index_j:
                    # For this path find closest intersection
                    # print( checkPath(paths[index_i][index_j],obstacles))
                    pass

        #Display
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
                plt.plot(x,y)

        # ----Points
        for point in points:
            plt.scatter(point[0],point[1])
        plt.title("Graph "+str(problem))

        plt.show()

main()
