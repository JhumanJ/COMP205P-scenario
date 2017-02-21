#Imports
import ast, math

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

	# Array that stores the indexes of the points of the awaken robots 
	listOfAwakenedRobots = [len(paths)]
	
	# list of the solutions 
	listOfPaths = []

	# All points are initiated to index 
	for i in range(len(paths)):
	 	listOfAwakenedRobots[i] = -1;

	listOfAwakenedRobots[0] = 0;

    for row in range(len(paths)-1): #It covers all the nodes in the graph not counting the return to start node

        shortestEdge = [(0,0)(0,0)]
        botNumber = -1

        for awakenBot in range(listOfAwakenedRobots):

        	if listOfAwakenedRobots[awakenBot] != -1:

        		currentNode = listOfAwakenedRobots[awakenBot]
        
		      	for column in range(len(paths)):    #It looks at all unvisited point from the current point it is visiting

		            if checkInPath(listOfAwakenedRobots, column, len(paths)):

			            if shortestEdge == [(0,0)(0,0)] and matrix[currentNode][column] !=0: #Initialisation of the shortest distance
			                shortestEdge = matrix[currentNode][column]
			                botNumber = awakenBot

			            if calc_dist(matrix[currentNode][j]) < calc_dist(shortestEdge): #It checks which edge is the shortest
			                shortestEdge = matrix[currentNode][column]
			                botNumber = awakenBot 

		listOfAwakenedRobots[botNumber] = 
	    listOfPaths = addToSolution(listOfPaths, shortestEdge, botNumber)

	return listOfPaths

def main():
    f = open('robots.mat.txt','r')
    for i in range (1,2):
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
        print('\nGraph '+str(i)+':')
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
            print(tempList)

        # Base case
        # if len(obstacles)==0:
        



        # for i in range(len(paths)):
        #     for j in range(len(paths)):
        #         if i==j:
        #             print(0)
        #         else:
        #             # print paths[i][j]
        #             print(calc_dist(paths[i][j]))

main()
