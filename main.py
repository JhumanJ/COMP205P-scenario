#Imports
import ast, turtle, time

#Open File


f = open('robots.mat.txt','r')
for i in range (1,4):
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

    # print('\nGraph '+str(i)+':')
    # print("Points ("+str(len(points))+"): " + str(points))
    # print("Obstacles ("+str(len(obstacles))+"): " + str(obstacles))

    # Matrix of paths
    paths = []

    for i in range(0,len(points)):
        tempList = []
        for j in range(0,len(points)):
            if i == i:
                tempList.append(())
            else:
                tempTuple = (i,j)
                print(str(i) +","+str(j))
                tempList.append(tempTuple)
        paths.append(tempList)
        print(tempList)
