#Imports
import ast, turtle, time, math

#Open File
def calc_dist(my_list):
    my_sum = 0
    for item in my_list:
        distance = math.sqrt( ((item[0][0]-item[1][0])**2)+((item[0][1]-item[1][1])**2) )
        my_sum = my_sum + distance
    return distance

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
                    tempList.append(())
                else:
                    tempTuple = [points[i],points[j]]
                    tempList.append(tempTuple)
            paths.append(tempList)
            print(tempList)

        # Base case
        # if len(obstacles)==0:

main()
