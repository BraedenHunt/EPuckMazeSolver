from Graph import Graph
from Djikstra import *
from math import trunc

class MazeReader:

    def mazeRead(self, maze):
        #Assumes start of 11.06
        rowNodes = []
        colNodes = []
        rowNodesAll = []
        colNodesAll = []
        edges = []
        for i in range(1,13):
            for j in range(1,13):
                if (maze[i][j] == '+'):
                    rowNodesAll.append(i+(j/100))
                    colNodesAll.append(j+(i/100))

                    #count the number of walls adjacent to the space
                    count = 0
                    upwall = 0
                    downwall = 0
                    leftwall = 0
                    rightwall = 0

                    if maze[i-1][j] == '#' or maze[i-1][j] == '?': #wall above
                        count+=1
                        upwall = 1

                    if maze[i+1][j] == '#' or maze[i+1][j] == '?': #wall below
                        count+=1
                        downwall = 1

                    if maze[i][j-1] == '#' or maze[i][j-1] == '?': #wall to the left
                        count+=1
                        leftwall = 1

                    if maze[i][j+1] == '#' or maze[i][j+1] == '?': #wall to the right
                        count+=1
                        rightwall = 1


                    if count == 3:
                        rowNodes.append(i+(j/100))
                        colNodes.append(j+(i/100))
                    elif count == 2:
                        if (upwall and downwall):
                            continue
                        elif (leftwall and rightwall):
                            continue
                        else:
                            rowNodes.append(i+(j/100))
                            colNodes.append(j+(i/100))
                    elif count == 1:
                        rowNodes.append(i+(j/100))
                        colNodes.append(j+(i/100))
                    elif count == 0:
                        rowNodes.append(i+(j/100))
                        colNodes.append(j+(i/100))
                elif (maze[i][j] == '@'):
                    rowNodesAll.append(i+(j/100))
                    colNodesAll.append(j+(i/100))
                    rowNodes.append(i+(j/100))
                    colNodes.append(j+(i/100))
                    goal = i+j/100

        #Add start node
        if 11.06 not in rowNodes:
            rowNodes.append(11.06)
        if 6.11 not in colNodes:
            colNodes.append(6.11)

        #sort node lists in ascending order
        rowNodes.sort()
        colNodes.sort()
        rowNodesAll.sort()
        colNodesAll.sort()

        #add horizontal paths
        for i in range(len(rowNodes)-1):
            if trunc(rowNodes[i]) == trunc(rowNodes[i+1]):
                row = trunc(rowNodes[i])
                temp1 = rowNodes[i]
                temp2 = rowNodes[i+1]
                dist = 0

                j = rowNodesAll.index(temp1)
                while trunc(rowNodesAll[j]) == row:
                    if rowNodesAll[j] == temp2:
                        #print("Adding Edge between " + str(temp1) + " and " + str(temp2) + " of length " + str(dist))
                        edges.append([temp1,temp2,dist])
                        break
                    elif (round(rowNodesAll[j+1] - rowNodesAll[j],2)) == 0.01:
                        dist+=1
                        j+=1
                        continue
                    else:
                        break

        #add vertical paths
        for i in range(len(colNodes)-1):
            if trunc(colNodes[i]) == trunc(colNodes[i+1]):
                row = trunc(colNodes[i])
                temp1 = colNodes[i]
                temp2 = colNodes[i+1]
                dist = 0

                j = colNodesAll.index(temp1)
                while trunc(colNodesAll[j]) == row:
                    if colNodesAll[j] == temp2:
                        tempRow1 = round((temp1-trunc(temp1))*100+trunc(temp1)/100,2)
                        tempRow2 = round((temp2-trunc(temp2))*100+trunc(temp2)/100,2)
                        #print("Adding Edge between " + str(tempRow1) + " and " + str(tempRow2) + " of length " + str(dist))
                        edges.append([tempRow1,tempRow2,dist])
                        break
                    elif (round(colNodesAll[j+1] - colNodesAll[j],2)) == 0.01:
                        dist+=1
                        j+=1
                        continue
                    else:
                        break


        nodes = rowNodes

        return nodes, edges, goal

    def print_result(self, previous_nodes, shortest_path, start_node, target_node):
        path = []
        node = target_node

        while node != start_node:
            path.append(str(node))
            node = previous_nodes[node]

        # Add the start node manually
        path.append(str(start_node))

        newPath = []
        for i in reversed(path):
            newPath.append(i)

        #print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
        #print(" -> ".join(reversed(path)))
        return newPath

    def GiveDirections(self, _maze):
        directions = []
        nodes,edges,goal = self.mazeRead(_maze)

        init_graph = {}
        for node in nodes:
            init_graph[node] = {}

        for edge in edges:
            init_graph[edge[0]][edge[1]] = edge[2]

        graph = Graph(nodes, init_graph)

        previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node=11.06)

        strPath = self.print_result(previous_nodes, shortest_path, start_node=11.06, target_node=goal)

        #make path of floats
        path = []
        for step in strPath:
            path.append(float(step))

        #Print Directions
        i = 0
        dirc = ''
        prevDirc = ''
        startNode = path[i]
        while i < len(path)-1:

            #check direction
            if trunc(path[i]) > trunc(path[i+1]):
                dirc = 'up'
            elif trunc(path[i]) < trunc(path[i+1]):
                dirc = 'down'
            elif round(path[i]-trunc(path[i]),2) > round(path[i+1]-trunc(path[i+1]),2):
                dirc = 'left'
            else:
                dirc = 'right'

            if prevDirc == '':
                prevDirc = dirc;
            else:
                if (dirc == prevDirc):
                    i+=1
                    continue
                else:
                    if prevDirc == 'up' or dirc == 'down':
                        dist = abs(trunc(startNode)-trunc(path[i]))
                    else:
                        dist = int(100*abs(round((round(startNode-trunc(startNode),2)-round(path[i]-trunc(path[i]),2)),2)))
                    directions.append([prevDirc, dist])
                    #print("Go " + prevDirc + " " + str(dist) + " spaces.")
                    prevDirc = dirc
                    startNode = path[i]


            i+=1
        startNode = path[len(path)-2]
        if dirc == 'up' or dirc == 'down':
            dist = abs(trunc(startNode)-trunc(path[len(path)-1]))
        else:
            dist = int(100*abs(round((round(startNode-trunc(startNode),2)-round(path[i]-trunc(path[i]),2)),2)))
        directions.append([prevDirc, dist])
        #print("Go " + str(prevDirc) + " " + str(dist) + " spaces.")

        return directions


###################################################################################################################
maze = [['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'],
        
        ['#', '+', '+', '@', '+', '+', '#', '+', '+', '+', '+', '+', '#', '#'],
        
        ['#', '+', '#', '#', '+', '#', '+', '#', '#', '#', '#', '+', '#', '#'],
        
        ['#', '+', '#', '?', '+', '#', '+', '#', '+', '+', '+', '+', '+', '#'],
        
        ['#', '+', '?', '#', '+', '+', '+', '#', '#', '#', '#', '+', '#', '#'],

        ['#', '+', '#', '?', '#', '#', '+', '#', '#', '#', '#', '+', '#', '#'],
        
        ['#', '+', '#', '?', '?', '#', '+', '+', '+', '+', '+', '+', '+', '#'],

        ['#', '+', '#', '?', '?', '#', '+', '#', '#', '#', '#', '+', '#', '#'],
        
        ['#', '+', '#', '?', '#', '#', '+', '#', '?', '?', '#', '+', '#', '#'],
        
        ['#', '+', '+', '+', '+', '+', '+', '#', '?', '?', '#', '+', '#', '#'],
        
        ['#', '#', '#', '#', '+', '#', '+', '#', '#', '#', '#', '+', '#', '#'],
        
        ['#', '#', '#', '#', '+', '#', '+', '#', '+', '+', '+', '+', '#', '#'],
        
        ['#', '+', '+', '+', '+', '#', '+', '#', '#', '#', '#', '+', '+', '#'],
        
        ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#']]

    
