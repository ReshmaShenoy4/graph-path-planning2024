#!/usr/bin/env python3
import matplotlib.pyplot as plt
import time
import networkx as nx
import heapq
import math
from itertools import zip_longest

def read_config(file):
    f = open(file, "r")
    lines = f.readlines()

    n_Hauler = 1
    n_LP = 0
    n_ULP = 0
    n_SO = 0
    n_CS = 1
    init_haulers = []
    LP_positions = []
    ULP_positions = []
    SO_positions = []
    CS_positions = []
    batery_capacity = 0
    initial_energy = 0

    # for following the line number
    index = 1
    for line in lines:
        # read number of Haulers
        if(index == 1):
            n_Hauler = int(line.split("//")[0])

        # read number of LPs
        elif(index == 2):
            n_LP = int(line.split("//")[0])

        # read number of ULPs
        elif(index == 3):
            n_ULP = int(line.split("//")[0])

        # read number of SOs
        elif(index == 4):
            n_SO = int(line.split("//")[0])

        # read number of CSs
        elif(index == 5):
            n_CS = int(line.split("//")[0])

        # read Initial position of haulers
        elif(index == 6):
            t = line.split("//")[0].split("-")
            for i in range(n_Hauler):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                init_haulers.append(x)

        # read position of LPs
        elif (index == 7):
            t = line.split("//")[0].split("-")
            for i in range(n_LP):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                LP_positions.append(x)

        # read position of ULPs
        elif (index == 8):
            t = line.split("//")[0].split("-")
            for i in range(n_ULP):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                ULP_positions.append(x)

        # read position of SOs
        elif (index == 9):
            t = line.split("//")[0].split("-")
            for i in range(n_SO):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                SO_positions.append(x)
        # read position of CSs
        elif (index == 10):
            t = line.split("//")[0].split("-")
            for i in range(n_CS):
                cordinate = t[i].replace("[", "").replace("]", "")
                x = []
                for j in range(2):
                    x.append(int(cordinate.split(",")[j]))
                CS_positions.append(x)
        # read battery capacity
        elif(index == 11):
            if(line.split("//")[0] != '\t'):
                batery_capacity = int(line.split("//")[0])

        # read initial energy
        elif(index == 12):
            if(line.split("//")[0] != '\t'):
                initial_energy = int(line.split("//")[0])
        index += 1
    return init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, batery_capacity, initial_energy


def read_mission(file):
    f = open(file, "r")
    lines = f.readlines()

    mission = []

    for line in lines:
        temp = line.split("\t//")[0].split(",")
        mission.append(temp)

    return mission


def write_output(makespan, completion_times, exec_time, path, file):
    f = open(file, "w")
    f.write("//Quantitative values\n")
    f.write(str(makespan)+"\t//Makespan" + "\n")
    for i, c in enumerate(completion_times):
        f.write(str(c)+"\t//Mission completion time hauler " + str(i+1)+"\n")
    f.write(str(exec_time)+"\t//Application execution time (in millisecond)"+"\n")
    f.write("//Path to the final points\n")
    f.write(path)
    f.close()

################### NEW FUNCTIONS#######################################
def split_strings(inp_list):
    result = [[s[0],int(s[1:])] for s in inp_list]
    return result

def find_path(parent,goal):
    path=[]
    while goal in parent:
        path.append(goal)
        goal = parent[goal]
    path.append(goal)
    return path
##########################################################################       
def main():
    """
    Replace your input files location and name.
    You can also use command line argument to get input files name.

    functions:
        read_config     -> input: config file name 
                        -> output: all variables in config file

        read_mission    -> input: mission file name
                        -> output: mission for each hauler

        write_output    -> input: scenario makespan, completion time of each hauler, elapsed time of program,
                                     path to the final points, output file name
                        -> output: void

    """
    config_file = "config.txt"
    mission_file = "mission.txt"
    output_file = "output.txt"

    #config_file  = "../dataset1/Part3/Hard/14/5_config.txt"
    #mission_file = "../dataset1/Part3/Hard/14/5_mission.txt"
    #output_file  = "../dataset1/Part3/Hard/14/5_output.txt"

    init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, batery_capacity, initial_energy = read_config(
        config_file)

    mission = read_mission(mission_file)

    start = time.time()
    #################################
    # place your main code here!
 
    ###################################################
    # CREATING GRAPH MODEL
    ###################################################
    ip = init_haulers
    hauler_num = len(init_haulers)
    mission_seq=[]
    final_mission = []
    #print(hauler_num)
    for n in range(0,hauler_num):
        mission_seq.append(split_strings(mission[n]))   
        mission_cor = [LP_positions[s[1]-1] if (s[0]=='L') else ULP_positions[s[1]-1] for s in mission_seq[n]]
        mission_cor = [init_haulers[n]] + mission_cor
        final_mission.append(mission_cor)
    #print(f"Final mission is{final_mission}")

    G = nx.Graph()
    #Fnd the grid size
    #print(init_haulers + LP_positions + ULP_positions + SO_positions)
    full_list = init_haulers + LP_positions + ULP_positions + SO_positions
    x_max = max(sublist[0] for sublist in full_list)
    y_max = max(sublist[1] for sublist in full_list)
    #print(full_list)
    #print(x_max)
    #print(y_max)
    grid_size = max(max(fl) for fl in full_list)
    #print(SO_positions)
    #print(f" Grid Size: {grid_size}")
    node_id = 1
    #Adding nodes to the graph, and not adding the ones in static obstacles. The empty nodes in the plot are the obstacles.
    for row in range(1,x_max+1):
        for col in range(1,y_max+1):
            #print(col)
            if [row,col] not in SO_positions:
                node_id = node_id + 1
                G.add_node(node_id,pos=[row,col])
            #else:
                #print([row,col])
                

    #Node needs a string or int. But if we give a tuple(x,y), it goes as attribute. 
    pos =  nx.get_node_attributes(G,'pos')
    #print(pos)

    # For adding attribute pos as a label to the nodes in the plot
    labels = {node:f"{x},{y}" for node,[x,y] in pos.items()}
    visited_nodes = []

    # For every created node, I have a list of neighbours ( up, down, left and right). I now need to create a graph so I connect the node with its neighbor. Since this is 
    # undirected graph, I do connections only once. So, if the neighnor node is a valid node, then I form connections. If a node is already visited that means, all its relevant
    # connections are done. so that I dont form multiple same edges.
    for node_id,[x,y] in pos.items():
        neighbor_list = [[x+1,y],[x-1,y],[x,y+1],[x,y-1]]

        for node_neighbor,cor in pos.items():
            if (cor in neighbor_list and cor not in visited_nodes):
                G.add_edge(node_id,node_neighbor)
        
        visited_nodes.append([x,y])
    #print(visited_nodes)

    nx.draw(G,pos,with_labels=False, node_color='blue')
    nx.draw_networkx_labels(G,pos,labels=labels,font_size=8)
    #plt.show()

    ###################################################
    # ALGORITHM
    ###################################################
    #G is manhattan distance from the source node
    #H is the manhanttan distance from destination code
    #F is the cost = G + H
    #source = init_haulers[0]
    #destination = mission_seq_l[0]
    itemize = 0
    output= ""
    makespan = 0
  
    ######################### A Star Function ###########################
    def a_star_algorithm(final_mission): 
        path = []
        for first,second in zip(final_mission,final_mission[1:]):
            source = first
            destination = second
            open_pq = [] #Open Priority Queue
            visited = []
            parent = {}
            final = []
            reached = 0
            for node_id,[x,y] in pos.items():
                if(source == [x,y]): # Reached source
                    g = 0
                    h= abs(destination[0]-source[0]) + abs(destination[1]-source[1])
                    f = h
                    heapq.heappush(open_pq,(f,g,node_id,[x,y])) #Pushed source in open priority queue
                    count=0
                    while open_pq:
                        f_value,g_value,n_id,[a,b] = heapq.heappop(open_pq) #Pop the topmost   
                        #print(f"Current node {[a,b]}")
                        visited.append((f_value,n_id,[a,b])) #Moved popped node to the closed/visited list
                        neighbor_list= list(G.neighbors(n_id)) # Generated the 4 neighbours of the popped node
                        #print(f"neighbor_list for {[a,b]} is {neighbor_list}") 
                        for n in neighbor_list:
                            cor = G.nodes[n].get('pos')
                            #print(cor)
                            if(cor==destination):
                                reached = 1
                                parent[tuple(cor)] = tuple([a,b]) #Assign parent to each neighbor and put it in parent dict
                                #print(f"{cor} Reached Destination")
                                break
                            else:
                                #Compute g,h,f for the neighbor node
                                g = g_value + 1
                                h= abs(destination[0]-cor[0]) + abs(destination[1]-cor[1])
                                f = g+h
                                #print(f"g,h,f for node:{cor} is {g}, {h},{f}")
                                if any(t[2]==n for t in open_pq): # Checking if neighbor was already in open_q with lesser F value
                                    found = [t for t in open_pq if t[2]==n]
                                    if (found[0][0]>f): # The older one in visited has higher f value, push the current one
                                        #print("Modified Visited Node since the current values have smaller f value")
                                        open_pq.remove(found)
                                        heapq.heapify(open_pq)
                                        heapq.heappush(open_pq,(f,g,n,cor))  
                                        parent[tuple(cor)] = tuple([a,b]) #Assign parent to each neighbor and put it in parent dict  
                                elif any(t[1]==n for t in visited):
                                    found = [t for t in visited if t[1]==n]
                                    if (found[0][0]>f): # The older one in visited has higher f value, push the current one
                                        #print("Modified Visited Node since the current values have smaller f value")
                                        visited.remove(found) 
                                        heapq.heappush(open_pq,(f,g,n,cor))   
                                        parent[tuple(cor)] = tuple([a,b]) #Assign parent to each neighbor and put it in parent dict
                                else:#push the neighbor in open
                                    heapq.heappush(open_pq,(f,g,n,cor))
                                    parent[tuple(cor)] = tuple([a,b]) #Assign parent to each neighbor and put it in parent dict
                        count = count + 1
                        if reached : break
            final = find_path(parent,tuple(destination))
            final.reverse()
            final = final[:-1] # removes destination duplicates
            #makespan = makespan + len(final)
            #print(final)
            path = path + final
        path = [list(items) for items in path] + [destination]
        return path

    h=0
    compare_path=[]
    result_path = []
   
    ######################### Mission for all haulers #####################
    for f in final_mission:
        #print(f"mission in final mission for hauler {h} is {f}")
        path = a_star_algorithm(f)
        #print(f" For hauler {h}, the path is {path} is with len as {len(path)}")
        h+=1
        compare_path = compare_path + [path]

    ######################### Compare mission for collision ###############
    def compare_logic(compare_path):
        detected_check=0
        collision = False
        for index, elements in enumerate(zip_longest(*compare_path)):
            #print(index)
            #print(elements)
            if(sum(1 for x in elements if x is not None)==1): break
            for i in range(len(elements)):
                for j in range(i+1,len(elements)):
                    if(elements[i]==elements[j]):
                        detected_check=1
                        #print(f"Making detected_check as {detected_check}")
                        #print(f"Collision Detected for hauler: {i} and {j} for element {elements[i]} ay index {index}")
                        if(index < len(compare_path[i])): # Hauler is still active
                            value = compare_path[j][index-1]
                            if(value in elements) : ### A hauler has come if you stay stationary
                                new_value = compare_path[i][index-1]
                                #print(f"Changed hauler {i} to {new_value}")
                                compare_path[i].insert(index,new_value)
                            else:
                                compare_path[j].insert(index,value)
                                #print(f"Changed hauler {j} to {value}")
                            j+=1
                            collision=True
                        else: # Hauler has disappered, so hauler holds and other continues.
                            value = compare_path[i][index-1]
                            compare_path[i].insert(index,value)
                if(collision): 
                    collision=False
                    break # because you have taken care                 
        return detected_check, compare_path

    detected_check, result_path = compare_logic(compare_path)
    #print(f"detected_check after first run is {detected_check}")
    iter_check=0
    #while(detected_check==1):
    #    #print(f"ITERATION = {iter_check}")
    #    detected_check,result_path = compare_logic(result_path)
    #    iter_check += 1
    #    #if(iter_check==3):
    #    #    break
    lengths = [len(sublist)-1 for sublist in result_path]
    #print(lengths)
    makespan = max(lengths)
    #print(f"Makespan is {makespan}")
    for sublist in result_path:
        while(len(sublist)<=makespan):
            sublist.append(sublist[-1])

    sublist_lengths = [len(sub) for sub in sublist]
    #max_l = max(sublist_lengths)
    
        
    for first, pos_list in enumerate(zip(*result_path)): 
        output = output+ str(first)  
        for i in pos_list:
            i = str(i).replace(", ",",")    
            output = output + "," + i
        output = output + "\n"
        
    #print(output)
    #################################
    end = time.time()
    elapsed_time = int((end - start)*1000)
    
    # call the writing function here
    # example: 
    
    write_output(makespan, lengths, elapsed_time, output,output_file)



if __name__ == "__main__":
    main()
