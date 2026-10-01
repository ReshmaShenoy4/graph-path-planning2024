#!/usr/bin/env python3
import matplotlib.pyplot as plt
import time
import networkx as nx
import heapq
import math
import numpy as np

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

def wc_obstacles_num (SO_positions,start,end):
    x_min = min(start[0],end[0])
    x_max = max(start[0],end[0])
    y_min = min(start[1],end[1])
    y_max = max(start[1],end[1])

    count = sum( 1 for x,y in SO_positions if x_min<=x<=x_max and y_min<=y<=y_max)
    return count

def bc_obstacles_num (SO_positions,start,end):
    x_min = min(start[0],end[0])
    x_max = max(start[0],end[0])
    y_min = min(start[1],end[1])
    y_max = max(start[1],end[1])

    path = set()
    for x in range (x_min,x_max+1):
        path.add((x,start[1]))

    for y in range (y_min,y_max+1):
        path.add((end[0],y))
    count = sum( 1 for x,y in SO_positions if set([x,y]) in path)
    return count

        
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
    #config_file = "../../dataset1/Part2/Easy/10/2_config.txt"
    #mission_file = "../../dataset1/Part2/Easy/10/2_mission.txt"
    #output_file = "../../dataset1/Part2/Easy/10/2_output.txt"


    init_haulers, LP_positions, ULP_positions, SO_positions, CS_positions, battery_capacity, initial_energy = read_config(
        config_file)

    mission = read_mission(mission_file)

    start = time.time()
    #################################
    # place your main code here!
 
    ###################################################
    # CREATING GRAPH MODEL
    ###################################################
    ip = init_haulers
    #print(ip)
    mission_seq   = split_strings(mission[0])                         
    mission_seq_l = [LP_positions[s[1]-1] if (s[0]=='L') else ULP_positions[s[1]-1]  for s in mission_seq]
    final_mission = init_haulers + mission_seq_l
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
    count=0
    seq=[]
    
    ####################################################################
    def a_star_algorithm(source,destination):   
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
                    #print(f"energy for {[a,b]} is {energy}") 
                    for n in neighbor_list:
                        cor = G.nodes[n].get('pos')
                        #print(cor)
                        if(cor==destination):
                            reached = 1
                            parent[tuple(cor)] = tuple([a,b]) #Assign parent to each neighbor and put it in parent dict
                            #print(f"{cor} Reached Destination whose parent is {[a,b]}")
                            if(cor== CS_positions[0]): energy = battery_capacity
                            break
                        else:
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
                    if reached: break  
        final = find_path(parent,tuple(destination))
        final.reverse()
        final = final[:-1] # removes destination duplicates
        energy = 50*len(final)
        return energy,final

    ################ LOGIC TO DECIDE SOURCE and DESTINATION #############
    mission_list=final_mission
    energy = initial_energy
    ptr=0
    #print(f"Mission List is {mission_list}")
    debug = [mission_list[0]]
    #print(f"Debug list is {debug}")
    while (ptr < len(mission_list)-1):
        #print(f"ptr is {ptr}")
        if(ptr <= len(mission_list)-3):
            e_sd1,sd1   = a_star_algorithm(mission_list[ptr],mission_list[ptr+1])
            e_d1d2,d1d2 = a_star_algorithm(mission_list[ptr+1],mission_list[ptr+2])

            e_d1c,d1c   = a_star_algorithm(mission_list[ptr+1],CS_positions[0])
            e_d2c,d2c   = a_star_algorithm(mission_list[ptr+2],CS_positions[0])
            e_sc,sc     = a_star_algorithm(mission_list[ptr],CS_positions[0])

            e_cd1,cd1   = a_star_algorithm(CS_positions[0],mission_list[ptr+1])
            e_cd2,cd2   = a_star_algorithm(CS_positions[0],mission_list[ptr+2])

            if(energy>=(e_sd1 + e_d1d2 + e_d2c)):
                #print(f"I am here in stage1 for ptr = {ptr}")
                seq.append(sd1)
                seq.append(d1d2)
                debug = debug  + [mission_list[ptr+1]] + [mission_list[ptr+2]]
                energy = energy - e_sd1 - e_d1d2     
            elif (energy>=(e_sd1 + e_d1c)):
                #print(f"I am here in stage2 for ptr = {ptr}")
                seq.append(sd1)
                seq.append(d1c)
                #seq.append(cd1)
                #seq.append(d1d2)
                seq.append(cd2)
                debug = debug + [mission_list[ptr+1]] + [CS_positions[0]] + [mission_list[ptr+2]]
                #energy = battery_capacity - e_cd1 - e_d1d2
                energ= battery_capacity - e_cd2
            elif ((energy<e_sd1) or (energy < (e_sd1 + e_d1c))):
                #print(f"I am here in stage3 for ptr = {ptr}")
                seq.append(sc)
                seq.append(cd1)
                debug = debug + [CS_positions[0]] + [mission_list[ptr+1]]
                if((battery_capacity- e_cd1) > (e_d1d2 + e_d2c)): # go to d2 from d1
                    seq.append(d1d2)
                    energy = battery_capacity - e_cd1 - e_d1d2
                    debug = debug + [mission_list[ptr+2]]
                else:
                    seq.append(d1c)
                    seq.append(cd2)
                    energy = battery_capacity-e_cd2
                    debug = debug + [CS_positions[0]] + [mission_list[ptr+2]]
            ptr=ptr+2
        else:
            if(ptr==len(mission_list)-2):
                #print(f"I am here for len is 2 less for ptr = {ptr}")
                e_sd1,sd1   = a_star_algorithm(mission_list[ptr],mission_list[ptr+1])
                e_sc,sc     = a_star_algorithm(mission[ptr],CS_positions[0])
                e_d1c,d1c   = a_star_algorithm(mission_list[ptr+1],CS_positions[0])
                if(energy<e_sd1):
                    seq.append(sc)
                    seq.append(cd1)
                    debug = debug + CS_positions[0] + mission_list[ptr+1]
                else:
                    debug = debug + mission_list[ptr+1] + mission_list[ptr+2]
            else:
                 print(f"I am here for no valid case for ptr = {ptr}")
    
    #print(f"Debug list is {debug}")
    #print(f"Seq is {seq}")
    
    flat_list = np.concatenate(seq).tolist()
    flat_list = flat_list + [mission_list[-1]]
    flat_list = [item for item in flat_list for _ in ([item] * 5 if item == CS_positions[0] else [item])]
    #print(f"Flat list is {flat_list}")
    makespan = makespan + len(flat_list)
    for fl in flat_list:
        #print(fin)
        fl = str(list(fl)).replace(", ",",")
        output = output+str(itemize)+","+str(fl) + "\n"
        #print(output)
        itemize = itemize + 1
    #print(f"Output looks like = {output}")
    #################################
    end = time.time()
    elapsed_time = int((end - start)*1000)
    
    # call the writing function here
    # example: 
    write_output(makespan, [makespan], elapsed_time, output,output_file)



if __name__ == "__main__":
    main()
