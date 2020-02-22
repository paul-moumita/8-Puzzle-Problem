# Author: Moumita Paul
# ENPM661 Planning for Autonomous Robots

import numpy as np 

#----------------------------------------------
# Functions for Node generation and exploration
#----------------------------------------------

# Find blank tile location 
def blank_tile(matrix):
	for i in range(0,len(matrix)):
		for j in range(0,len(matrix)):
			if(matrix[i,j]== 0):
				a = i 
				b = j
	return a, b

# Avoid repeated node
def avoid_repeated_node(x,y,val):
	repeated_node = 0
	for i in range(val):
		search_para = y[:,:,i]
		if (np.array_equal(x,search_para) == 1):
			repeated_node = 1
			break
	return repeated_node

# Action move right 
def action_move_right(parent_node):
	possible_right = 0
	matrix = parent_node.copy()
	blank_tile_i,blank_tile_j = blank_tile(matrix)
	
	if(blank_tile_j<len(matrix)-1):
		matrix[blank_tile_i,blank_tile_j] = matrix[blank_tile_i,blank_tile_j+1]
		matrix[blank_tile_i,blank_tile_j+1] = 0
		possible_right = 1
		
	return matrix, parent_node, possible_right

# Action move left
def action_move_left(parent_node):
	possible_left = 0
	matrix = parent_node.copy()
	blank_tile_i,blank_tile_j = blank_tile(matrix)
	if(blank_tile_j>0):
		matrix[blank_tile_i,blank_tile_j] = matrix[blank_tile_i,blank_tile_j-1]
		matrix[blank_tile_i,blank_tile_j-1] = 0
		possible_left = 1
		# print (matrix,'left')
	return matrix, parent_node, possible_left

# Action move up
def action_move_up(parent_node):
	possible_up = 0
	matrix = parent_node.copy()
	blank_tile_i,blank_tile_j = blank_tile(matrix)
	if(blank_tile_i>0):
		matrix[blank_tile_i,blank_tile_j] = matrix[blank_tile_i-1,blank_tile_j]
		matrix[blank_tile_i-1,blank_tile_j] = 0
		possible_up = 1
		
	return matrix, parent_node, possible_up

# Action move down 
def action_move_down(parent_node):
	possible_down = 0
	matrix = parent_node.copy()
	blank_tile_i,blank_tile_j = blank_tile(matrix)
	if(blank_tile_i<len(matrix)-1):
		matrix[blank_tile_i,blank_tile_j] = matrix[blank_tile_i+1,blank_tile_j]
		matrix[blank_tile_i+1,blank_tile_j] = 0
		possible_down = 1
	
	return matrix, parent_node, possible_down

# Back Tracking 
def back_tracking(found,node_info,nodes_list):
    path = [found]

    while(found!=0):
        found=int(node_info[0,1,found])
        path=np.append([path],[found])

    path_nodes = np.zeros((3,3,len(path)))
    for i in range(0,len(path)):
        path_nodes[:,:,i]=nodes_list[:,:,path[i]]

    return path_nodes, path	


def main():

	# User input
	initial_state = np.zeros((3,3))
	print("Enter initial matrix state row-wise:") 
	for row in range(3):
		print("Enter Row {}:".format(row))
		initial_state[row,:] = [int(input()) for x in range(3)] 
	print("Initial State entered")
	initial_state = np.mat(initial_state)
 
    # Default
	# initial_state = np.mat([[1, 0, 3], [4, 2, 5], [7, 8, 6]])
	
	goal_state = np.mat([[1,2,3],[4,5,6],[7,8,0]])
	iterations_no = 181441 #9!/2

	# Node initialization
	nodes_list = np.zeros((3,3,iterations_no))

	# Nodeinfo initialization
	node_info = np.zeros((1,3,iterations_no))

	
	nodes_list[:,:,0] = initial_state
	node_info[:,:,0] = [0,0,0]

	node_found = -1

	i = 1 # Child
	j = 0 # Parent
	goal_reached = 0

	if(np.array_equal(goal_state,initial_state)):
		print("Goal is same as initial state")
		goal_reached = 1
		node_found = 0

	while(i<iterations_no and goal_reached == 0):
		
		parent_node = nodes_list[:,:,j]
		nodes_list[:,:,i],parent_node,status_right = action_move_right(parent_node)
		repeated_node = avoid_repeated_node(nodes_list[:,:,i],nodes_list,i)
		if (status_right == 1 and repeated_node == 0):
			node_info[:,:,i] = [i,j,0]
			if(np.array_equal(nodes_list[:,:,i],goal_state)):
				goal_reached = 1
				node_found = i
			i = i+1
		
		nodes_list[:,:,i],parent_node,status_left = action_move_left(parent_node)
		repeated_node = avoid_repeated_node(nodes_list[:,:,i],nodes_list,i)
		if (status_left == 1 and repeated_node == 0):
			node_info[:,:,i]=[i,j,0]
			if(np.array_equal(nodes_list[:,:,i],goal_state)):
				goal_reached = 1
				node_found = i
			i = i+1

		nodes_list[:,:,i],parent_node,status_up = action_move_up(parent_node)
		repeated_node = avoid_repeated_node(nodes_list[:,:,i],nodes_list,i)
		if (status_up == 1 and repeated_node == 0):
			node_info[:,:,i]=[i,j,0]
			if(np.array_equal(nodes_list[:,:,i],goal_state)):
				goal_reached = 1
				node_found = i
			i = i+1


		nodes_list[:,:,i],parent_node,status_down = action_move_down(parent_node)
		repeated_node = avoid_repeated_node(nodes_list[:,:,i],nodes_list,i)
		if (status_down == 1 and repeated_node == 0):
			node_info[:,:,i]=[i,j,0]
			if(np.array_equal(nodes_list[:,:,i],goal_state)):
				goal_reached = 1
				node_found = i
			i = i+1
		
		j = j+1
	
	no_nodes = i

	#if goal found
	if(goal_reached == 1):

		print("Path Back tracking from goal to start")
		path_nodes,path = back_tracking(node_found,node_info,nodes_list)

		nodes_final = np.zeros((no_nodes,1,9))
		for i in range(no_nodes):
			nodes_final[i,:,:] = np.reshape(nodes_list[:,:,no_nodes-1-i], (1,9),order='F')
			
		with open('Nodes.txt', 'w') as file:
		    for node in nodes_final:
		        np.savetxt(file, node, fmt='%-2.0f')

		nodes_info_final = np.zeros((no_nodes,1,3))
		for i in range(no_nodes):
			nodes_info_final[i,:,:] = np.reshape(node_info[:,:,no_nodes-1-i], (1,3),order='F')
		
		with open('NodeInfo.txt', 'w') as file:
		    for node_info in nodes_info_final:
		        np.savetxt(file, node_info, fmt='%-2.0f')

		nodes_path_final = np.zeros((len(path),1,9))
		for i in range(len(path)):
			nodes_path_final[i,:,:] = np.reshape(path_nodes[:,:,len(path)-1-i], (1,9),order='F')

		with open('nodePath.txt', 'w') as file:

		    for node_path in nodes_path_final:
		        np.savetxt(file, node_path, fmt='%-2.0f')

	else:
		
		print("This initial configuration can not be solved")

if __name__ == "__main__":
	main()




