# -*- coding: utf-8 -*-
"""proj1_enpm661_hamza.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1LO5iwFwF5yWsVr6tKCLG18GpS0nz6x1h
"""

import numpy as np

#Converts List to 3x3 np.array
def List2Mat(data):
  mat = np.copy(data)
  mat = np.reshape(data,(3,3)) 
  return mat
#Converts Matrix to List
def Mat2List(listfile):
  array = np.copy(listfile)
  #Converts np array to 1d List
  lst=[]
  for i in array:
    for j in i:
      element= j.tolist()
      lst.append(element)
  return lst

#Returns key(0) position in np.array
def get_key_pos(Node):
  Node=np.array(Node)
  x= np.argwhere(Node == 0)
  i = x[0][0]
  j = x[0][1]
  return i,j

#Functions to move key/0 in the 3x3 Matrix
def move_up(CurrentNode):
  NewNode = List2Mat(CurrentNode)
  i,j=get_key_pos(NewNode)
  if i>0 :
    NewNode[i][j],NewNode[i-1][j]=NewNode[i-1][j],NewNode[i][j]
  return Mat2List(NewNode)

def move_down(CurrentNode):
  NewNode = List2Mat(CurrentNode)
  i,j=get_key_pos(NewNode)
  if i<2 :
    NewNode[i][j],NewNode[i+1][j]=NewNode[i+1][j],NewNode[i][j]
  return Mat2List(NewNode)

def move_left(CurrentNode):
  NewNode = List2Mat(CurrentNode)
  i,j=get_key_pos(NewNode)
  if j>0:
    NewNode[i][j],NewNode[i][j-1]=NewNode[i][j-1],NewNode[i][j]
  return Mat2List(NewNode)

def move_right(CurrentNode):
  NewNode = List2Mat(CurrentNode)
  i,j=get_key_pos(NewNode)
  if j<2:
    NewNode[i][j],NewNode[i][j+1]=NewNode[i][j+1],NewNode[i][j]
  return Mat2List(NewNode)

#Fucntion to get start and goal states
def get_input():
  #printing the values in a 3x3 matrix
  print("Enter entries ROW-WISE")
  print('\n Enter Start State:')
  start_st = []
  for i in range(3):
      element_val = []
      for j in range(3):
          element_val.append(int(input()))
      start_st.append(element_val)
  start_st=Mat2List(start_st)
  print('\n Enter Goal State:')
  goal_st = []
  for i in range(3):
      element_val = []
      for j in range(3):
          element_val.append(int(input()))
      goal_st.append(element_val)
  goal_st=Mat2List(goal_st)
  #print('start state: ',start_st)
  #print('\n goal state:',goal_st)

  return start_st,goal_st

def get_parent_node(nodeindex,bk_dict):
  #Returns List of Parent_Node_Index for Nodes_Index
  # Node_Index Parent_Index
  parent_index =[0]
  for i in range(1,len(nodeindex)):
    parent = bk_dict[str(i)]
    parent_index.append(parent[0])
  return parent_index

def generate_path(index,bk_dict):
  '''
  Returns List of Parent Nodes
  Parameters:
  Input:
  index = int
    index of goal state achieved (last node)
  bk_dict = dictionary
    dictionary containing all the child-parent node indices
  Output:
  p = list
    Returns list of parent nodes
  '''
  q=index
  p=[index]
  while(q != 0):
    q = bk_dict[str(q)]
    q=q[0]
    p.append(q)
  p.reverse()
  return p

def puzzle8_solve(start_state, goal_state):

  solved = 0
  nodes_index=[]
  parent_node=[]
  count = 1
  backtrack = {}

  nodes_index.append(start_state)
  parent_node.append([start_state])

  print(start_state)
  print('Start State')

  for layer in parent_node:

    if (solved == 1): break
    parent_node.append([])

    for sublayer in layer:

      if (solved == 1): break

      #parent
      current_node = sublayer
      index_sublayer = nodes_index.index(sublayer)

      #generating children
      child_nodes = []
      child_nodes.append(move_right(current_node))
      child_nodes.append(move_left(current_node))
      child_nodes.append(move_up(current_node))
      child_nodes.append(move_down(current_node))

      for i in range(0,len(child_nodes)):
        if (child_nodes[i] == goal_state):
          parent_node[count].append(child_nodes[i])
          nodes_index.append(child_nodes[i])
          index_child = nodes_index.index(child_nodes[i])
          backtrack.setdefault(str(index_child), []).append(index_sublayer)
          index_last = index_child
          print(child_nodes[i])
          print("Solved!")
          solved = 1

          break
        else:
          #adding
          if ((child_nodes[i] not in nodes_index) & (child_nodes[i] is not None)):
            parent_node[count].append(child_nodes[i])
            nodes_index.append(child_nodes[i])
            index_child = nodes_index.index(child_nodes[i])
            backtrack.setdefault(str(index_child), []).append(index_sublayer)
  
  bt_path = generate_path(index_last,backtrack)  
  node_parent_index = get_parent_node(nodes_index,backtrack)
  print('Total number of Nodes generated: ',len(nodes_index))
  print('BackTracked Path: ',bt_path)

  return nodes_index,node_parent_index,bt_path

#Functions to generate Nodes.txt and NodesInfo.txt
def gen_nodes_file(nodes_index_list):
  with open('Nodes.txt', 'w') as fp:
    for item in nodes_index_list:
        # write each item on a new line
        for val in item:
          fp.write("%s " % val)
        fp.write('\n')
    print('Nodes.txt Generated')

#Function to generate NodesInfo.txt
def gen_nodes_info_file(nodes_index_list,node_parent_list):
  p= node_parent_list
  with open('NodesInfo.txt', 'w') as fp:
    for i in range(0,len(nodes_index_list)):
        # write each item on a new line
        fp.write("%s %s" % (i,p[i]))
        for val in nodes_index_list[i]:
          fp.write("%s " % val)
        fp.write('\n')
  print('NodesInfo.txt Generated')

#Function to generate nodePath.txt
def gen_nodepath_file(nodesfile,pathfile):
  M_col=[]
  with open('nodePath.txt', 'w') as fp:
    for i in pathfile:
      i=int(i)
      M=List2Mat(nodesfile[i]).T
      M_col = Mat2List(M)
      for val in M_col:
       fp.write("%s " % val)
      fp.write('\n')
  print('nodePath.txt Generated')

#startstate = [1,4,7,2,5,8,3,0,6]
#goal = [1,6,7,2,0,5,4,3,8]

startstate = [1,2,3,4,0,5,7,8,6]
goal = [1,2,3,4,5,6,7,8,0]

s,g=get_input()

nodes,node_parent,path=puzzle8_solve(s,g)

#Generated Nodes.txt and NodesInfo.txt
gen_nodes_file(nodes)
gen_nodes_info_file(nodes,node_parent)
gen_nodepath_file(nodes,path)

#Prints the lists from generate_path
def printMatrix(nodeIndex,path):
  '''
  PRINT OUTPUT MATRICES 
  '''
  for i in path:
    i=int(i)
    print(List2Mat(nodeIndex[i]))
printMatrix(nodes,path)

