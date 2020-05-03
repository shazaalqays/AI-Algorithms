import numpy as np
import puzzle8, puzzle15, puzzle24

print("Enter size of puzzle")
options = {1:'8',2:'15',3:'24'}
print("1: 8-puzzle \n2: 15-puzzle \n3: 24-puzzle")
ch=int(input("Your choice is: "))
def switch(ch):
    if ch==1:
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 16 as a comment
        # print("Enter Input array")
        # test = []
        # for i in range(9):
        #     test.append(int(input("Element:")))
        # test = np.array(test).reshape(3,3)
        test = np.array([1,2,3,4,6,8,0,7,5]).reshape(3,3)
        initial_state = test
        # our goal state is:
        goal_state = np.array([1,2,3,4,5,6,7,8,0]).reshape(3,3)
        print("Initial state")
        print(initial_state)
        print("---------------------")
        print("GOAL")
        print(goal_state)
        print("---------------------")
        # create root node
        root_node = puzzle8.Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
        # choose algorithm to use
        print("Enter algorithm for solving")
        options = {1:'BFS',2:'DFS',3:'ID-DFS',4:'Astar with manhattan distance'}
        print("1: BFS\n2: DFS \n3: ID-DFS \n4: Astar with manhattan distance")
        ch2 = int(input("Your choice is: "))
        def switch(ch2):
            if ch2 == 1:
                root_node.breadth_first_search(goal_state)
            elif ch2 == 2:
                root_node.depth_first_search(goal_state)
            elif ch2 == 3:
                root_node.iterative_deepening_DFS(goal_state)
            elif ch2 == 4:
                root_node.a_star_search(goal_state,heuristic_function = 'manhattan')
        switch(ch2)
    elif ch==2:
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 50 as a comment
        # print("Enter Input array")
        # test = []
        # for i in range(16):
        #     test.append(int(input("Element: ")))
        # test = np.array(test).reshape(4,4)
        test = np.array([1,2,3,4,5,6,7,8,9,10,11,12,0,13,14,15]).reshape(4,4)
        initial_state = test
        # our goal state is:
        goal_state = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0]).reshape(4,4)
        print("Initial state")
        print(initial_state)
        print("---------------------")
        print("GOAL")
        print(goal_state)
        print("---------------------")
        # create root node
        root_node = puzzle15.Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
        # choose algorithm to use
        print("Enter algorithm for solving")
        options = {1:'BFS',2:'DFS',3:'ID-DFS',4:'Astar with manhattan distance'}
        print("1: BFS\n2: DFS \n3: ID-DFS \n4: Astar with manhattan distance")
        ch2 = int(input("Your choice is: "))
        def switch(ch2):
            if ch2 == 1:
                root_node.breadth_first_search(goal_state)
            elif ch2 == 2:
                root_node.depth_first_search(goal_state)
            elif ch2 == 3:
                root_node.iterative_deepening_DFS(goal_state)
            elif ch2 == 4:
                root_node.a_star_search(goal_state,heuristic_function = 'manhattan')
        switch(ch2)
    elif ch==3:
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 84 as a comment
        # print("Enter Input array")
        # test = []
        # for i in range(25):
        #     test.append(int(input("Element: ")))
        # test = np.array(test).reshape(5,5)
        test = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,0,21,22,23,24]).reshape(5,5)
        initial_state = test
        # our goal state is:
        goal_state = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,0]).reshape(5,5)
        print("Initial state")
        print(initial_state)
        print("---------------------")
        print("GOAL")
        print(goal_state)
        print("---------------------")
        # create root node
        root_node = puzzle24.Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
        # choose algorithm to use
        print("Enter algorithm for solving")
        options = {1:'BFS',2:'DFS',3:'ID-DFS',4:'Astar with manhattan distance'}
        print("1: BFS\n2: DFS \n3: ID-DFS \n4: Astar with manhattan distance")
        ch2 = int(input("Your choice is: "))
        def switch(ch2):
            if ch2 == 1:
                root_node.breadth_first_search(goal_state)
            elif ch2 == 2:
                root_node.depth_first_search(goal_state)
            elif ch2 == 3:
                root_node.iterative_deepening_DFS(goal_state)
            elif ch2 == 4:
                root_node.a_star_search(goal_state,heuristic_function = 'manhattan')
        switch(ch2)
switch(ch)
