# AI-Project
In a 8-15-24 puzzle we have to find the solution using BFS, DFS, ID, and A*.
# Getting started
In order to work with puzzle we will use `numpy` library to work with matrices, and `time` library to calculate time spent.
## Prerequisites
First of all, we need to install the packages mentioned before.
# Generate Puzzle
In `all_puzzles.py` file we import `8_puzzle`, `15_puzzle`, and `24_puzzle` files to declare the puzzle. By using swich case we choose the size of puzzle, and another swich case to choose the algorithm to use. See following code.
```
print("Enter size of puzzle")
options = {1:'8',2:'15',3:'24'}
print("1: 8-puzzle \n2: 15-puzzle \n3: 24-puzzle")
ch=int(input("Your choice is: "))
def switch(ch):
    if ch==1:
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 22 as a comment
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
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 56 as a comment
        # print("Enter Input array")
        # test = []
        # for i in range(16):
        #     test.append(int(input("Element: ")))
        # test = np.array(test).reshape(4,4)
        test = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,0,14,15]).reshape(4,4)
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
        # if we want to declare a puzzle randomly please uncomment following commented line and make line 90 as a comment
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
```
### NOTE
We will continue explaining the `8_puzzle.py` file since all the files are similar except some changes to fit the size of the puzzle.
# Generate state
Each state will be represented by a node object which consist of some atribuites and methods. The attribuites are state is currently in, parent node, action "in which direction the move is going to be done", depth in the search tree, costs, and children nodes for the current node so we can move from one node to another. We have four possible moves, so we declare 4 functions to do that, so before any move we will check if the move is possible or not. If possible we will return the new state and the tile that being moved, and if not we will return false. Also we have functions to calculate the heuristics. See following calss `node`.
```
class Node():
    def __init__(self, state, parent, action, depth, step_cost, path_cost, heuristic_cost):
        self.state = state
        self.parent = parent  # parent node
        self.action = action  # move up, left, down, right
        self.depth = depth  # depth of the node in the tree
        self.step_cost = step_cost  # g(n), the cost to take the step
        self.path_cost = path_cost  # accumulated g(n), the cost to reach the current node
        self.heuristic_cost = heuristic_cost  # h(n), cost to reach goal state from the current node

        # children node
        self.move_up = None
        self.move_left = None
        self.move_down = None
        self.move_right = None
        
    # see if moving down is valid
    def try_move_down(self):
        # index of the empty tile
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 0:
            return False
        else:
            up_value = self.state[zero_index[0] - 1, zero_index[1]]  # value of the upper tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = up_value
            new_state[zero_index[0] - 1, zero_index[1]] = 0
            return new_state, up_value
    
     # see if moving right is valid
    def try_move_right(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 0:
            return False
        else:
            left_value = self.state[zero_index[0], zero_index[1] - 1]  # value of the left tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = left_value
            new_state[zero_index[0], zero_index[1] - 1] = 0
            return new_state, left_value
   

    # see if moving up is valid
    def try_move_up(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[0] == 2:
            return False
        else:
            lower_value = self.state[zero_index[0] + 1, zero_index[1]]  # value of the lower tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = lower_value
            new_state[zero_index[0] + 1, zero_index[1]] = 0
            return new_state, lower_value

    # see if moving left is valid
    def try_move_left(self):
        zero_index = [i[0] for i in np.where(self.state == 0)]
        if zero_index[1] == 2:
            return False
        else:
            right_value = self.state[zero_index[0], zero_index[1] + 1]  # value of the right tile
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = right_value
            new_state[zero_index[0], zero_index[1] + 1] = 0
            return new_state, right_value

    # return user specified heuristic cost
    def get_h_cost(self, new_state, goal_state, heuristic_function, path_cost, depth):
        if heuristic_function == 'num_misplaced':
            return self.h_misplaced_cost(new_state, goal_state)
        elif heuristic_function == 'manhattan':
            return self.h_manhattan_cost(new_state, goal_state)

    # return heuristic cost: number of misplaced tiles
    def h_misplaced_cost(self, new_state, goal_state):
        cost = np.sum(new_state != goal_state) - 1  # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0  # when all tiles matches

    # return heuristic cost: sum of Manhattan distance to reach the goal state
    def h_manhattan_cost(self, new_state, goal_state):
        current = new_state
        # digit and coordinates they are supposed to be
        goal_position_dic = {1: (0, 0), 2: (0, 1), 3: (0, 2), 4: (1, 0), 5: (1, 1), 6: (1, 2), 7: (2, 0), 8: (2, 1),
                             0: (2, 2)}
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i, j] != 0:
                    sum_manhattan += sum(abs(a - b) for a, b in zip((i, j), goal_position_dic[current[i, j]]))
        return sum_manhattan
```
And we have a function to print the path. Once the goal node is found, trace back to the root node and print out the path
```
def print_path(self):
        # create FILO stacks to place the trace
        state_trace = [self.state]
        action_trace = [self.action]
        depth_trace = [self.depth]
        step_cost_trace = [self.step_cost]
        path_cost_trace = [self.path_cost]
        heuristic_cost_trace = [self.heuristic_cost]

        # add node information as tracing back up the tree
        while self.parent:
            self = self.parent
            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.heuristic_cost)

        # print out the path
        step_counter = 0
        while state_trace:
            print('step', step_counter)
            print(state_trace.pop())
            print('action=', action_trace.pop(), ', depth=', str(depth_trace.pop()), \
                  ', step cost=', str(step_cost_trace.pop()), ', total_cost=', \
                  str(path_cost_trace.pop() + heuristic_cost_trace.pop()), '\n')
            step_counter += 1
        print("Total Number of steps: ",step_counter-1) # outside while loop, counter incremented by one
```
## BFS
Breadth-first search (BFS) is an algorithm for traversing or searching tree or graph data structures. It starts at the tree root and explores all of the neighbor nodes at the present depth prior to moving on to the nodes at the next depth level.
### Pseudocode
```
While queue is not empty
* Pop the first element from queue
* Record its depth and path cost
* Add it to visited set
* Compare the current state with goal state
    If equal 
        Print the solution path
    If not equal
        Try possible moves
        Add it to the end of the queue
```
### Python code
```
def breadth_first_search(self, goal_state):
        start = time.time()
        queue = [self]  # queue of found but unvisited nodes, FIFO
        queue_num_nodes_popped = 0  # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1  # max number of nodes in the queue, measuring space performance
        depth_queue = [0]  # queue of node depth
        path_cost_queue = [0]  # queue for path cost
        visited = set([])  # record visited states

        while queue:
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)

            current_node = queue.pop(0)  # select and remove the first node in the queue
            queue_num_nodes_popped += 1
            current_depth = depth_queue.pop(0)  # select and remove the depth for current node

            if current_depth <= 100:             # Condition for declaring the puzzle unsolvable.
                current_path_cost = path_cost_queue.pop(0)  # select and remove the path cost for reaching current node
                visited.add(tuple(current_node.state.reshape(1, 9)[0]))  # avoid repeated state, which is represented as a tuple
                # when the goal state is found, trace back to the root node and print out the path
                if np.array_equal(current_node.state, goal_state):
                    current_node.print_path()
                    print("number of Nodes in Queue: ", len(queue))
                    print("number of Nodes in Visited: ", len(visited))
                    print('Time performance:', str(queue_num_nodes_popped), 'nodes popped off the queue.')
                    print('Space performance:', str(queue_max_length), 'nodes in the queue at its max.')
                    print('Time spent: %0.2gs' % (time.time() - start))
                    return True

                else:
                    # see if moving upper tile down is a valid move
                    if current_node.try_move_down():
                        new_state, up_value = current_node.try_move_down()
                        # check if the resulting node is already visited
                        if tuple(new_state.reshape(1, 9)[0]) not in visited:
                            # create a new child node
                            current_node.move_down = Node(state=new_state, parent=current_node, action='down',
                                                      depth=current_depth + 1,
                                                      step_cost=up_value, path_cost=current_path_cost + up_value,
                                                      heuristic_cost=0)
                            queue.append(current_node.move_down)
                            depth_queue.append(current_depth + 1)
                            path_cost_queue.append(current_path_cost + up_value)

                    # see if moving left tile to the right is a valid move
                    if current_node.try_move_right():
                        new_state, left_value = current_node.try_move_right()
                        # check if the resulting node is already visited
                        if tuple(new_state.reshape(1, 9)[0]) not in visited:
                            # create a new child node
                            current_node.move_right = Node(state=new_state, parent=current_node, action='right',
                                                       depth=current_depth + 1,
                                                       step_cost=left_value, path_cost=current_path_cost + left_value,
                                                       heuristic_cost=0)
                            queue.append(current_node.move_right)
                            depth_queue.append(current_depth + 1)
                            path_cost_queue.append(current_path_cost + left_value)

                    # see if moving lower tile up is a valid move
                    if current_node.try_move_up():
                        new_state, lower_value = current_node.try_move_up()
                        # check if the resulting node is already visited
                        if tuple(new_state.reshape(1, 9)[0]) not in visited:
                            # create a new child node
                            current_node.move_up = Node(state=new_state, parent=current_node, action='up',
                                                    depth=current_depth + 1,
                                                    step_cost=lower_value, path_cost=current_path_cost + lower_value,
                                                    heuristic_cost=0)
                            queue.append(current_node.move_up)
                            depth_queue.append(current_depth + 1)
                            path_cost_queue.append(current_path_cost + lower_value)

                    # see if moving right tile to the left is a valid move
                    if current_node.try_move_left():
                        new_state, right_value = current_node.try_move_left()
                        # check if the resulting node is already visited
                        if tuple(new_state.reshape(1, 9)[0]) not in visited:
                            # create a new child node
                            current_node.move_left = Node(state=new_state, parent=current_node, action='left',
                                                      depth=current_depth + 1,
                                                      step_cost=right_value, path_cost=current_path_cost + right_value,
                                                      heuristic_cost=0)
                            queue.append(current_node.move_left)
                            depth_queue.append(current_depth + 1)
                            path_cost_queue.append(current_path_cost + right_value)
            else:
                print("Max depth exceeded")
                print('Time spent: %0.2gs' % (time.time() - start))
                return False
```
