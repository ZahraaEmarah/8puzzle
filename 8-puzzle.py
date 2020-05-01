import sys
import numpy as np
import heapq
import math
import time


class Node:
    # x and y axis
    x = [0, 1, 2, 0, 1, 2, 0, 1, 2]
    y = [2, 2, 2, 1, 1, 1, 0, 0, 0]

    def __init__(self, state, parent, cost=0, h=0, g=0):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.h = h
        self.g = g

    @staticmethod
    def move_left(index, new):
        # swap the one on the left with zero
        temp_state = new[index]
        new[index] = new[index - 1]  # swap with left , move one space
        new[index - 1] = temp_state
        return new

    @staticmethod
    def move_right(index, new):
        # swap the one on the left with zero
        temp_state = new[index]
        new[index] = new[index + 1]  # swap with right , move one space
        new[index + 1] = temp_state
        return new

    @staticmethod
    def move_up(index, new):
        # swap the one up with zero
        temp_state = new[index]
        new[index] = new[index - 3]  # swap with above , move 3 spaces
        new[index - 3] = temp_state
        return new

    @staticmethod
    def move_down(index, new):
        # swap the one down with zero
        temp_state = new[index]
        new[index] = new[index + 3]  # swap with below , move 3 spaces
        new[index + 3] = temp_state
        return new

    def __lt__(self, other):  # sort the heap on the cost of states in A* algorithm
        return self.cost < other.cost

    def __eq__(self, other):  # compare nodes on their states only
        if not isinstance(other, Node):
            return NotImplemented
        return self.state == other.state


# algorithm for DFS
def dfs(initial_state, goal_state):
    steps = []  # list to trace back the steps
    frontier = []  # initialize the stack
    explored = set()  # the explored is an empty set
    fexplored = set()  # used to combine the frontier and explored
    int_state = Node(initial_state, None)  # intial state of the board
    nodes_expanded = 0
    depth = 0
    frontier.append(int_state)  # append the intial state to the frontier
    while frontier:  # loop till the stack is empty
        popped_state = frontier.pop()  # pop the top board found in the frontier
        explored.add(tuple(popped_state.state))  # add it to the explored
        fexplored.add(tuple(popped_state.state))
        printboard(popped_state.state)  # print it
        if np.array_equal(goal_state, popped_state.state):
            print("SUCCESS")
            print("******************")
            print("Total nodes expanded are =", nodes_expanded)
            print("Path to goal:")
            parent = popped_state.parent  # get the current state's parent
            steps.append(popped_state)
            while parent:  # get the parent of each node until we reach the root
                steps.append(parent)
                parent = parent.parent
                depth = depth + 1  # to calculate the depth
            while steps:  # print the trace of steps from the start
                x = steps.pop()
                printboard(x.state)
            print("Depth = ", depth)
            print("Total nodes expanded =", nodes_expanded)
            return "SUCCESS"
        nodes_expanded = nodes_expanded + 1
        neighbors = children(popped_state)  # get the children of this node
        for neighbor in neighbors:  # loop on this node to add them to the frontier
            if tuple(neighbor.state) not in fexplored:  # if not in frontier or explored
                frontier.append(neighbor)
                fexplored.add(tuple(neighbor.state))  # this is used to add both explored and frontier
    return "FAILURE"


def bfs(initial_state, goal_test):
    int_state = Node(initial_state, None)
    frontier_queue = [int_state]  # initialize queue
    explored = []  # list of explored nodes
    steps = []  # list to trace back the steps
    depth = 0
    nodes_expanded = 0
    print("Expanded states: ")
    while frontier_queue:
        state = frontier_queue.pop(0)
        nodes_expanded = nodes_expanded + 1
        explored.append(tuple(state.state))
        printboard(state.state)
        if np.array_equal(goal_test, state.state):  # goal found
            print("SUCCESS")
            print("******************")
            print("Path to goal: ")
            parent = state.parent  # get the current state's parent
            steps.append(state)
            while parent:  # get the parent of each node until we reach the root
                steps.append(parent)
                parent = parent.parent
                depth = depth + 1
            while steps:  # print the trace of steps from the start
                x = steps.pop()
                printboard(x.state)
            print("Depth = ", depth)
            print("Total nodes expanded =", nodes_expanded)
            return "SUCCESS"
        neighbors = children(state)
        for neighbor in neighbors:
            if tuple(neighbor.state) not in explored:
                explored.append(tuple(neighbor.state))
                frontier_queue.append(neighbor)
    return "FAILURE"


def a_star(initial_state, goal_state, heuristic):
    print("\n**** " + heuristic.upper() + " DISTANCE ****")

    # initialize root
    g = 0
    depth = 0
    cost_of_path = 0
    int_state = Node(initial_state, None)
    goal = Node(goal_state, None)
    expanded = 0

    if heuristic == "manhattan":
        int_state.h = manhattan_distance(int_state, goal)  # get h(n) of root
        int_state.g = g  # initialize g(n)
        int_state.cost = int_state.g + int_state.h  # calculate cost of root
    elif heuristic == "euclidean":
        int_state.h = euclidean_distance(int_state, goal)
        int_state.g = g
        int_state.cost = int_state.g + int_state.h

    frontier = [int_state]  # initialize a Min heap
    explored = []  # list of explored states
    steps = []  # list of states of the final path (steps to solution)

    while frontier:  # while the heap is not empty
        state = heapq.heappop(frontier)  # pop the state with the minimum cost
        heapq.heapify(frontier)
        children_of_poppedstate = children(state)  # get all possible moves from the  popped state
        explored.append(state)  # add it to the list of explored states

        if np.array_equal(goal_state, state.state):  # if the popped state is the goal state
            print("SUCCESS")
            print("******************")
            print("Steps of solution:")
            parent = state.parent  # get the current state's parent
            steps.append(state)
            while parent:  # get the parent of each node until we reach the root
                steps.append(parent)
                parent = parent.parent
            while steps:  # print the trace of steps from the start
                z = steps.pop()
                depth = z.g
                print("g = {}, h = {}".format(z.g, round(z.h, 2)))
                print("cost is {}".format(round(z.cost, 2)))
                cost_of_path = cost_of_path + z.cost
                printboard(z.state)
            print("Depth = {}".format(depth))
            print("cost of path = {}".format(cost_of_path))
            print("Expanded nodes of {} distance = {}".format(heuristic, expanded))
            return "SUCCESS"

        expanded = expanded + 1
        # if we reach this point it means the popped state isn't the goal state
        # we loop on all its possible children
        for x in children_of_poppedstate:

            if heuristic == "manhattan":
                x.h = manhattan_distance(x, goal)  # calculate manhattan heuristic
                x.g = x.parent.g + 1  # set g(n)
                x.cost = x.g + x.h  # calculate cost
            elif heuristic == "euclidean":
                x.h = euclidean_distance(x, goal)  # calculate euclidean heuristic
                x.g = x.parent.g + 1
                x.cost = x.g + x.h

            # if child is a new state
            if x not in explored and x not in frontier:
                heapq.heappush(frontier, x)  # add it to the heap of unexplored states
                heapq.heapify(frontier)

            # if child has been explored before
            if x in explored:
                index = get_index(explored, x)
                # compare its cost with its version in the explored list
                if explored[index].cost > x.cost:  # if it has a better cost than its version
                    explored.remove(x)  # remove the version
                    heapq.heappush(frontier, x)  # insert the new state with the better cost
                    heapq.heapify(frontier)

            # if child in the heap of states to be explored
            if x in frontier:
                index = get_index(frontier, x)
                # compare its cost with its version in the heap
                if frontier[index].cost > x.cost:  # if it has a better cost than its version
                    frontier[index].cost = -100  # decrease the key of its version
                    heapq.heappop(frontier)  # remove it from the heap
                    heapq.heapify(frontier)
                    heapq.heappush(frontier, x)  # insert the new state with the better cost
                    heapq.heapify(frontier)

    return "FAILURE"


# function to get the index of a state in a list
def get_index(heap, node):
    for i in range(len(heap) + 1):
        if node.state == heap[i].state:
            return i


def children(node):
    neighbors = list()
    # there are four possibilities , either move left , move right , move up or  move down
    index = node.state.index(0)
    if index < 6:  # move down
        neighbors.append(Node(move(node.state, 1, node), node))
    if index not in {2, 5, 8}:  # move right
        neighbors.append(Node(move(node.state, 2, node), node))
    if index > 2:  # move up
        neighbors.append(Node(move(node.state, 3, node), node))
    if index not in {0, 3, 6}:  # move left
        neighbors.append(Node(move(node.state, 4, node), node))

    return neighbors


def move(state, the_move, node):
    new_state = state[:]  # take a copy of the state

    index = new_state.index(0)
    if the_move == 1:  # Down
        return node.move_down(index, new_state)
    if the_move == 2:  # Right
        return node.move_right(index, new_state)
    if the_move == 3:  # Up
        return node.move_up(index, new_state)
    if the_move == 4:  # Left
        return node.move_left(index, new_state)


def manhattan_distance(current_state, goal_state):
    total_h = 0
    c_state = list(current_state.state)  # current state
    g_state = list(goal_state.state)  # goal state

    for i in current_state.state:  # loop on all numbers in a state
        if i == 0:  # zero isn't included in the calculation of manhattan heuristic
            continue
        index_current = c_state.index(i)  # get the number's index in the current state
        index_goal = g_state.index(i)  # get the number's index in the goal state
        # calculate hn(n)
        h = abs(current_state.x[index_current] - goal_state.x[index_goal]) \
            + abs(current_state.y[index_current] - goal_state.y[index_goal])
        total_h = total_h + h  # h(n) total
    return total_h


def euclidean_distance(current_state, goal_state):
    total_h = 0
    c_state = list(current_state.state)  # current state
    g_state = list(goal_state.state)  # goal state

    for i in current_state.state:  # loop on all numbers in a state
        if i == 0:  # zero isn't included in the calculation of the heuristic
            continue
        index_current = c_state.index(i)  # get the number's index in the current state
        index_goal = g_state.index(i)  # get the number's index in the goal state
        # calculate hn(n)
        h = math.sqrt(pow((current_state.x[index_current] - goal_state.x[index_goal]), 2) + pow(
            (current_state.y[index_current] - goal_state.y[index_goal]), 2))
        total_h = total_h + h  # h(n) total
    return total_h


# print the board
def printboard(state):
    print("  *  *  * ", "\n", state[0:3], "\n", state[3:6], "\n", state[6:], "\n", " *  *  *")


def main():
    goal_state = 0, 1, 2, 3, 4, 5, 6, 7, 8
    initial_state = eval(sys.argv[1])
    algorithm = sys.argv[2]
    if len(initial_state) != 9:
        print("Error: Wrong number of entries")
        return
    if algorithm == 'dfs':
        start_time = time.time()
        dfs(list(initial_state), goal_state)
        end_time = time.time()
        print("Running time = {}".format(end_time - start_time))
    elif algorithm == 'bfs':
        start_time = time.time()
        bfs(list(initial_state), goal_state)
        end_time = time.time()
        print("Running time = {}".format(end_time - start_time))
    elif algorithm == 'a*':
        start_time = time.time()
        a_star(list(initial_state), goal_state, "manhattan")
        end_time = time.time()
        print("Running time = {}".format(end_time - start_time))
        start_time = time.time()
        a_star(list(initial_state), goal_state, "euclidean")
        end_time = time.time()
        print("Running time = {}".format(end_time - start_time))


if __name__ == "__main__":
    main()
