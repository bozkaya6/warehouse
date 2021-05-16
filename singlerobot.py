import gym
import gym_warehouse
import heapq
import numpy as np


class Node:
    def __init__(self, parent: (), position: ()):
        self.g = 0  # distance to start node
        self.h = 0  # distance to goal node
        self.f = 0  # total cost
        self.position = position
        self.parent = parent

    # to compare nodes with each other
    def __eq__(self, other):
        return self.position == other.position

    # to sort nodes with each other according to their total cost
    def __lt__(self, other):
        return self.f < other.f

    # for printing nodes
    def __repr__(self):
        return '({0},{1})'.format(self.position, self.f)


def heuristic_fnc(current_node, goal):
    return abs(current_node[0] - goal[0]) + abs(current_node[1] - goal[1])


def a_star_search(warehouse, start, goal):
    # lists for open and closed nodes
    open_nodes = []
    closed_nodes = []
    height = len(warehouse)
    width = len(warehouse[0])

    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_nodes.append(start_node)

    while len(open_nodes) > 0:
        open_nodes.sort()
        current_node = open_nodes.pop(0)
        closed_nodes.append(current_node)
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.append(current_node.position)
            return path[::-1]

        (x, y) = current_node.position
        neighbours = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        for n in neighbours:
            warehouse = np.array(warehouse)
            neighbour = Node(current_node, n)
            if 0 <= n[0] < height:
                if 0 <= n[1] < width:
                    warehouse_val = warehouse[n[0]][n[1]]
                    if warehouse_val == '*':
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbour in closed_nodes:
                continue
            neighbour.g = heuristic_fnc(neighbour.position, start_node.position)
            neighbour.h = heuristic_fnc(neighbour.position, goal_node.position)
            neighbour.f = neighbour.g + neighbour.h
            if insert_to_open_list(open_nodes, neighbour):
                open_nodes.append(neighbour)

    return None


def insert_to_open_list(open, neighbour):
    for node in open:
        if neighbour == node and neighbour.f >= node.f:
            return False
    return True


def run_agent():
    # create the Gym environment
    env = gym.make('singlerobot-warehouse-v0')
    x_row = 0
    start_node = (0, 0)
    goal_node = (0, 0)
    s = env.look()

    for row in s:
        current_column = 0
        for column in row:
            if column == 'a':
                start_node = (x_row, current_column)
            elif column == 'A':
                goal_node = (x_row, current_column)
            else:
                current_column += 1
        x_row += 1

    print("np.array(s):")
    print(np.array(s))
    path = a_star_search(s, start_node, goal_node)
    print("Start position of the robot: ",start_node)
    print("Goal position: ",goal_node)
    print("The found optimal path for the robot using A* search is:")
    print(str(path))

    done = False
    step = 0

    while 1:
        env.render()  # for printing the environment

        # sense

        current_row = path[step][0]
        current_column = path[step][1]
        target_row = path[step + 1][0]
        target_column = path[step + 1][1]
        if current_row - target_row < 0:
            ob, rew, done = env.step(env.ACTION_DOWN)
        elif current_row - target_row > 0:
            ob, rew, done = env.step(env.ACTION_UP)
        elif current_column - target_column > 0:
            ob, rew, done = env.step(env.ACTION_LEFT)
        elif current_column - target_column < 0:
            ob, rew, done = env.step(env.ACTION_RIGHT)

        if done:
            break
        step += 1

    env.close()


if __name__ == "__main__":
    run_agent()
