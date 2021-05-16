import gym
import gym_warehouse
import heapq
import numpy as np


class Node:
    def __init__(self, parent: (), position: ()):
        self.g = 0  # distance to start node
        self.h = 0  # distance to goal node
        self.f = 0  # total cost
        self.time = 0
        self.position = position
        self.parent = parent

    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.position[0], self.position[1], self.time))

    # Print node
    def __repr__(self):
        return '({0},{1})'.format(self.position, self.f)


def heuristic_fnc(current_node, goal):
    return abs(current_node[0] - goal[0]) + abs(current_node[1] - goal[1])


def insert_to_open_list(open, neighbour):
    for node in open:
        if neighbour == node and neighbour.f >= node.f:
            return False
    return True


def a_star_search(warehouse, start, goal, all_paths):
    open_nodes = []
    closed_nodes = []
    wait_list = {}

    height = len(warehouse)
    width = len(warehouse[0])
    wait_bool = 1

    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_nodes.append(start_node)

    while len(open_nodes) > 0:
        open_nodes.sort()
        current_node = open_nodes.pop(0)
        closed_nodes.append(current_node)
        if current_node not in wait_list:
            wait_list[current_node] = wait_bool
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append((current_node.position[0], current_node.position[1], current_node.time))
                current_node = current_node.parent
            path.append((current_node.position[0], current_node.position[1], current_node.time))

            return path[::-1]

        (x, y) = current_node.position
        neighbours = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        for n in neighbours:
            warehouse = np.array(warehouse)
            neighbour = Node(current_node, n)
            if 0 <= n[0] < height:
                if 0 <= n[1] < width:
                    warehouse_val = warehouse[n[0]][n[1]]
                    # if there is an obstacle
                    if warehouse_val == '*':
                        continue
                    else:
                        # if there is another robot in the way
                        if warehouse[n[0]][n[1]] != '.':
                            if not str(warehouse[n[0]][n[1]]).isupper():
                                continue
                else:
                    # if it is out of bound column-wise
                    continue
            else:
                # if it is out of bounds row-wise
                continue
                # if it is already visited
            if neighbour in closed_nodes:
                continue

            if insert_to_open_list(open_nodes, neighbour):
                # the first path for the first robot
                if len(all_paths) == 0:

                    neighbour.g = heuristic_fnc(neighbour.position, start_node.position)
                    neighbour.h = heuristic_fnc(neighbour.position, goal_node.position)
                    neighbour.f = neighbour.g + neighbour.h
                    neighbour.time = current_node.time + 1
                    open_nodes.append(neighbour)
                elif len(all_paths) > 0:
                    # if the desired location will be empty at the desired time
                    if (neighbour.position[0], neighbour.position[1], current_node.time + 1) not in all_paths:
                        # To avoid the condition in which two robots will replace each other's position the next step

                        # if the current position will be filled by another robot in the next time
                        if (current_node.position[0], current_node.position[1], current_node.time + 1) in all_paths:
                            # but not by a robot whose location is the desired location of the current robot
                            if (neighbour.position[0], neighbour.position[1], current_node.time) not in all_paths:
                                neighbour.g = heuristic_fnc(neighbour.position, start_node.position)
                                neighbour.h = heuristic_fnc(neighbour.position, goal_node.position)
                                neighbour.f = neighbour.g + neighbour.h
                                neighbour.time = current_node.time + 1
                                open_nodes.append(neighbour)
                        else:

                            if wait_list[current_node] < 100:
                                wait_list[current_node] += wait_bool
                                neighbour.g = heuristic_fnc(neighbour.position, start_node.position)
                                neighbour.h = heuristic_fnc(neighbour.position, goal_node.position)
                                neighbour.f = neighbour.g + neighbour.h
                                neighbour.time = current_node.time + 1
                                open_nodes.append(neighbour)

    return None


def run_agent():
    # create the Gym environment
    env = gym.make('multirobot-warehouse-v0')
    robots_paths = dict()
    all_paths = []
    x = 0
    start_nodes = {}
    goal_nodes = {}
    s = env.look()
    for row in s:
        y = 0
        for column in row:
            if column != '.' and column != '*':
                if str(column).islower():
                    start_nodes[column] = (x, y)
                else:
                    goal_nodes[column] = (x, y)
            else:
                y += 1
        x += 1
    sorted_start_nodes = sorted(start_nodes)
    sorted_goal_nodes = sorted(goal_nodes)
    print("Starting positions of robots:")
    for i in sorted_start_nodes:
        print("Robot ", str(i), " :", str(start_nodes[i]))

    print("Goal positions of robots:")
    for i in sorted_start_nodes:
        goal_node = str(i).upper()
        print(str(i).upper(), " :", str(goal_nodes[goal_node]))

    # the algorithm for multirobot given in hw.pdf
    for node in sorted_start_nodes:
        goal_node = str(node).upper()
        path = a_star_search(s, start_nodes[node], goal_nodes[goal_node], all_paths)
        if not path:
            return False
        else:
            for step in path:
                all_paths.append(step)
            robots_paths[node] = path
        print(str("The path of robot ") + str(node) + " is :" + str(path))

    done = False
    step = 0
    while 1:
        env.render()  # you can used this for printing the environment

        movements = []
        # start nodes={a:(x,y), b: (x,y)...}
        # robot paths = { a: (path of a((x,y),(x,y),(x,y)), b:(path of b)..}
        for robot in sorted_start_nodes:
            if len(robots_paths[robot]) - 1 > step:

                current_row = robots_paths[robot][step][0]
                current_column = robots_paths[robot][step][1]
                target_row = robots_paths[robot][step + 1][0]
                target_column = robots_paths[robot][step + 1][1]
                if current_row - target_row < 0:
                    movements.append(env.ACTION_DOWN)

                elif current_row - target_row > 0:
                    movements.append(env.ACTION_UP)

                elif current_column - target_column > 0:
                    movements.append(env.ACTION_LEFT)

                elif current_column - target_column < 0:
                    movements.append(env.ACTION_RIGHT)

                elif current_row == target_row and current_column == target_column:
                    movements.append(env.ACTION_WAIT)


            else:
                movements.append(env.ACTION_WAIT)

        print(movements)
        ob, rew, done = env.step(movements)

        if done:
            break
        step += 1

    env.close()


if __name__ == "__main__":
    run_agent()
