import numpy as np
from copy import deepcopy
# from lib.calculateFK import FK
# from lib.loadmap import loadmap
# from lib.detectCollision import detectCollision

from calculateFK import FK
from detectCollision import detectCollision
from loadmap import loadmap


# Defining a class to track each node configuration and its parent node:
class Node:
    def __init__(self, q, parent):
        self.q = q
        self.parent = parent


# Defining a method to return the distance between two configurations:
def dist(A, B):  # A & B are numpy arrays containing the joint angles of configurations A and B:
    return np.linalg.norm(A - B)


# Defining a function to return the closest parent node index and configuration to sampled node:

def closest_node_in_tree(tree, q_sampled):
    min_dist = 5000  # initialization with a large value
    tree_config_list = [node.q for node in tree]  # list containing tree node configurations
    for i in range(len(tree_config_list)):
        d_sampled = dist(q_sampled, tree_config_list[i])

        if d_sampled >= min_dist:
            continue
        min_dist = d_sampled
        parent_index = i
        parent_config = tree_config_list[i]
    return parent_index, parent_config


# Defining a method to check for self collision:

def self_collision_test(toe_joint_pos, inflated_head_joints):
    for i in range(4, 8):
        x = toe_joint_pos[i][0]
        y = toe_joint_pos[i][1]
        z = toe_joint_pos[i][2]
        for j in range(4):
            x_min = inflated_head_joints[j][0]
            y_min = inflated_head_joints[j][1]
            z_min = inflated_head_joints[j][2]
            x_max = inflated_head_joints[j][3]
            y_max = inflated_head_joints[j][4]
            z_max = inflated_head_joints[j][5]

            if (x < x_max) and (x > x_min) and (y < y_max) and (y > y_min) and (z < z_max) and (z > z_min):
                return True
    return False

# Defining an obstacle collision check method between any two configurations:

def isRobotCollided(head, toe, map):
    obstacle_inflation = 0.095
    joint_inflation = 0.09

    n_div = 20 #  number of line segments to represent the entire trajectory.

    head_joint_pos = FK().forward(head)[0]
    toe_joint_pos = FK().forward(toe)[0]

    inflated_obstacle = np.zeros(6)

    inflated_head_joints = np.zeros(48).reshape(8,6)
    inflated_head_joints[:, 0:3] = head_joint_pos - joint_inflation
    inflated_head_joints[:, 3:6] = head_joint_pos + joint_inflation

    # self-collision check:
    self_collision_list = np.array(self_collision_test(toe_joint_pos, inflated_head_joints))

    if np.any(self_collision_list):
        return True

    for obstacle in map[0]:
        inflated_obstacle[:3] = obstacle[:3] - obstacle_inflation
        inflated_obstacle[3:] = obstacle[3:] + obstacle_inflation

        collision_list = np.array(detectCollision(toe_joint_pos[0:6], toe_joint_pos[1:7], inflated_obstacle))
       
        if np.any(collision_list):
            return True

        for i in range(n_div):
            x_config = head + i * (toe - head) / n_div
            y_config = head + (i + 1) * (toe - head) / n_div

            x_config_jp = FK().forward(x_config)[0]
            y_config_jp = FK().forward(y_config)[0]

            # obstacle collision check between two configurations:
            collision_list = np.array(detectCollision(x_config_jp, y_config_jp, inflated_obstacle))

            if np.any(collision_list):
                return True
    return False



def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # joint limits:

    lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    # initialize path:
    path = []

    # initialize tree nodes from each end:

    starting_tree = []
    goal_tree = []

    starting_tree.append(Node(start, None))  # adding the start configuration as root node in starting tree
    goal_tree.append(Node(goal, None))  # adding the goal configuration as root node in goal tree

    iterations = 0  # initializing iteration variable.
    max_iterations = 2000  # maximum number of sampling.
    goal_flag = False

    while iterations <= max_iterations and not goal_flag:

        q_sampled = np.random.uniform(0.95 * lowerLim, 0.95 * upperLim)  # sampling a random node configuration
        # while respecting the joint limits.

        in_starting_tree = False
        in_goal_tree = False

        # finding parent node (index and configuration) to sampled node in starting tree:
        parent_start_index, parent_start_config = closest_node_in_tree(starting_tree, q_sampled)

        # Obstacle collision and self-collision test between the random sampled node and its parent node in starting
        # tree:
        if not isRobotCollided(parent_start_config, q_sampled, map):
            # adding q_sampled to the starting tree if it passes our tests.
            new_node = Node(q_sampled, parent_start_index)
            starting_tree.append(new_node)
            in_starting_tree = True

        # finding parent node (index and configuration) to sampled node in goal tree:
        parent_goal_index, parent_goal_config = closest_node_in_tree(goal_tree, q_sampled)

        # Obstacle collision and self-collision test between the random sampled node and its parent node in goal tree:
        if not isRobotCollided(parent_goal_config, q_sampled, map):
            # adding q_sampled to the goal tree if it passes our tests.
            new_node = Node(q_sampled, parent_goal_index)
            goal_tree.append(new_node)
            in_goal_tree = True

        # Tree connection test!
        goal_flag = in_starting_tree and in_goal_tree

        iterations = iterations + 1

    if goal_flag:  # i.e. if path exists:
        present_node = starting_tree[-1]
        path.append(present_node.q)  # adding common node in both starting and goal tree to path.

        # Keep adding intermediate nodes in starting tree by backtracking:

        while not np.array_equal(present_node.q, start):
            present_node = starting_tree[present_node.parent]
            path.append(present_node.q)

        # Reverse path to sort the nodes from starting node to present common node:
        path.reverse()

        cur_node = goal_tree[-1]
        present_node = goal_tree[cur_node.parent]
        path.append(present_node.q)

        # Do the same in goal path:
        while not np.array_equal(present_node.q, goal):
            present_node = goal_tree[present_node.parent]
            path.append(present_node.q)

    return path  # returns the final path!


if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0, -1, 0, -2, 0, 1.57, 0])
    goal = np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
