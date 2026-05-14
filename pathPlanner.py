# Import any libraries required
import random


# This function returns the straight-line distance from the current cell
# to the goal cell. In A* this is used as the heuristic h(n).
def euclidean_distance(node, goal):
    return ((goal[0] - node[0]) ** 2 + (goal[1] - node[1]) ** 2) ** 0.5


# This function checks the open list and returns the node with the
# lowest estimated total cost f(n).
def get_min_f_score_node(open_nodes, f_score):
    best_node = None
    best_score = float('inf')

    for node in open_nodes:
        # Get the current f-score for this node
        current_score = f_score.get(node, float('inf'))
        # Check whether this node is better than the best one found so far
        if current_score < best_score:
            best_node = node
            best_score = current_score

    return best_node


# This function returns all valid neighbouring cells for the current node.
# Only four directions of movement are allowed.
def get_neighbors(node, grid, ROW, COL):
    neighbors = []
    x, y = node

    possible_moves = [
        (x + 1, y),     # move right
        (x - 1, y),     # move left
        (x, y + 1),     # move down
        (x, y - 1)      # move up
    ]

    for nx, ny in possible_moves:
        # A neighbour is valid if it stays inside the grid and is not an obstacle.
        if 0 <= nx < COL and 0 <= ny < ROW:
            if grid[nx][ny] == 1:
                neighbors.append((nx, ny))

    return neighbors


# Once the goal has been found, this function follows the stored parent
# links backwards from the goal to the start and rebuilds the final route.
def reconstruct_path(parent, current_node):
    path = [current_node]

    while parent[current_node] is not None:
        current_node = parent[current_node]
        path.append(current_node)

    path.reverse()
    return path


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    
    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])

    # A* uses three cost values:
    # g(n) is the path cost from the start node,
    # h(n) is the Euclidean distance to the goal,
    # and f(n) = g(n) + h(n).

    # Make a list to store the final path coordinates
    path = []

    # The open list stores nodes that can still be explored.
    open_nodes = [start]

    # The closed list stores nodes that have already been explored.
    closed_nodes = []

    # These dictionaries store the best route found so far.
    parent = {start: None}
    g_score = {start: 0}
    h_score = {start: euclidean_distance(start, end)}
    f_score = {start: g_score[start] + h_score[start]}

    # Print debug messages to the GUI message box
    display_message("Running A* search")
    display_message("Start location is " + str(start))

    # Continue searching until the goal is found or there are no nodes left.
    while open_nodes:
        # Choose the node with the lowest total estimated cost.
        current_node = get_min_f_score_node(open_nodes, f_score)

        # If the goal is reached, rebuild the path from end to start.
        if current_node == end:
            display_message("Goal reached")
            path = reconstruct_path(parent, current_node)
            break

        # Move the current node from the open list to the closed list.
        open_nodes.remove(current_node)
        closed_nodes.append(current_node)

        # Check each valid neighbour of the current node.
        for neighbor in get_neighbors(current_node, grid, ROW, COL):

            # Skip nodes that have already been explored.
            if neighbor in closed_nodes:
                continue

            # Each horizontal or vertical move has a cost of 1.
            tentative_g = g_score[current_node] + 1

            # Add new nodes to the open list.
            if neighbor not in open_nodes:
                open_nodes.append(neighbor)

            # If the new path is not better, keep the old one.
            elif tentative_g >= g_score.get(neighbor, float('inf')):
                continue

            # Update the stored path if this route is better.
            parent[neighbor] = current_node
            g_score[neighbor] = tentative_g
            h_score[neighbor] = euclidean_distance(neighbor, end)
            f_score[neighbor] = g_score[neighbor] + h_score[neighbor]

    if len(path) == 0:
        display_message("No path found", "WARN")

    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES
    return path

#end of file