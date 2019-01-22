from enum import Enum
from queue import PriorityQueue
import numpy as np
import networkx as nx
from scipy.spatial import Voronoi
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Voronoi points
    points = []

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min,
                            0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min,
                            0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min,
                            0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min,
                            0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1
            # Center of obstacle
            points.append([north - north_min, east - east_min])

    print('Building Voronoi graph')
    voronoi_graph = Voronoi(points)
    print('Voronoi graph has {0} ridge vertices (edges)'.format(
        len(voronoi_graph.ridge_vertices)))
    # Check each edge from graph.ridge_vertices for collision
    edges = []
    for v in voronoi_graph.ridge_vertices:
        p1 = voronoi_graph.vertices[v[0]]
        p2 = voronoi_graph.vertices[v[1]]
        if not in_collision(grid, p1, p2):
            edges.append((
                (int(p1[0]), int(p1[1])),
                (int(p2[0]), int(p2[1]))))
    print('Found {0} collision free edges'.format(len(edges)))
    graph = nx.Graph()
    for e in edges:
        graph.add_edge(e[0], e[1], weight=np.linalg.norm(
            np.array(e[0]) - np.array(e[1])))
    return grid, int(north_min), int(east_min), graph


def in_collision(grid, p1, p2):
    """Returns true if a line from p1 to p2 collides with some obstacle.
    """
    cells = list(bresenham(int(p1[0]), int(p1[1]),
                           int(p2[0]), int(p2[1])))
    for cell in cells:
        if (cell[0] < 0 or cell[0] >= grid.shape[0] or
            cell[1] < 0 or cell[1] >= grid.shape[1] or
                grid[cell[0], cell[1]] == 1):
            return True
    return False


def trim_path(grid, path):
    """Removes unnecessary vertices from the given path.
    """
    while True:
        trimmed_path = []
        i = 0
        while i < len(path):
            trimmed_path.append(path[i])
            j = i + 2
            while j < len(path):
                if in_collision(grid, path[i], path[j]):
                    break
                j += 1
            i = j - 1
        if len(trimmed_path) < len(path):
            # Try to trim iteratively
            path = trimmed_path
        else:
            # Cannot trim anymore
            return path


def closest_point(graph, current_point):
    """Returns the closest point in the `graph` to the `current_point`.
    """
    graph_nodes = list(graph.nodes)
    return graph_nodes[np.argmin(np.linalg.norm(
        np.array(graph_nodes) - np.array(current_point), axis=1))]


def heuristic(n1, n2):
    return np.linalg.norm(np.array(n1) - np.array(n2))


def a_star(graph, heuristic, start, goal):
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    branch = {}
    found = False
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)
    path = []
    path_cost = 0
    if found:
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost
