#!/usr/bin/env python3

def breadth_first(map, start, goal, direction):
    queue = [start]
    explored = []
    parents = {tuple(start): None}
    goal_reached = False

    while queue:
        node = queue.pop(0)
        if node == goal:
            goal_reached = True
            break

        neighbours = get_neighbours(node, direction)
        for neighbour in neighbours:
            if tuple(neighbour) not in parents and map[neighbour[0]][neighbour[1]] == 1:
                queue.append(neighbour)
                explored.append(neighbour)
                parents[tuple(neighbour)] = tuple(node)

    if goal_reached:
        get_path(start, goal, parents)
    else:
        print("Stuck")


def get_path(start, goal, parents):
    path = []
    node = tuple(goal)
    while node is not None:
        path.append(list(node))
        node = parents[node]
    path.reverse()
    print("The path is: " + str(path))


def get_neighbours(node, direction):
    rows, cols = len(Map), len(Map[0])
    neighbours = []
    potential_neighbours = []

    if direction == 1:  # Clockwise (CW)
        potential_neighbours = [
            [node[0], node[1] + 1], [node[0] + 1, node[1] + 1],
            [node[0] + 1, node[1]], [node[0] + 1, node[1] - 1],
            [node[0], node[1] - 1], [node[0] - 1, node[1] - 1],
            [node[0] - 1, node[1]], [node[0] - 1, node[1] + 1]
        ]
    elif direction == -1:  # Counterclockwise (CCW)
        potential_neighbours = [
            [node[0], node[1] + 1], [node[0] - 1, node[1] + 1],
            [node[0] - 1, node[1]], [node[0] - 1, node[1] - 1],
            [node[0], node[1] - 1], [node[0] + 1, node[1] - 1],
            [node[0] + 1, node[1]], [node[0] + 1, node[1] + 1]
        ]

    for neighbour in potential_neighbours:
        if 0 <= neighbour[0] < rows and 0 <= neighbour[1] < cols:
            neighbours.append(neighbour)

    return neighbours


if __name__ == '__main__':
    Map = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 1, 1, 1, 0],
        [0, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0]
    ]
    breadth_first(Map, [1, 1], [5, 5], 1)  # map, start, goal, direction

