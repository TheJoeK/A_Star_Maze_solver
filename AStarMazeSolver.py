# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

from __future__ import annotations
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional
import sys
T = TypeVar('T')

Location = TypeVar('Location')


class Graph(Protocol):
    def neighbors(self, id: Location) -> List[Location]: pass



import collections

def draw_tile(graph, id, style):
    r = " 1 "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " p "
    if 'start' in style and id == style['start']: r = " S "
    if 'goal' in style and id == style['goal']:   r = " G "
    if id in graph.walls: r = "000"
    return r


def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

GridLocation = Tuple[int, int]


class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []

    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls

    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]  # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse()  # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results


class WeightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass


class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}

    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

import heapq


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]
# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries

def reconstruct_path(came_from: Dict[Location, Location],
    start: Location, goal: Location) -> List[Location]:
    current: Location = goal
    path: List[Location] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)  # optional
    path.reverse()  # optional
    return path


def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def astar_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current: Location = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

def find_start(grid_list):
    for i in range(len(grid_list)):
        for j in range(len(grid_list[i])):
            if grid_list[i][j] == 'S': return (j,i)

def find_goal(grid_list):
    for i in range(len(grid_list)):
        for j in range(len(grid_list[i])):
            if grid_list[i][j] == 'G': return (j,i)

def find_walls(grid_list):
    walls = []
    for i in range(len(grid_list)):
        for j in range(len(grid_list[i])):
            if grid_list[i][j] == '%': walls.append((j,i))
    return walls

def find_heuristic_values(grid_list,goal):
    h_dict = {}
    for i in range(len(grid_list)):
       for j in range(len(grid_list[i])):
           if grid_list[i][j] != '%':
                h_dict[(j,i)] = heuristic(goal,(j,i))
    return h_dict

def showUnsolved(daMaze):
    for mazeLine in daMaze:
        print(mazeLine)

def mazeFileReader(mazeFile):
    derGrid = []
    with open(mazeFile, 'r') as file:
        for line in file.readlines():
            derGrid.append(line.strip())
    return derGrid    

def solveMazes(maze):
    
    
    grid_list = mazeFileReader(maze)
    start = find_start(grid_list)
    goal = find_goal(grid_list)
    grid = GridWithWeights(len(grid_list[1]), len(grid_list))
    grid.walls = find_walls(grid_list)
    grid.weights = find_heuristic_values(grid_list, goal)
    came_from, cost_so_far = astar_search(grid,start,goal)
    draw_grid(grid, path=reconstruct_path(came_from, start=start, goal=goal))
    print("Solution Cost: " + str(len(reconstruct_path(came_from, start=start, goal=goal))))
    print("Nodes Explored: " + str(len(came_from)))


def solveMazeFeeder():
    maze1unsolved = open("maze_1.txt")
    maze2unsolved = open("maze_2.txt")
    maze3unsolved = open("maze_3.txt")
    maze1 = "maze1.txt"
    maze2 = "maze2.txt"
    maze3 = "maze3.txt"
    print()
    print()
    print("Joseph Kelly")
    print("CSC 667 Final Project")
    print("Let us use the A* algorithm to solve some mazes!")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print()
    print("Maze 1")
    showUnsolved(maze1unsolved)
    print()
    print("Maze 1 solution")
    solveMazes(maze1)
    print()
    print()
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print()
    print("Maze 2")
    showUnsolved(maze2unsolved)
    print()
    print("Maze 2 solution")
    solveMazes(maze2)
    print()
    print()
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print()
    print("Maze 3")
    showUnsolved(maze3unsolved)
    print()
    print("Maze 3 solution")
    solveMazes(maze3)
    print()
    print()



solveMazeFeeder()