# A_Star_Maze_solver
# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

# mazes were generated using https://www.dcode.fr/maze-generator


# Project Description
I took the basics for the A* algorithm from the sample code listed above. 
This was the final project for My LSUS CSC 667 AI course. I created this to demonstrate the fundamentals of one of the best graph traversal and path search algorithms. https://en.wikipedia.org/wiki/A*_search_algorithm

This python program takes takes in 6 .txt files for 3 mazes. Each maze has 2 text files. One (labeled maze_{num}.txt) is for displaying the unsolved maze in command prompt/terminal so the user can see what it looks like where the walls are made of zeros and the potential paths are made of ones. The read file for the algorithm where the maze walls are composed of "%" and the potential paths are empty spaces is labeled as maze{num}.txt. Both text files have a start (S) and goal (G) character for the algorithm to reference where to begin and end. The program reads the second text file then outputs the solved maze with the least cost path shown with a series of the character "p." The output unused path areas are filled with ones, and the walls are composed of zeros. 


# install python3 and python
download or clone entire folder
open folder in command line or IDE of choice such as VS Code
use the command 'python AStarMazeSolver.py'
results will be displayed. 