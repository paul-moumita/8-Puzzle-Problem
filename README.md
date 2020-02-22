# 8-Puzzle-Problem
Python script is developed to solve 8 puzzle problem.

## Brute Force Search Algorithm
Brute force algorithm is used to reach the goal node from start node by generating all possible unique nodes. 


## Solution:
Found all the possible unique states of the 8-Puzzle problem starting from the given initial state. 
[[1, 0, 3], [4, 2, 5], [7, 8, 6]] 

From the initial state of the puzzle, used 4 different moves [Right, Left, Up , Down]  in all the directions to generate new nodes and checked the validity of the newly generated node. 

Used Back Tracking to find the path to solve the problem.

Found Goal State. [[1, 2, 3] , [4, 5, 6], [7, 8, 0]]

## Run Code

```
python3 Project1.py
```
To plot output:

```
python3 plot_path.py
```

## Requirements

Numpy 1.11
Python3
