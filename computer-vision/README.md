# Micromouse Computer Vision Section
---
## Description
This directory contains the full implementation of the Computer Vision that enables the robot to navigate a maze

## Installatiion
- Install Python 3.10+ and Pip
- Clone this project
- Open a terminal and run
```bash
python -m venv .venv
source .venv/bin/activate
pip intall -r requirements.txt
```


### Full Maze Navigation
This part uses the file `cell_maze_solver.ipynb`. Each section contains it header that describes the functionally of the implementation
The framework can be divided into 4 main categories

1. Image Preprocessing
  1. The code will attempts to take an image capture from an external camera. Or one could just load a predefined image from the cached images
  2. The code will then Crop the image into a square and apply morphological operations to create a mask for future processing
2. Meshgrid Creation
  1. The code will try to find the anchor of the maze that is attached on the ground from the image
  2. The code will then create a meshgrid of size 9x9 for maze creation
3. Maze Transform
  1. In this part, the code will attemp to connect the grids and create a full maze by considering whether the connection is valid or not
  2. A graph structure will be created with marking to indicate cell to cell connectivity
  3. A new canvas will be created, which reveals the connection and the whole maze structure in 1:1 scale
4. Path Generation
  The code will
  1. Use Breath First Search (BFS) to create a solution from start to goal
  2. Use Asymptotically-Optimal RRT (RRT*) to create a solution from start to goal
  3. Two groups of coordinate vector are generated in a way that direct Copy-and-Paste to the robot is desired.


  
### Continuous Maze Navigation
This part uses the file `pillar_maze_solver.ipynb`. Each section contains it header that describes the functionally of the implementation
The framework can be divided into 5 main categories

1. Image Preprocessing
  1. The code will attempts to take an image capture from an external camera. Or one could just load a predefined image from the cached images
  2. The code will then Crop the image into a square and apply morphological operations to create a mask for future processing
2. Meshgrid Creation
  1. The code will try to find the anchor of the maze that is attached on the ground from the image
  2. The code will then create a meshgrid of size 9x9 for maze creation
3. Hollow Pillar Finding
  1. This part is hardcoded by specifying the offset of the hollowed out maze
  2. The code will then search through the region of interest and label the pillars out
4. Maze Transform
  1. In this part, the code will attemp to connect the grids and create a full maze by considering whether the connection is valid or not
  2. A graph structure will be created with marking to indicate cell to cell connectivity
  3. A new canvas will be created, which reveals the connection and the whole maze structure in 1:1 scale
5. Path Generation
  The code will
  1. Use Breath First Search (BFS) to create a solution from start to goal
  2. Use Asymptotically-Optimal RRT (RRT*) to create a solution from start to goal
  3. Two groups of coordinate vector are generated in a way that direct Copy-and-Paste to the robot is desired.
