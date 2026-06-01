This repository mirrors the same project available at:
https://github.com/suaalim/LeafGrowthSimulationAndNonLinearOptimization


## Leaf Growth Simulation and Non-Linear Optimization
This project is an OpenGL based C++ application for simulating leaf growth that models natural phenomenon.


## Documentation
👉 To view the project documentation & results, go to (2025Research/Research report.pdf)


## Setup Instructions 📦 
1. Install Requirements

Install Visual Studio 2022

Download the Community Edition

During installation, select:

Desktop development with C++


2. Clone the Repository

git clone https://github.com/suaalim/2025Research.git


3. Open the Project in Visual Studio

Open Visual Studio 2022

Go to:

File → Open → Folder

Select the folder you cloned: 2025Research (This folder contains the CMakeLists.txt file, which is VERY important)


4. Configure the Project

Visual Studio should automatically detect and configure CMake.

If it does not: Project → Configure Cache


6. Build the Project

Go to:

Build → Build All

or

Build → Build 453-skeleton


6. Run the Program

Run the executable: 453-skeleton.exe


🎮 Controls

Keyboard Controls

G → Simulate growth

S → Subdivide branch

T → Subdivide branch and add a new branch

P → Save leaf structure data

	Saved output location: ...\working directory\2025Research\Code\out\build\x64-Debug\geometry_data
	
Mouse Controls

Left Click (on contour point) → Add a new branch

Left Click (on branching node) → Remove a node

Mouse Wheel Scroll → Zoom in/out

Camera Controls

Arrow Keys (Up/Down/Left/Right) → Move camera


⚠️ Required Code Modification

Before building, you must update the following lines in:

main.cpp

Update these lines:

Line 406

..workingdirectory/2025Research/Code/assets/shaders/test.vert

Line 407

..workingdirectory/2025Research/Code/assets/shaders/test.vert

Line 418

..workingdirectory/2025Research/plyFile/choose transform_matrices file from the folder

Make sure the working directory matches your local repository path.
