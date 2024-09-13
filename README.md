# Visibility-based Marching (VBM)
Visibility-based marching method is an exact wave propagation technique that has an O(n) compute and space complexity. Solves several shortcomings of SOTA marching methods and inherently produces globally optimal paths to all grid points (one-to-all path planning). <br>
Compared to SOTA marching methods:
-Uses exact distance function (Minkowski or any other) instead of approximations obtained with discritized Eikonal equation
- Inherently produces globally optimal paths instead of requiring gradient descent per path query
- Faster and more efficient
- More accurate
- Verified with Anya in obstacle-rich environments
- Ability to march from multiple starting positions without wave collisions

([Preprint](https://ieeexplore.ieee.org/document/10679927)). <br>
Code developed for C++ 20. <br>
The code has four main methods: <br>
- visibilityBasedSolver: performs VBM <br>
- vStarSearch: $V^*$ greedy version of VBM <br>
- aStarSearch: for comparison purposes with vStarSearch $V^*$ <br>
- computeDistanceFunction: computes Euclidean Distance Function for the map <br>

Set which methods to use in config/settings.config and what results to save. Uses SFML to make plots that are saved in output/ directory. You can either load a map or generate one randomly (can also fix the randomness seed number for repeatability). 

## Sample marching in a couple of environments: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/euclidean.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/AcrossTheCape.png) <br>
Huge 3201x3201 maze VBM starting from initial wave source (1605, 1605): <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/test_huge_maze.png) <br>
Sample marching image result generated using SFML in C++: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/visibilityBasedSolver.png) <br>

## Sample marching of different distance functions: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/chessboard.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/cityblock.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/cubic.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/quasiEuclidean.png) <br>

## Sample marching starting from multiple sources: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/multiple_sources.png) <br>

## Sample ESDF computation using VBM: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/ESDF.png) <br>

## Sample ESDF computation using VBM generated using SFML in C++: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/distanceFunction.png) <br>

VBM relies on an underlying visibility algorithm that computes visibility as shown below (introduced in https://github.com/IbrahimSquared/visibility-heuristic-path-planner). <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/visibility_polygon.jpg) <br>

## Sample vStar and aStar image/path results generated using SFML in C++: <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/vstar.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-based-marching/blob/main/images/examples/astar.png) <br>

Please read how to compile/build the C++ files first below. <br>

Inside config folder there is a settings.config file which the code parses. The code can run in two modes, either generating a random/fixed seed environment with specific dimensions & number/size of rectangular obstacles (mode=1) or reading an image (mode=2). <br>

It additionally implements an AStar and a VStar (greedy VBM) solver for comparison/completion purposes.
The repo includes expfig and MSFM with their licenses inside for usage/comparison. <br>

# To build or compile using cmake in Linux
Required: <br>
cmake and g++: <br>
``` sudo apt install cmake ``` <br>
``` sudo apt install g++ ``` <br>
libsfml-dev: <br>
``` sudo apt install libsfml-dev ``` <br>
Set compiler path if needed, make sure SFML libraries are installed, then: <br>
``` mkdir -p build && cd build ``` <br>
``` cmake .. ``` <br>
``` make ```

# Important: Standard for inputting/reading images 
Mode=2 reads an image map, the path of which is specified in imagePath in settings.config, for example lab_image_edited.png (893x646) in the folder images. <br>
Bottom left corner is the origin. <br>
This is relevant to selecting initialFrontline (starting point/s) and target_x and target_y (end/target point). This standard can ofcourse be changed.
Note that initialFrontline in settings.config has the form {x1, y1, x2, y2, x3, y3...}, where x and y are the coordinates of starting positions.

# MATLAB interface for reading output and generating plots/visualizations
To interface with MATLAB, the code interface_MATLAB.m calls vbm.exe (visibility based marching) using vbm.bat, where the components of the .bat file are: <br>
``` set path=%path:C:\Program Files\MATLAB\R2022b\bin\win64;=% ``` <br>
``` vbm.exe ``` <br>
Make sure to change the path for your MATLAB installation directory inside the .bat (and use the proper version). <br>
The code then parses the settings and reads the results and plots them nicely.

# Octave interface
Similar to MATLAB interface, provided by [@peter-ap](https://github.com/peter-ap).


# Instructions to build the C++ code on Windows in Visual Studio Code
We provide tasks.json, c_cpp_properties.json, and launch.json for building and launching the code in Visual Studio Code. <br>
We tested with gcc MINGW64 MSYS2 compiler: <br>
``` pacman -S mingw-w64-x86_64-gcc ``` <br>
Make sure to change the compiler path in tasks.json for both debug and release modes: <br>
``` "command": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\g++.exe" ```
The tasks.json automatically links the SFML libraries with the argument ``` "-lsfml-graphics" ```. <br>
For debug purposes, change "miDebuggerPath": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\gdb.exe" path as well in launch.json. <br>
Make sure your antivirus is not blocking the build if you are on Windows. <br>

Dependencies: <br>
If you are using Visual Studio Code and MSYS2: <br>
Install SFML in MSYS2 using:  <br>
``` pacman -S mingw-w64-x86_64-sfml ``` <br>

# Instructions to build the C++ code on Windows in Visual Studio Code using CMakeLists.txt
We provide CMakeLists.txt for easy building and compilation too. Install CMake Tools extension on VSCode and configure the kit and the generator. <br>
We used: <br>
``` pacman -S mingw-w64-x86_64-cmake ``` <br>
Make sure cmake is working using ``` cmake --version ```, a reload may be necessary <br>
Set the cmakePath accordingly in settings.json: <br>
``` "cmake.cmakePath": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\cmake.exe", ``` <br>
This has been tested with the CMake Kit ``` GCC 12.2.0 x86_64-w64-mingw32 ``` both in release and debug modes.

