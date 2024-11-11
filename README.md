# RoboOp Library
Library for autonomous operating robots.
## Install Instructions
### CMake
Builds the shared library and the test binary.
```
cmake -B build
cmake --build build
```
### Python
Copies over the source and header files into their respective folders in your current directory.
```
python setup.py
```
## Features
### Spline path generation
Generate quintic spline paths by inputting control points.
```cpp
std::shared_ptr<Trajectory> path = TrajectoryBuilderFactory::create({{0, 0}, 0})
        .to({{10, 10}, 0.25})
        .to({{20, 10}, 0})
        .build();
```
![Splines](https://github.com/user-attachments/assets/d7be3dc5-1212-4b20-9aaf-f71e97a21b9a)
### Ramsete
The Ramsete controller is currently under testing.
### Tank Drive
Tank drive implementation for path following. 
### Future Plans
- Localization implementations
- Motion profiling
- GUI visualizer (Separate Project)
