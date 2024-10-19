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
### Pure Pursuit
Follows the path using pure pursuit.
```cpp
follower.followPath(path);
while (follower.update()) {
  // Other tasks
}
```
### Tank Drive
Tank drive implementation for path following.
```cpp
class TankDrive: public IDriveTrain {
private:
    std::shared_ptr<MotorGroup> left;
    std::shared_ptr<MotorGroup> right;
    std::shared_ptr<PID> translationalPID, rotationalPID;
    Eigen::Vector2d dimensions;
public:
    TankDrive(
        const Eigen::Vector2d& dimensions,
        const std::shared_ptr<MotorGroup>& left,
        const std::shared_ptr<MotorGroup>& right,
        const std::shared_ptr<PID>& rotationalPID,
        const std::shared_ptr<PID>& translationalPID
    ): dimensions{dimensions}, left{left}, right{right}, rotationalPID{rotationalPID}, translationalPID{translationalPID}{}
    void setTarget(const Pose2d& point) override;
};
```
![PathFollowing](https://github.com/user-attachments/assets/bde5b021-e757-4931-8267-993c2b3dca61)
### Future Plans
- Localization implementations
- Motion profiling
- GUI visualizer (Separate Project)
