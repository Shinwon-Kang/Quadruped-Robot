# Quadruped Robot


![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white) ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

Sequence Control + Pose Optimization  

Robot : Unitree A1

## Flow Chart
![flowchart](https://user-images.githubusercontent.com/28734653/156926509-ec0896cd-ae10-4c2b-bc94-eb41c8c6f3f5.png)

## Gait Pattern Generator
![Screenshot from 2022-03-06 23-24-00](https://user-images.githubusercontent.com/28734653/156927414-dd2c9f21-1e91-45ce-b205-17445feb3263.png)


## Leg Trajectory Generator
- Bézier curve

## Pose Control
Optimization to find the robot's posture that minimizes the distance from the tip of the foot to the shoulder.
- Quadratic Programming  
![Screenshot from 2022-03-06 23-27-30](https://user-images.githubusercontent.com/28734653/156927487-af41e6b3-c79e-41cb-9ab8-363408eed059.png)
