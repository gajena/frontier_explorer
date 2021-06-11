# robot_exploration
Initial implementation of this one:
```
T. Cieslewski, E. Kaufmann and D. Scaramuzza, "Rapid exploration with multi-rotors: 
A frontier selection method for high
speed flight," 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 
Vancouver, BC, 2017, pp.
2135-2142.
doi: 10.1109/IROS.2017.8206030
```

Abstract:
``` 
Exploring and mapping previously unknown environments while avoiding collisions with obstacles is 
a fundamental task for autonomous robots. In scenarios where this needs to be done rapidly, 
multi-rotors are a good choice for the task, as they can cover ground at potentially very 
high velocities. Flying at high velocities, however, implies the ability to rapidly
plan trajectories and to react to new information quickly. In this paper, we propose 
an extension to classical frontier based exploration that facilitates exploration at 
high speeds. The extension consists of a reactive mode in which the
multi-rotor rapidly selects a goal frontier from its field of view. 
The goal frontier is selected in a way that minimizes the change in 
velocity necessary to reach it. While this approach can increase 
the total path length, it significantly reduces the exploration time, since the multi-rotor 
can fly at consistently higher speeds.
```
and this one:
```
A. Bircher, M. Kamel, K. Alexis, H. Oleynikova and R. Siegwart, 
"Receding Horizon "Next-Best-View" Planner for 3D Exploration," 
2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016, pp. 1462-1468.
doi: 10.1109/ICRA.2016.7487281
```
Abstract: 
```
This paper presents a novel path planning algorithm for the autonomous exploration of 
unknown space using aerial robotic platforms. The proposed planner employs a receding 
horizon “next-best-view” scheme: In an online computed random tree it finds the best branch, 
the quality of which is determined by the amount of unmapped space that can be explored. 
Only the first edge of this branch is executed at every planning step, 
while repetition of this procedure leads to complete exploration results. 
The proposed planner is capable of running online, onboard a robot with limited resources. 
Its high performance is evaluated in detailed simulation studies as well as in a challenging
real world experiment using a rotorcraft micro aerial vehicle. Analysis on the computational
complexity of the algorithm is provided and its good scaling properties enable the handling 
of large scale and complex problem setups.
```
## Credit
initially inspired from: https://github.com/demplo/robot_exploration
