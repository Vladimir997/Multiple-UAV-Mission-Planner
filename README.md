# Multiple-UAV-Mission-Planner

This project is a continuation of a previously created Mission Planner for multiple UAVs. The focus of this project is the calculation
of the path around previously drawn obstacles on the map. 

It uses an iterative algorithm, as opposed to the previous which uses a recursive. Some bugs are fixed when moving from one obstacle to
another. Also some unnecessary points are removed from the path.

The algorithm fails, when obstacles are too close to eachother. The solution to this should be found in merging the obtacles, as the
vehicle can't pass between them anyway. There is a function that tries to do that, but there are still some errors that need to be
fixed.

Other functions are changed so that they work with different kinds of obstacles, but the function for adding offset might need some
improvement. The video in the next link shows two drones avoiding multiple obstacles. The obstacles are positioned in a way that
all of the functions are used.

https://drive.google.com/file/d/1vF93Jjs-YCm5ilJZqMlpFoffNpUE3c8K/view?usp=sharing

It's possible to connect one drone to the mission planner and to FlightGear simultaneously, but not multiple drones at once. In order
to make this mission planner work with different kinds of vehicles, some changes need to be made in the code, because ardupilot
supports different kinds of vehicles, but the rover for example doesn't need a function like arm and take off.

The algorithm can be further optimized to avoid doing the same calculations when not needed.

Link to original project: https://github.com/95danlos/Multi-Robot-Mission-Planner
