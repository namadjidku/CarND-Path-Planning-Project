### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Reflection
The code for path planing consists of three sections: prediction, behavior planning and trajectory generation.

##### Prediction

Using sensor fusion data, we iterate through available agent cars and store in the vector 'predictions' d coordinate, predicted s coordinate and velocity for each agent. Velocity is computed as the sum of vectors vx and vy. For the highway, we assume that each agent will continue on the lane it is currently following with a constant velocity. Thus, for d coordinate there will be no changes, for s coordinate, we compute its predicted value using a linear motion model.    

##### Behavior Planning

For behaviour planning, FSM is used with three states: keep lane (KL), lane change left (LCL) and lane change right (LCR). Function successor_states returns successors for the current state. From LCL and LCR we can transit only to KL state. From KL state, we can go to states LCL and LCR depending on the current lane.

For each successor, we compute possible trajectories (function generate_trajectories). By trajectories we mean a lane number and a lane velocity. To generate possible trajectories, we iterate through the vector predictions, and estimate the agents positions. If we have an agent moving in front of us within 30 meters with a speed lower than the current velocity, we consider changing the lane. We can also stay in the current lane slowing down. For changing the lane, we check wether we have agents on the left and on the right within 10 meters gap.   

After the possible trajectories are computed, we estimate the cost of each trajectory. The trajectory with a minimum cost is considered as the final one. Cost is computed as exp((current_velocity-target_velocity) / (abs(current_lane-intended_line) + 1)). If current_velocity is less than target_velocity, the cost is small and the car will speed up, in case target_velocity is more than current_velocity, the cost is high and changing the lane will be more preferable than keeping the current one. 

###### Trajectory Generation

The code for trajectory generation was giving in the video, in the course material. 

It uses splines to interpolate the path points to get a smoth trajectory. The path points at each iteration is a set of 5 points. First two points are either last two from the previous trajectory or the current position of the vehicle and the estimated previous one. The other three points are generated in fernet coordinates, equally spaced by 30 meters starting after the reference point. After interpolation, we take the rest of points left from the previous trajectory and add the new generated ones up to 50 points all together. This approach guarantees a smoth transition between trajectories. 


##### Performance 
