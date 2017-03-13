# RPP

## Overview

The purpose of this program is the calculation of the optimal path (in terms of distance traveled) and the safest path,
between two specified points in a convex environment with convex or line obstacles . 

The robot can be one of the three following types : disk,line or rectangular.

The paths can be calculated for two types of motion: 

* Pure translational motion : The robot retains a constant orientation in its movement.

* Combined rotational and translational motion : The robot can change its orientation while moving.

(For disk robots , pure translational and combined motions are identical).

## Screenshots

<img src = "https://cloud.githubusercontent.com/assets/20325266/23851681/097ed4fc-07ed-11e7-9370-f1aa3eea29bb.jpg" width = "49%" >
<img src = "https://cloud.githubusercontent.com/assets/20325266/23851682/0a7ea8fa-07ed-11e7-8c1f-0ef5ebff7276.jpg" width = "49%" >

## Dependencies

This program uses the following libraries :

* *vvr framework* : Graphical environment implementation . 

* *geolib* : Data representation of various objects and shapes, utilization of several computational geometry algorithms.   
             ( http://www.geolib.co.uk/ )

* *clipper* : Calculation of unions,intersections and differences between polygons . Clipper is licensed under the Boost Software License.
               ( http://www.angusj.com/delphi/clipper.php )

## Installation

## How it works

### Minkowski sums, modified boundary and resulting free space

In order to simplify calculations , the robot is considered to be a point in space (specifically,the point chosen is always its center).
The information about its dimensions is 'transferred' to the obstacles and the boundary of the environment . They are modified accordingly so that the resulting environment is mathematically equivalent to its initial counterpart.

The mathematical tool used in order to correctly modify the dimensions of the obstacles, are the Minkowski sums (calculated in an approximate manner, through selecting key points of both the robot and the obstacle).

For the boundary , different approximate geometric techniques are used (note that the environment boundary contracts as the dimensions
of the robot increase while the obstacles dilate , and vice versa). A modified environment boundary is extracted.

By calculating the union of the modified obstacles and afterwards the boolean difference with the modified environment boundary, we
can extract the polygonal space in which the robot can move. We will refer to it as **free space**.

### Generalized Voronoi diagram

The generalized Voronoi diagram is used to calculate the safest path. The steps used for its calculations are the following :

* The obstacles and the environment boundary are discretized . The discretization is more dense at the areas where their direction changes. With this tecnhique , we get a better approximation of the initial environment characteristics.

* For every resulting point of the discretization , the Voronoi diagram is calculated (using the half-plane intersection method). A large number of cells is generated after this procedure.

* Of all the cell lines that are generated , the ones that pass through obstacles or exit the environment boundaries are deleted. 
 The remaining lines are transformed into a graph representation.

### Safest path calculation

For the calculation of the safest path, the generalized Voronoi diagram is utilized.

Specifically, the procedure is the following :

* For the starting and ending points , the nearest nodes of the graph are found, and the respective lines are 'drawn'.
* A greedy best-first algorithm is implemented, to find a path between the starting and ending nodes. The metric used is the euclidean distance between the nodes of the graph.

### Optimal path calculation

The metric with which optimality is pursued is the total distance travelled.
For the calculation of the optimal path , the following procedure is implemented :

* An attempt to draw a straight line between the starting and ending points is made. If that is possible without an intersection occuring,then the straight path defined by that line is the optimal path, and the algorithm terminates.
* If a straight line is not possible , then another approach is used. From the starting point , a very small line is drawn with direction to the ending point. This line is continuously (and slowly with each iteration) increasing in size. Upon a 'collision' with an obstacle , two possible paths are created : one which travels in a clockwise manner to the boundary of the obstacle (the modified version)  and one which travels in a counter-clockwise manner.
* As each line progresses upon the obstacle boundary, a continuous check on whether it can stop travelling on the boundary and resume its straight path towards the ending point is made.
* This algorithm repeats from the second step and beyond : all the possible paths make an effort to reach the ending point , and every time a path meets an obstacle,it is split into two possible paths.
* When all the paths have reached their destination , the optimal in terms of distance travelled is finally chosen.

### Extra procedures & calculations for the combined motion

For the rotational motion , the input is not limited to the positions of the starting and ending points. The orientation of the robot at these positions must also be given.

The following procedure is implemented :

* For an (evenly spaced) number of different angles , the free space is calculated. 

* The union of all the free spaces is calculated . The resulting space will represent , in a sense, the best case scenario for all the angles used in the previous step.

* For the resulting space, the previously mentioned algorithms are implemented for the calculation of the safest and optimal paths , but with one distiction. For every step (for every node in the case of the safest path, and for every growth interval for the optimal) a collision check is made with the current orientation. If no collision is detected, the orientation persists. Otherwise, a new orientation is chosen (from the set of angles chosen in the first step) . If multiple suitable new orientations are found, the one which requires the smallest change in angle is selected.

## Usage, input and parameters

**Note for compilation with Visual Studio** : For better execution speed, select 'Release' in the solution configuration.


## License

For personal , educational and/or other uses covered under fair use. This limitation is due to the dependencies of this program.

Copyright (C) 2017 Stelios Tsiakalos
