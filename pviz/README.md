pviz
====

PR2 Visualization Library
_by Benjamin Cohen (with new features added by Ellis Ratner)_

A very easy to use library that allows you to visualize the PR2 robot in
Rviz using visualization markers - with one line of code. It also has easy
to use functions to visualize all geometric shapes, meshes and text. This
library is great for debugging. 

PViz relies on the KDL for kinematics which means that TF and Gazebo are not
needed.

**_Note: the master branch is tested working with Groovy (very possible still works in Fuerte)_**

#### Visualization Functions:

##### PR2:
 * visualizeRobot
 * visualizeRobotWithTitle
 * visualizeGripper
 * visualizeTrajectory

##### Other:
 * visualizePose
 * visualizeSphere
 * visualizeCube
 * visualizeLine
 * visualizeMesh
 * visualizeText

#### Marker Functions:

##### Get Filled in Marker Msgs:
 * getRobotMarkerMsg
 * getRobotMeshesMarkerMsg
 * getGripperMeshesMarkerMsg
 * getCubeMsg

##### Work with Marker Msgs:
 * publish   (overloaded for Marker & MarkerArray)
 * deleteVisualizations

**_(Many of the functions above are overloaded with many different options of parameters to use)_**

## Install

 `git clone https://github.com/bcohen/leatherman.git`

 `git clone https://github.com/bcohen/pviz.git`
 
 `rosmake pviz`
 
## Demo

 `roslaunch pviz pviz_example.launch`
 
 In rviz, change the fixed_frame to "/map" and add a visualization marker display. You can leave the topic in the display as the default.
 
 





