# To-Do List

## System Architecture

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

<p align="center">
  <img src="./assets/Architecture.jpg" width="95%">
</p>

## 1. Traffic Light Detection:  


<table style="width:100%">
 <tr>
   <th align="center">Package Path</th>
   <td>/ros/src/tl_detector/</th>
 </tr>
 <tr>
   <th  align="center">Diagram</td>
   <td><img src="./assets/tl_detector.jpg" width="100%"></td>
 </tr>
 <tr>
   <th  align="center">Function</td>
   <td> Publishes the locations to stop for red traffic lights</td>
 </tr>
</table>

The detection of the traffic lights should take place in `/tl_detector/tl_detector.py`

### Traffic Light Classification

Classification should take place in `/tl_detector/light_classification_model/tl_classfier.py`


## 2. Waypoint Updater:  


<table style="width:100%">
 <tr>
   <th align="center">Package Path</th>
   <td>/ros/src/waypoint_updater/</th>
 </tr>
 <tr>
   <th  align="center">Diagram</td>
   <td><img src="./assets/waypoint-updater.jpg" width="100%"></td>
 </tr>
 <tr>
   <th  align="center">Function</td>
   <td> Updates the target velocity property of each waypoint based on traffic light and obstacle detection data. Publish a list of waypoints ahead of the car with their respective target velocities</td>
 </tr>
</table>

## 3. Vehicle Controller:

<table style="width:100%">
 <tr>
   <th align="center">Package Path</th>
   <td>/ros/src/twist_controller/</th>
 </tr>
 <tr>
   <th  align="center">Diagram</td>
   <td><img src="./assets/DBW_Node.jpg" width="100%"></td>
 </tr>
 <tr>
   <th  align="center">Function</td>
   <td> Will publish throttle, brake and steering commands in order to achieve target linear and angular velocities</td>
 </tr>
</table>
