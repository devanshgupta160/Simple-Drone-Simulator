# Simple-Drone-Simulator
### Authors:
 - Devansh Gupta(2019160)
 - Mudit Aggarwal(2019063)
### Cyborg - Robotics Club of IIIT-D
### Project Title: Drone Simulator from scratch
### References:
- https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
- https://liu.diva-portal.org/smash/get/diva2:1129641/FULLTEXT01.pdf
### Description:
- A simple simulator which does not take wind-speed in the environment into account by design
- For mechanics, uses a small angle approximation and for corresponding control uses a simple PID
    model which can be accessed by the second link in the references.
- Can serve as a simple test-bed for testing different control techniques on underactuated systems
    or quadrotors.
### Implementation:
- Used an approximate version of the Newton-Euler Rigid Body equations to model the mechanics of the drone
- Used an inner PID loop to set the temporary positional destnations and calculated the respective roll
  and pitch angle deestinations and used the change in the respective error of yaw angles to configure the
  PID destination for yaw and finally applied the PID on the yaw,roll,and pitch angles.
### Running code on your system:
- Clone this repository
- Install Matplotlib and Numpy in your system
- run the command
    - python3 simulator.py
 - to run the simulator on the default param and waypoint files provided in the repo
- run the command:
    - python3 simulator.py --waypoints [filename1.txt] --params [filename2.txt] 
  - to run the simulator on your own waypoint and param files named filename1.txt and filename2.txt
    respectively
