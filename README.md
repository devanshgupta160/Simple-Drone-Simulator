# Drone Simulator
Simulators play an important role in engineering as they provide a harmless and a convenient way to test a particular system against various situations to make our system ready for different adversities which may occur in the real world with a less but a valid probability. Due to the high chance of failure in aerial robotics and the high cost of damage, simulators are necessary in order to move work forward. So, curious about how simulators work, we here at Cyborg-the robotics club of IIITD  have developed our own prototype of a drone simulator which seeks to simulate a category of aerial robots known as quadrotors from scratch.

**What is a quadrotor?**

A quadrotor is a type of drone which has four rotors equidistant from the center of the drone, whose variations in speeds of individual rotors are used to move it in the forward, backward,sideways directions and also used to rotate the drone along its axis.

The forward and backward motion of the drone is known as pitch, the sideways motion is known as the roll and the rotatory motion along the axis perpendicular to the plane is known as yaw.

**How does a quadrotor work?**

The four rotors of a quadrotor are used to rotate the propellers which are then used to push the air downwards to give a liftoff to the quadrotor. Now the rotation of the four individual propellers are programmed in such a way that in the case of an unbalanced torque, the drone does not rotate due to the imbalance of rotational momentum caused by the propellers. So therefore, propellers along one direction rotate clockwise while the propellers along the other diagonal rotate along the anticlockwise direction.


![rotor_labelling.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/rotor_labelling.png "image_tooltip")

*   **Roll**

       While rolling, we are going to ensure that the rotational velocity of the left hand rotors(3 and 4) is greater than the rotational velocity of the right hand rotors(1 and 2) and vice versa depending on the direction in which we are to drive the drone.

*   **Pitch**

       While pitching, we are going to ensure that the rotational velocity of the forward rotors(1 and 4) is greater than the rotational velocity of the backward rotors(2 and 3) and vice versa depending on the direction in which we are to drive the drone.

*   **Yaw**

       While performing yaw, we will ensure that the opposite rotors still have same speed as we are to ensure that the drones are to not roll or pitch but we increase rotational speed on one pair of opposite rotors(say  1 and 3) which causes an imbalance of angular momentum along the z-axis causing the drone to rotate along that particular axis without compromising the balance of momentum of roll and pitch.


Now, we have covered the basic intuition of drones but how are we to make a method which decides which combination of these three actions to take in order for the drone to reach a point that we want it to reach without keeping trajectory constraints in mind? So, essentially we are to describe the next state of the system from the current state using a series of functions applied on each state value and we get an overall working map of the drone’s internal control which we have shown below.


![flowchart.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/flowchart.png)

Now, if we are to formalize our core problem, then if would be as follows:

_Find a set of control equations such that when we release the drone from a state S(current point and velocity) and it is to reach a point corresponding to the state S’(final point and zero velocity as it has reached that point) then our equation must force the system abiding by is laws of mechanics in such a state I from S such that ||S’-I||&lt;=k for a small constant k for a large amount of time. The equations must bring the drone from state S to state I in a favorable amount of time._

**Quadrotor Mechanics**


![basic_orientation.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/basic_orientation.png "image_tooltip")

The quadrotor is dealt with in its own frame, different from the inertial frame of reference. The rotations are dealt with by using Euler angle based mechanics.

![orientation.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/orientation.png "image_tooltip")

The Euler angles based system works by breaking down every rotation into 3 consecutive fixed axis rotations (through the psi, theta, and phi angles). 


The model uses a state vector with 12 states:

	[x, y, z, psi, theta, phi, x’, y’, z’, psi’, theta’, phi’]

This is enough information to define the drone at any particular point uniquely. 

The main equations boil down to (Here ft represents the thrust factor associated with the quadrotor):

![dynamics_eqns.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/dynamics_eqns.png "image_tooltip")

This might be a slightly more helpful schematic of the drone for the equations:

![detailed_diagram.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/detailed_diagram.png "image_tooltip")

In addition to the above equations, we also used the Euler Newton Equation, as well as applied some small angle approximations.However, the approximations are close enough to effectively model most real time scenarios the drone might face.Additionally, we have also not considered the effect of wind speed on the drone. We have implemented the dynamics in the state vector representation version of the above equation which can be cleared from the code below:

![dynamics.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/dynamics.png "image_tooltip")

**Quadrotor Control**

The quadrotor uses a standard PID controller as its error reducing entity, that stands for Proportional-Integral-Derivative controller. The quadrotor can move in the x, y, z direction, as well as rotate through psi, theta, and phi. Hence, we need to control the rotors keeping all these 6 conditions in mind. 

The drone is assumed to be in the linear region of controls (small angle approximations), and the proportional, integral, and derivative of the errors of the coordinates are scaled and added to the RPMs accordingly. This implements a very standard PID controller.

The control of this quadrotor is according to the control schematic


![schematic.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/schematic.png "image_tooltip")


We have simulated the Height Controller, Position Controller, and Angle Controller in the code snippet given below and each block represents a PID controller(described below), which we can think of a device which minimizes the error between the desired value and current value by sending appropriate signals to the system. So, we have implemented a multi loop PID controller w.r.t the Position and Angle where the internal loop is the Position Controller where the desired values are fed and corresponding angle aims or references are set up for the external PID loop of the Angle controller. We can think of it in a way that if we are to move the drone to the right by a certain amount of units, then we can say that we are essentially mapping one rightward motion into a lot of angular configurations in between which the Angle controller tries to achieve. We will show the code snippet of the controller after we discuss PID in detail, so that there is a certain amount of clarity when interpreting the code. We need not simulate the Kalman Filters as we are simulating a drone so, we get those values by just computing the state dots and updating the current state using linearization to the next state and feeding the next state into the dynamics and so on.

**PID Controller**

Note that this section is a small digression into PID control and can be skipped if you understand the basic workings of a PID controller.

The schematic of a PID controller can be shown as below:-

![PID.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/PID.png "image_tooltip")


A PID controller consists of three parts:-

1. Proportional Component: This component is the main component which drives the agent to the region of low error of a particular measure. It is basically the component that we obtain when we multiply the error at that instant by a constant. If we provide this as an actuation, then we can see that the agent will head towards to low error and then will oscillate lightly about the required region as it will be in a cycle i.e. negative error then actuation then overshoot then positive error and then opposite directioned actuation and back to initial state. But, we can intuitively see with the above series of actions that in that actuation, it does get the agent to the state with low error.
2. Integral Component: Now, this component is the component which if implemented correctly does two things on top of the proportional constraint, first it reduces the amplitude of the oscillations and secondly, it makes the agent robust against noise in the environment(for example wind speed in our case would be fixed by the integral component of our multi loop PID). So, the integral component is calculated by summing all the previous errors(equivalent to integration of error function) and multiplying it with a constant and adding it to our actuation we are to provide to the agent. Now, we can see that if the agent is oscillating then, it will seek to decay the oscillations as if there is an overshoot in the positive error in the previous errors, then the agent when in a negative error state will reduce the absolute value of its error with a positive actuation added and vice versa will also be true. This repeatedness leads to a decayed oscillation, but only if our integral values are controlled and hence our integral constant is usually low compared to the derivative and the proportional constant. Secondly, it becomes robust against noise as the sum of all the previous errors cancels the spike in error in some cases and makes the system robust to noise.
3. Derivative Component: This component is used to stop the oscillations altogether and causes the agent to almost perfectly follow the reference point. In this component we essentially add the derivative of the error function scaled by a constant by a certain activation. Now this stops the oscillations as it adds the rate of change to the original actuation and hence at very rapid changes and oscillations, it regularizes and stabilizes the oscillations eventually and hence we obtain a stable agent.

One thing to keep in mind is that these constants should be in a particular range in order to get good stability or else the agent will not be as stable as expected. All these values are found for all the 6 movements we discussed above, and are scaled with their necessary constants (which have to be empirically determined). They are then added to the RPMs of their corresponding rotors.

The code snippet for the controller we used in the simulation.


![controlcode.png](https://github.com/devanshgupta160/Simple-Drone-Simulator/blob/master/Images/controlcode.png)

**Input and Modelling**

The drone takes the waypoints it has to cross from a separate waypoints text file. We need to specify the coordinates of the point, as well as the orientation of the drone (as yaw). The simulator is made on Python, and uses Matplotlib to plot the simulations.

**References**



*   [Quadrotor control: modeling, nonlinear control design, and simulation](https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf)
*   [Position and Trajectory Control of a Quadcopter Using PID and LQ Controllers](https://liu.diva-portal.org/smash/get/diva2:1129641/FULLTEXT01.pdf)
*   [File:PID-feedback-loop-v1.png](https://en.wikipedia.org/wiki/File:PID-feedback-loop-v1.png)

### Running code on your system:
- Clone this repository
- Install Matplotlib and Numpy in your system
- Run the command below to run the simulator on the default param and waypoint files provided in the repo
   - python3 simulator.py
- Run the command below to run the simulator on your own waypoint and param files named filename1.txt and 
  filename2.txt respectively
   - python3 simulator.py --waypoints [filename1.txt] --params [filename2.txt] 
### Authors:
 - Devansh Gupta(2019160)
 - Mudit Aggarwal(2019063)
