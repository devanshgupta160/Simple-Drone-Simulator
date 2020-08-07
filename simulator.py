"""
Authors:
	Devansh Gupta(2019160)
	Mudit Aggarwal(2019063)
Cyborg - Robotics Club of IIIT-D

Project Title: Flight Simulator from scratch

References:
	1. https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
	2. https://liu.diva-portal.org/smash/get/diva2:1129641/FULLTEXT01.pdf

Cross Validation Code: https://github.com/abhijitmajumdar/Quadcopter_simulator/blob/master/controller.py

Description:
    1. A simple simulator which does not take wind-speed in the environment into account by design
    2. For mechanics, uses a small angle approximation and for corresponding control uses a simple PID
    model which can be accessed by the second link in the references.
    3. Can serve as a simple test-bed for testing different control techniques on underactuated systems
    or quadrotors.
"""

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
import argparse

def minmax(val, range):
    if val>range[1]:
        val = range[1]
    if val<range[0]:
        val = range[0]
    return val

def reached(waypoint,a,b,c,d,max_error_coord=0.09, max_error_yaw=0.05):
    return ((abs(waypoint[0]-a) <= max_error_coord) and (abs(waypoint[1]-b) <= max_error_coord)\
         and (abs(waypoint[2]-c) <= max_error_coord) and (abs(waypoint[3]-d) <= max_error_yaw))

def load_params(drone, file):
    f = open(file, "r")
    params = f.readlines()
    f.close()
    drone.com[0],drone.com[1],drone.com[2],drone.theta,drone.phi,drone.psi = map(float,params[0].strip().split(" ")[1].split(","))
    drone.mass = float(params[1].strip().split(" ")[1])
    drone.l = float(params[2].strip().split(" ")[1])
    drone.r = float(params[3].strip().split(" ")[1])
    drone.prop_dia = float(params[4].strip().split(" ")[1])
    drone.prop_pitch = float(params[5].strip().split(" ")[1])
    drone.yaw_error_rate_scaler = float(params[6].strip().split(" ")[1])
    drone.tilt_limits[0],drone.tilt_limits[1] = eval(params[7].strip().split(" ")[1].split(",")[0]), eval(params[7].strip().split(" ")[1].split(",")[1])
    drone.yaw_limits = list(map(float,params[8].strip().split(" ")[1].split(",")))
    drone.motor_speed_limits = list(map(float,params[9].strip().split(" ")[1].split(",")))
    drone.linear_pid['X']['P'],drone.linear_pid['X']['I'],drone.linear_pid['X']['D'] = map(float,params[10].strip().split(" ")[1].split(","))
    drone.linear_pid['Y']['P'],drone.linear_pid['Y']['I'],drone.linear_pid['Y']['D'] = map(float,params[11].strip().split(" ")[1].split(","))
    drone.linear_pid['Z']['P'],drone.linear_pid['Z']['I'],drone.linear_pid['Z']['D'] = map(float,params[12].strip().split(" ")[1].split(","))
    drone.angular_pid['th']['P'],drone.angular_pid['th']['I'],drone.angular_pid['th']['D'] = map(float,params[13].strip().split(" ")[1].split(","))
    drone.angular_pid['ph']['P'],drone.angular_pid['ph']['I'],drone.angular_pid['ph']['D'] = map(float,params[14].strip().split(" ")[1].split(","))
    drone.angular_pid['ps']['P'],drone.angular_pid['ps']['I'],drone.angular_pid['ps']['D'] = map(float,params[15].strip().split(" ")[1].split(","))
    return drone, float(params[16].strip().split(" ")[1]), float(params[17].strip().split(" ")[1]),\
        float(params[18].strip().split(" ")[1]), params[19].strip().split(" ")[1],\
            params[20].strip().split(" ")[1], params[21].strip().split(" ")[1],\
            params[22].strip().split(" ")[1]

class Drone():
    #------------DEFINING COM AND PROP POSITIONS-----------
    com = np.array([0.0,0.0,0.0])
    prop1 = np.array([0.0,0.0,0.0])
    prop2 = np.array([0.0,0.0,0.0])
    prop3 = np.array([0.0,0.0,0.0])
    prop4 = np.array([0.0,0.0,0.0])
    #-----------DEFINING ROTOR SPEEDS-----------------------
    r1 = 0.0
    r2 = 0.0
    r3 = 0.0
    r4 = 0.0
    #-----------DEFINING INITIAL ANGLES---------------------
    theta = 0.0
    phi = 0.0
    psi = 0.0
    #-----------DEFINING STATE VECTOR-----------------------
    state = np.zeros((12,))
    state_dot = np.zeros((12,))
    #-----------DEFINING INTRINSIC PARAMETERS---------------
    mass = 1.2
    l = 0.3
    r = 0.1
    moi_x = ((2*(3*mass/5)*r**2)/5)+(2*(mass/10)*l**2)
    moi_y = moi_x
    moi_z = ((2*(3*mass/5)*r**2)/5)+(4*(mass/10)*l**2)
    prop_pitch = 4.5
    prop_dia = 10.0
    thrust_fac = 4.392e-8 * (pow(prop_dia,3.5)/sqrt(prop_pitch)) * 4.23e-4 * prop_pitch
    drag_fac = thrust_fac*0.0245
    delta_t = 0.01
    g = 9.80665
    #----------PID UTILITIES---------------------------------
    int_linear_err = {'X':0.0,'Y':0.0,'Z':0.0}
    int_angular_err = {'th':0.0,'ph':0.0,'ps':0.0}
    linear_pid = {'X':{'P':300.0,'I':0.04,'D':450.0},'Y':{'P':300.0,'I':0.04,'D':450.0},'Z':{'P':7000.0,'I':4.5,'D':5000.0}}
    angular_pid = {'th':{'P':22000.0,'I':0.0,'D':12000.0},'ph':{'P':22000.0,'I':0.0,'D':12000.0},'ps':{'P':1500.0,'I':1.2,'D':0.0}}
    tilt_limits = [-pi/3, pi/3]
    yaw_limits = [-800,800] 
    motor_speed_limits = [4000,10000]
    yaw_error_rate_scaler = 0.18
    def initialize(self):
        self.set_initial_state_vector()
        self.set_other_coordinates()
    
    def set_initial_state_vector(self):
        self.state[0] = self.com[0]
        self.state[1] = self.com[1]
        self.state[2] = self.com[2]
        self.state[6] = minmax(self.theta, self.tilt_limits)
        self.state[7] = minmax(self.phi, self.tilt_limits)
        self.state[8] = self.set_cyclic_angle(self.psi)

    def set_cyclic_angle(self,vec):
         return (vec+np.pi)%(2*np.pi) - np.pi

    def set_com(self):
        self.com = self.state[0:3]

    def get_orientation_vector(self):
        v1 = self.prop2 - self.com
        v2 = self.prop1 - self.com
        dir_vec = np.cross(v2,v1)
        return self.com,dir_vec

    def set_angles(self):
        self.theta = minmax(self.state[6],self.tilt_limits)
        self.phi =  minmax(self.state[7],self.tilt_limits)
        self.psi = self.state[8]

    def set_other_coordinates(self):
        rot_x = np.array([[1.0,0.0,0.0],[0.0,cos(self.theta),-1*sin(self.theta)],[0.0,sin(self.theta),cos(self.theta)]])
        rot_y = np.array([[cos(self.phi),0.0,sin(self.phi)],[0.0,1.0,0.0],[-1*sin(self.phi),0.0,cos(self.phi)]])
        rot_z = np.array([[cos(self.psi),-1*sin(self.psi),0.0],[sin(self.psi),cos(self.psi),0.0],[0.0,0.0,1.0]])
        rot_net = np.dot(rot_z,np.dot(rot_y,rot_x))
        self.prop2 = self.com-np.dot(rot_net, np.array([self.l,0,0]))
        self.prop1 = self.com+np.dot(rot_net, np.array([0,self.l,0]))
        self.prop4 = self.com+np.dot(rot_net, np.array([self.l,0,0]))
        self.prop3 = self.com-np.dot(rot_net, np.array([0,self.l,0]))

    def set_state_dots(self):
        self.set_angles()
        moi = np.array([[self.moi_x, 0.0, 0.0],[0.0, self.moi_y, 0.0],[0.0, 0.0, self.moi_z]])
        moi_inv = np.array([[1/self.moi_x, 0.0, 0.0],[0.0, 1/self.moi_y, 0.0],[0.0, 0.0, 1/self.moi_z]])
        rot_x = np.array([[1.0,0.0,0.0],[0.0,cos(self.theta),-1*sin(self.theta)],[0.0,sin(self.theta),cos(self.theta)]])
        rot_y = np.array([[cos(self.phi),0.0,sin(self.phi)],[0.0,1.0,0.0],[-1*sin(self.phi),0.0,cos(self.phi)]])
        rot_z = np.array([[cos(self.psi),-1*sin(self.psi),0.0],[sin(self.psi),cos(self.psi),0.0],[0.0,0.0,1.0]])
        rot_net = np.dot(rot_z,np.dot(rot_y,rot_x))
        self.state_dot[0] = self.state[3]
        self.state_dot[1] = self.state[4]
        self.state_dot[2] = self.state[5]
        X_double_dot = np.array([0.0,0.0,-1*self.g]) + (1/self.mass)*np.dot(rot_net,np.array([0.0,0.0,self.thrust_fac*(self.r1**2 + self.r2**2 + self.r3**2 + self.r4**2)]))
        self.state_dot[3] = X_double_dot[0]
        self.state_dot[4] = X_double_dot[1]
        self.state_dot[5] = X_double_dot[2]
        self.state_dot[6] = self.state[9]
        self.state_dot[7] = self.state[10]
        self.state_dot[8] = self.state[11]
        omega = self.state[9:12]
        tau = np.array([self.l*self.thrust_fac*(self.r1**2-self.r3**2), self.l*self.thrust_fac*(self.r2**2-self.r4**2), self.drag_fac*(self.r1**2-self.r2**2+self.r3**2-self.r4**2)])
        omega_dot = np.dot(moi_inv, (tau - np.cross(omega, np.dot(moi,omega))))
        self.state_dot[9] = omega_dot[0]
        self.state_dot[10] = omega_dot[1]
        self.state_dot[11] = omega_dot[2]

    def dynamics_euler_angles(self):
        self.set_state_dots()
        self.state += self.delta_t*self.state_dot
        self.state[6:9] = self.set_cyclic_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])
        self.set_angles()
        self.set_com()
        self.set_other_coordinates()
        result = []
        result.extend(list(self.com))
        result.extend(list(self.prop1))
        result.extend(list(self.prop2))
        result.extend(list(self.prop3))
        result.extend(list(self.prop4))
        return result
    
    def controller_update(self,x_des,y_des,z_des,yaw_des):
        x_error = x_des - self.state[0]
        y_error = y_des - self.state[1]
        z_error = z_des - self.state[2]
        self.int_linear_err['X'] += x_error
        self.int_linear_err['Y'] += y_error
        self.int_linear_err['Z'] += z_error
        change_wrt_x = self.linear_pid['X']['P']*x_error - self.linear_pid['X']['D']*(self.state[3]) + self.linear_pid['X']['I'] * self.int_linear_err['X']
        change_wrt_y = self.linear_pid['Y']['P']*y_error - self.linear_pid['Y']['D']*(self.state[4]) + self.linear_pid['Y']['I'] * self.int_linear_err['Y']
        change_wrt_z = self.linear_pid['Z']['P']*z_error - self.linear_pid['Z']['D']*(self.state[5]) + self.linear_pid['Z']['I'] * self.int_linear_err['Z']
        dest_theta = change_wrt_x*sin(self.state[8]) - change_wrt_y*cos(self.state[8])
        dest_phi = change_wrt_x*cos(self.state[8]) + change_wrt_y*sin(self.state[8])
        dest_theta = minmax(dest_theta, self.tilt_limits)
        dest_phi = minmax(dest_phi, self.tilt_limits)
        theta_error = dest_theta - self.state[6]
        phi_error = dest_phi - self.state[7]
        psi_error = yaw_des - self.state[8]
        psi_dot_error = self.yaw_error_rate_scaler*self.set_cyclic_angle(psi_error) - self.state[11]
        self.int_angular_err['th'] += theta_error
        self.int_angular_err['ph'] += phi_error
        self.int_angular_err['ps'] += psi_dot_error
        change_wrt_th = self.angular_pid['th']['P']*theta_error - self.angular_pid['th']['D']*self.state[9] + self.angular_pid['th']['I'] * self.int_angular_err['th']
        change_wrt_ph = self.angular_pid['ph']['P']*phi_error - self.angular_pid['ph']['D']*self.state[10] + self.angular_pid['ph']['I'] * self.int_angular_err['ph']
        change_wrt_ps = self.angular_pid['ps']['P']*psi_dot_error + self.angular_pid['ps']['I'] * self.int_angular_err['ps']
        change_wrt_ps = minmax(change_wrt_ps, self.yaw_limits)
        self.r1 += change_wrt_z + change_wrt_ps + change_wrt_th
        self.r2 += change_wrt_z - change_wrt_ps + change_wrt_ph
        self.r3 += change_wrt_z + change_wrt_ps - change_wrt_th
        self.r4 += change_wrt_z - change_wrt_ps - change_wrt_ph
        self.r1 = minmax(self.r1, self.motor_speed_limits)
        self.r2 = minmax(self.r2, self.motor_speed_limits)
        self.r3 = minmax(self.r3, self.motor_speed_limits)
        self.r4 = minmax(self.r4, self.motor_speed_limits)

    def debug(self):
        print("{} {} {} {}".format(self.r1, self.r2, self.r3, self.r4))
    
fig = plt.figure()
ax = plt.axes(projection="3d")

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Simulation Files')
    parser.add_argument('--waypoints', default='waypoints_sample.txt', type=str,
                    help='Takes in the required waypoint file')
    parser.add_argument('--params', default='params_sample.txt', type=str,
                        help='Takes in the required parameter file')
    args = parser.parse_args()
    wp_file = open(args.waypoints,"r")
    wp_reader = wp_file.readlines()
    wp_file.close()
    waypoints = []
    for waypoint in wp_reader:
        waypoint = list(map(float,waypoint.strip()[1:-1].split(",")))
        waypoint[3] *= pi/180 
        waypoints.append(waypoint)
    colors = ["black", "red", "red", "blue", "blue"]
    labels = ["","1", "2", "3", "4"]
    d = Drone()
    d, wait_time, max_error_coord, max_error_yaw,ret,trav_path,wp_path,ori = load_params(d,args.params)
    ret = (ret=="True")
    trav_path = (trav_path=="True")
    wp_path = (wp_path=="True")
    ori = (ori=="True")
    d.initialize()
    if ret:
        if waypoints[-1] != [d.com[0],d.com[1],d.com[2]]:
            waypoints.append([d.com[0],d.com[1],d.com[2],d.psi])
    curr = 0
    start_time = 0
    is_first = True
    if trav_path:
        prev_x = [d.com[0]]
        prev_y = [d.com[1]]
        prev_z = [d.com[2]]
    if wp_path:
        wp_s = [[d.com[0],d.com[1],d.com[2],d.psi]] + waypoints
    while(1):
        plt.cla()
        if wp_path:
            ax.quiver(np.array(wp_s)[:-1,0],np.array(wp_s)[:-1,1],np.array(wp_s)[:-1,2],
                    np.array(wp_s)[1:,0]-np.array(wp_s)[:-1,0],
                    np.array(wp_s)[1:,1]-np.array(wp_s)[:-1,1],
                    np.array(wp_s)[1:,2]-np.array(wp_s)[:-1,2], 
                    length = 0.5, normalize = True)
            ax.plot(np.array(wp_s)[:,0],np.array(wp_s)[:,1],np.array(wp_s)[:,2], "--")
        if reached(waypoints[curr],d.state[0],d.state[1],d.state[2],d.state[8],max_error_coord,max_error_yaw):
            if is_first:
                start_time = time.time()
                is_first = False
            else:
                if(time.time()-start_time > wait_time):
                    if curr<len(waypoints):
                        curr+=1
                        is_first = True
        if curr == len(waypoints):
            break
        d.controller_update(waypoints[curr][0],waypoints[curr][1],waypoints[curr][2],waypoints[curr][3])
        l = d.dynamics_euler_angles()
        for i in range(5):
            ax.text(float(l[3*i]),float(l[3*i+1]),float(l[3*i+2]),labels[i], fontsize = 7)
        for i in range(1,5):
            x = np.array([l[0], l[3*i]])
            y = np.array([l[1], l[3*i+1]])
            z = np.array([l[2], l[3*i+2]])
            ax.plot3D(x, y, z, linewidth=2, color = colors[i])
        if trav_path:
            prev_x.append(l[0])
            prev_y.append(l[1])
            prev_z.append(l[2])
            ax.plot(prev_x, prev_y, prev_z)
        if ori:
            point,dir_vec = d.get_orientation_vector()
            ax.quiver(point[0],point[1],point[2],dir_vec[0],dir_vec[1],dir_vec[2],normalize = True,length=0.2)
        ax.set_xlim(-2.0, 2.0)
        ax.set_ylim(-2.0, 2.0)
        ax.set_zlim(0, 3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        fig.canvas.draw()
        plt.pause(0.01)