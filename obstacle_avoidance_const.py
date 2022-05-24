"""
Example script that allows a user to "push" the Crazyflie 2.X around
using your hands while it's hovering.
This examples uses the Flow and Multi-ranger decks to measure distances
in all directions and tries to keep away from anything that comes closer
than 0.2m by setting a velocity in the opposite direction.
The demo is ended by either pressing Ctrl-C or by holding your hand above the
Crazyflie.
"""
import logging
import sys
import time
import numpy.linalg as la
import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander

URI = 'radio://0/80/2M/E7E7E7E713'
log_pos = [[0,0,0], [0,0,0], [0,0,0]]
time_step = 100 #ms
VELOCITY = 0.3
LANDING_DIST = 3.5
    
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# To print and acess data
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    log_pos.append([x,y,z])
    #for idx, i in enumerate(list(data)):
    #        self.logs[self.count][idx] = data[i]
    #print('pos: ({}, {}, {})'.format(x, y, z))
    
def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=time_step)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

    
            
def is_close(range):
    MIN_DISTANCE = 0.4  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def sweep(inlandingzone):
    distx=0
    disty=0
    width_land_zone=1
    length=0.5
    if inlandingzone :
        for i in range(2) :
            while distx < width_land_zone :
                mc.start_linear_motion(0, 0.2, 0)
                distx = log_pos[-1][0]-log_pos[0][0]
                print(log_pos[-1])
            mc.start_linear_motion(0.2, 0, 0)
            time.sleep(1)
        mc.stop()
        mc.land()
    else :
        mc.land()

def print_position():
    print(f'x: {log_pos[-1][0]:.3f}, y: {log_pos[-1][1]:.3f}, z: {log_pos[-1][2]:.3f}')

def print_multiranger(mr):
    print(f'front: {mr.front}, left: {mr.left}, \
        right: {mr.right}, back: {mr.back}, \
        up: {mr.up}, down: {mr.down}'
        )

def obstacle_contourning(mc, mr):

    safety_distance = 0.15
    #obstacle_velocity = 0.3
    start_y = log_pos[-1][1]

    # goes left until obstacle is not in front
    mc.start_linear_motion(0, VELOCITY, 0)
    while is_close(mr.front):
        time.sleep(0.1)

    # wait a bit more
    time.sleep(1.0)

    # compute lateral distance
    lateral_distance = abs(log_pos[-1][1] - start_y)
    if lateral_distance < 0.1:
        lateral_distance = 0.3
    print(lateral_distance)

    # goes forward until obstacle is not on right
    mc.start_linear_motion(VELOCITY, 0, 0)
    time.sleep(2.0)

    while is_close(mr.right):
        time.sleep(0.1)

    mc.right(lateral_distance, velocity=VELOCITY)
    time.sleep(1.0)
    
    pass

def plot_z():
    print(log_pos)
    pos_array = np.array(log_pos)
    print(pos_array)
    plt.figure()
    plt.plot(pos_array[:,2])
    plt.xlabel("time")
    plt.ylabel("z [m]")
    plt.title("Drone altitude")


    pass

def plot_traj():
    print(log_pos)
    pos_array = np.array(log_pos)
    print(pos_array)
    plt.figure()
    plt.plot(pos_array[:,0], pos_array[:,1])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.xlim([0,5])
    ymin = 0
    ymax = 3
    plt.ylim([ymin,ymax])
    plt.vlines([1.5,3.5], ymin=ymin, ymax=ymax)
    plt.title("Drone Trajectory")


    pass

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    
     # Connect callbacks from the Crazyflie API
    #cf.connected.add_callback(connected)
    #cf.disconnected.add_callback(self.disconnected)
    #lpos = connected(URI)
    
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf, default_height=0.5) as mc:
            with Multiranger(scf) as mr:

                time.sleep(0.1)

                keep_flying = True
                state = "goal following"
                print("switched to " + state)
                start_position_printing(scf)

                while keep_flying:

                    if state == "obstacle avoidance":

                        obstacle_contourning(mc, mr)

                        # if is_close(mr.front):
                        #     velocity_x = 0.0
                        #     velocity_y = -VELOCITY

                        # if is_close(mr.back):
                        #     velocity_x = VELOCITY

                        # if is_close(mr.left):
                        #     velocity_y = -VELOCITY
                        # if is_close(mr.right):
                        #     velocity_y = VELOCITY

                        if not is_close(mr.front):
                            if log_pos[-1][0] > LANDING_DIST:
                                state = "grid search"
                                print("switched to" + state)
                            else:
                                state = "goal following"
                                print("switched to" + state)



                    if state == "goal following":
                        velocity_x = VELOCITY
                        velocity_y = 0.0
                        if log_pos[-1][0] > LANDING_DIST:
                            state = "grid search"
                            print("switched to" + state)

                        if is_close(mr.front) or is_close(mr.left) or is_close(mr.back) or is_close(mr.right):
                            state = "obstacle avoidance"
                            print("switched to" + state)

                    if state == "grid search":
                        #print("grid search")
                        keep_flying = False
                        mc.land()
                        break

                    if state == "return":
                        print(f'position : {log_pos[-1]}')
                        kp = 0.5
                        home_position = [1.0, 0.5]
                        distx = home_position[0] - log_pos[-1][0]
                        disty = home_position[1] - log_pos[-1][1]
                        dist_norm = la.norm([distx, disty])
                        if dist_norm > 0.2:
                            velocity_x = kp * distx / dist_norm
                            velocity_y = kp * disty / dist_norm
                        else:
                            mc.land()
                            print("landed")
                            time.sleep(1)
                            print("taking off")
                            mc.take_off()

                    if is_close(mr.up):
                        keep_flying = False

                    # if (log_pos[-3][2]-log_pos[-1][2])> 0.05:
                    #     print('base detectée')
                    #     mc.land()
                    #     keep_flying = False

                    print_position()
                    # print_multiranger(mr)


                    time.sleep(0.1)
                    mc.start_linear_motion(
                        velocity_x, velocity_y, 0)


            print('Demo terminated!')

            plot_z()
            plot_traj()

            plt.show()