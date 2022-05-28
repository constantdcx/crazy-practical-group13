import logging
import sys
import time
import numpy.linalg as la
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from enum import Enum

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander

URI = 'radio://0/80/2M/E7E7E7E713'
log_pos = [[0,0,0]]
timestep = 10 #ms
VELOCITY = 0.5
VELOCITY_LANDING = 0.3

# To change
HOME_BASE = np.array([0.25, 1.0]) # ???
LANDING_DIST = 1.5 # 3.5
WIDTH = 1.5 # 3.0

# Yasmine
pos_bf_obs = []
pos_ret = -1
go_front = -1
count = 0
dir_x = 0
dir_y = 0
a = 0
b = 0

class State(Enum):
    FORWARDS = 0
    OBSTACLE_AVOIDANCE = 1
    GRID_SEARCH = 2
    LANDING = 3
    RETURN = 4
    
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# To print and acess data
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    log_pos.append(np.array([x,y,z]))
    #for idx, i in enumerate(list(data)):
    #        self.logs[self.count][idx] = data[i]
    #print('pos: ({}, {}, {})'.format(x, y, z))
    
def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=timestep)
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

def security_check(range):
    MIN_DISTANCE = 0.6  # m
    if range is None:
        return -2
    if range > MIN_DISTANCE:
        return 0
    else:
        return 1



def print_position():
    time = len(log_pos) * (timestep * 1e-3)
    print(f'x: {log_pos[-1][0]:.3f}, y: {log_pos[-1][1]:.3f}, z: {log_pos[-1][2]:.3f}, time: {time:.3f}')

def print_multiranger(mr):
    print(f'front: {mr.front}, left: {mr.left}, \
        right: {mr.right}, back: {mr.back}, \
        up: {mr.up}, down: {mr.down}'
        )


def plot_z():
    end_time = len(log_pos) * (timestep * 1e-3)

    time = np.linspace(0.0, len(log_pos) * (timestep * 1e-3), len(log_pos))
    pos_array = np.array(log_pos)
    plt.figure()
    plt.plot(time, pos_array[:,2])
    plt.xlabel("time (s)")
    plt.ylabel("z [m]")
    plt.title("Drone altitude")


    pass

def plot_traj():
    pos_array = np.array(log_pos)
    plt.figure()
    plt.plot(pos_array[:,0]+HOME_BASE[0], pos_array[:,1]+HOME_BASE[1])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    #plt.xlim([0,5])
    ymin = 0
    ymax = 3
    #plt.ylim([ymin,ymax])
    plt.vlines([1.5,3.5], ymin=ymin, ymax=ymax, color='k')
    rect = Rectangle((0, 0), 5, 3, linewidth=1, edgecolor='k', facecolor='none')
    ax = plt.gca()
    ax.add_patch(rect)
    plt.title("Drone Trajectory")
    pass

delay = 30 # timesteps
base_threshold = 0.05 # m

def ascending_step():
    if len(log_pos) > delay:
        buffer = log_pos[-delay:][2]
        max = np.max(buffer)
        min = np.min(buffer)
        idx_min = np.argmin(buffer)
        idx_max = np.argmax(buffer)
        if (max - min > base_threshold) and idx_max < idx_min :
            print('ascending step')
            print("max = ", max, "min = ", min)
            return True
        else:
            return False

def descending_step():
    if len(log_pos) > delay:
        if (log_pos[-1][2]-log_pos[-delay][2]) > base_threshold:
            print('descending step')
            return True
        else:
            return False
        
def get_absolute_position():
    
    x = log_pos[-1][0] + HOME_BASE[0]
    y = log_pos[-1][1] + HOME_BASE[1]
    z = log_pos[-1][2]
    
    return np.array([x, y, z])

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

                length_sweep = 0.2
                width_land_zone = 0.8
                epsilon = 0.02 # landing sequence
                landing_step = 0
                landing_count = 0
                
                dir_x = 1
                dir_y = 0
                obstacle_step=0
                return_step = 0
                sweep_step = 0

                keep_flying = True
                state = State(0)
                prev_state = State(0)
                print("switched to " + state.name)
                start_position_printing(scf)

                while keep_flying:
                    #print("State : ", state.name)
                        
                    if security_check(mr.left) == 0: # we can go left, no obstacles found left
                        b = 1.0
                    if security_check(mr.right) == 0:
                        b = -1.0
                    if security_check(mr.back) == 0:
                        a = -1.0
                    if security_check(mr.front) == 0:
                        a = 1.0
                    
                    if state != State.OBSTACLE_AVOIDANCE:
                        if (is_close(mr.front) and dir_x>0) :
                            prev_state = state
                            state = State.OBSTACLE_AVOIDANCE
                            pos_bf_obs = np.array(log_pos[-1][0:2])
                            print('object detected front')

                        elif (is_close(mr.back) and dir_x<0) :
                            prev_state = state
                            state = State.OBSTACLE_AVOIDANCE
                            pos_bf_obs = np.array(log_pos[-1][0:2])
                            print('object detected back')

                        elif is_close(mr.left) and dir_y>0 :
                            prev_state = state
                            state = State.OBSTACLE_AVOIDANCE
                            pos_bf_obs = np.array(log_pos[-1][0:2])
                            print('object detected left')

                        elif is_close(mr.right) and dir_y<0 :
                            prev_state = state
                            state = State.OBSTACLE_AVOIDANCE
                            pos_bf_obs = np.array(log_pos[-1][0:2])
                            print('object detected right')
                    
                    if state == State.OBSTACLE_AVOIDANCE:
                        print("obstacle_step = ", obstacle_step)
                        
                        if obstacle_step == 0:
                            count = 0
                        
                            if (is_close(mr.front) and dir_x>0) :
                                velocity_x = 0
                                velocity_y = b*0.5
                                direction = 'f'

                            elif (is_close(mr.back) and dir_x<0) :
                                velocity_x = 0
                                velocity_y = b*0.5
                                direction = 'b'

                            elif is_close(mr.left) and dir_y>0 :
                                velocity_x = a*0.5
                                velocity_y = 0
                                direction = 'l'

                            elif is_close(mr.right) and dir_y<0 :
                                velocity_x = a*0.5
                                velocity_y = 0
                                direction = 'r'
                                
                            else: 
                                obstacle_step = 1
                        
                        if obstacle_step == 1:

                            if (direction == 'f' or direction == 'b') and ((security_check(mr.right) == 1 and b>0)or (security_check(mr.left) == 1 and b<0) or count <= 15):
                                print('avance')
                                if prev_state == State.GRID_SEARCH and get_absolute_position()[0] - pos_bf_obs[0] > length_sweep:
                                    state = State.GRID_SEARCH
                                    sweep_step += 1
                                else:
                                    velocity_x = 0.3*dir_x
                                    velocity_y = 0
                                    count +=1

                            elif (direction == 'l' or direction == 'r') and ((security_check(mr.front) == 1 and a<0) or (security_check(mr.back) == 1 and a>0) or count <= 15):
                                print('avance')
                                velocity_x = 0
                                velocity_y = 0.3*dir_y
                                count +=1
                            else:
                                obstacle_step = 2
                        
                        if obstacle_step == 2:
                            
                            dist_x = pos_bf_obs[0]-log_pos[-1][0]
                            dist_y = pos_bf_obs[1]-log_pos[-1][1]
                            print('dist_x:', dist_x, 'dist_y:', dist_y)
                            
                            if (direction =='f' or direction =='b') and abs(dist_y) >0.05:
                                print('tries to go back to position fb')
                                velocity_x = 0
                                velocity_y = np.sign(pos_bf_obs[1]-log_pos[-1][1])*0.3
                                print('velocity_x dist cond:', velocity_x, 'velocity_y', velocity_y)

                            elif (direction == 'l' or direction == 'r') and abs(dist_x) > 0.05:
                                print('tries to go back to position lr')
                                velocity_x = np.sign(pos_bf_obs[0]-log_pos[-1][0])*0.3
                                velocity_y = 0
                                print('velocity_x dist cond:', velocity_x, 'velocity_y', velocity_y)

                            else:
                                obstacle_step = 0
                                print('returned to initial place')

                                if log_pos[-1][0] > LANDING_DIST:
                                    state = State.GRID_SEARCH
                                    print("switched to" + state.name)
                                else:
                                    state = State.FORWARDS
                                    print("switched to" + state.name)

                    if state == State.FORWARDS:
                        velocity_x = dir_x * VELOCITY
                        velocity_y = 0.0
                        if log_pos[-1][0] > LANDING_DIST:
                            state = State.GRID_SEARCH
                            print("switched to" + state.name)

                    if state == State.GRID_SEARCH:
                        print("sweep_step ", sweep_step)
                        
                        # Go to right side of landing zone
                        if sweep_step == 0 :
                            velocity_x = 0
                            velocity_y = -VELOCITY
                            dir_x = 0
                            dir_y = -1
                            if get_absolute_position()[1] < epsilon:
                                sweep_step = 1
                        
                        # Go to left side
                        if sweep_step == 1 :
                            velocity_x=0
                            velocity_y=VELOCITY
                            dir_x = 0
                            dir_y = 1
                            
                            x0=get_absolute_position()[0]
                            if get_absolute_position()[1] > WIDTH : sweep_step=2

                        # Move forward 'length_sweep'
                        if sweep_step == 2 :
                            velocity_x=VELOCITY
                            velocity_y=0
                            dir_x = 1
                            dir_y = 0
                            if get_absolute_position()[0]-x0 > length_sweep : 
                                sweep_step=3
                                x0 = get_absolute_position()[0]

                        # Go to right side
                        if sweep_step==3 :
                            velocity_x=0
                            velocity_y=-VELOCITY
                            dir_x = 0
                            dir_y = -1
                            if get_absolute_position()[1] < 0 : sweep_step=4

                        # Move forward 'length_sweep' again
                        if sweep_step == 4 :
                            velocity_x=VELOCITY
                            velocity_y=0
                            dir_x = 1
                            dir_y = 0
                            if get_absolute_position()[0]-x0 > length_sweep : 
                                sweep_step=1
                                x0 = get_absolute_position()[0]

                        if ascending_step():
                            state = State.LANDING
                            print("switched to state ", state.name)

                    if state == State.RETURN:
                        print("return_step = ", return_step)
                        if return_step == 0:
                            #print('position : ', get_absolute_position())
                            velocity_x = - VELOCITY
                            velocity_y = 0
                            if abs(get_absolute_position([0]) - HOME_BASE[0]) < epsilon:
                                return_step = 1

                        if return_step == 1:
                            #print('position : ', get_absolute_position())
                            dist_y = get_absolute_position()[1] - HOME_BASE[1]
                            dir_x = 0
                            dir_y = - np.sign(dist_y)
                            velocity_x = 0
                            velocity_y = dir_y * VELOCITY
                            if abs(dist_y) < epsilon:
                                mc.land()
                                print("Youpi")
                                break
                        

                        """ 
                        kp = 0.5
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
                        """

                    if (state == State.LANDING):
                        #mc.land()
                        #break
                        print(f'landing_step = {landing_step}')
        
                        if landing_step == 0:
                            landing_step = 1
                            pos_1 = log_pos[-1][:2]
                            velocity_x /= 2.0
                            velocity_y /= 2.0

                        elif landing_step == 1:
                            # look for mid pos on first direction (pos_m1)
                            #motion_commander.start_linear_motion(vel_x, vel_y, 0)
                            if descending_step():
                                pos_2 = [log_pos[-1][0],log_pos[-1][1]]
                                pos_m1 = [(pos_1[0]+pos_2[0])/2,(pos_1[1]+pos_2[1])/2]
                                landing_step = 2
                                velocity_x = -velocity_x
                                velocity_y = -velocity_y
                                dir_x = -dir_x
                                dir_y = -dir_y

                        #go to pos_m1
                        elif landing_step == 2:
                            if ((log_pos[-1][0]-pos_m1[0])**2+(log_pos[-1][1]-pos_m1[1])**2)**0.5 < epsilon:
                                landing_step = 3
                                v_temp = velocity_x
                                velocity_x = -velocity_y
                                velocity_y = v_temp
                                dir_x = - dir_y
                                dir_y = dir_x

                        elif landing_step == 3:
                            offset = 0.3
                            vel_norm = la.norm([velocity_x, velocity_y])
                            offset_x = offset * velocity_x / vel_norm
                            offset_y = offset * velocity_y / vel_norm
                            pos_m1_offset = pos_m1 + [offset_x, offset_y]
                            print(pos_m1_offset)

                            if ((log_pos[-1][0]-pos_m1_offset[0])**2+(log_pos[-1][1]-pos_m1_offset[1])**2)**0.5 < epsilon:
                                landing_step = 4
                                velocity_x = -velocity_x
                                velocity_y = -velocity_y
                                dir_x = -dir_x
                                dir_y = -dir_y

                        # look for mid pos on second direction (pos_m2)
                        elif landing_step == 4:
                            if descending_step():
                                pos_1 = [log_pos[-1][0],log_pos[-1][1]]
                                velocity_x = -velocity_x
                                velocity_y = -velocity_y
                                dir_x = -dir_x
                                dir_y = -dir_y
                                landing_step = 5

                        elif landing_step == 5:
                            landing_count += 1
                            if landing_count > 8:
                                if descending_step():
                                    pos_2 = [log_pos[-1][0],log_pos[-1][1]]
                                    pos_m2 = [(pos_1[0]+pos_2[0])/2,(pos_1[1]+pos_2[1])/2]
                                    velocity_x = -velocity_x
                                    velocity_y = -velocity_y
                                    dir_x = -dir_x
                                    dir_y = -dir_y
                                    landing_step = 6

                        elif landing_step == 6:
                            if ((log_pos[-1][0]-pos_m2[0])**2+(log_pos[-1][1]-pos_m2[1])**2)**0.5 < epsilon:
                                velocity_x = 0.0
                                velocity_y = 0.0
                                mc.land()
                                landing_step = 0
                                landing_count = 0
                                print("Bravo Arthur !!")
                                time.sleep(5)
                                mc.take_off()
                                dir_x = -1
                                dir_y = 0
                                state = State.RETURN


                    # Emergency stop
                    if is_close(mr.up):
                        print("Emergency stop !")
                        mc.land()
                        break

                    # print_position()
                    # print_multiranger(mr)


                    time.sleep(0.1)
                    mc.start_linear_motion(velocity_x, velocity_y, 0)


            print('Finished !')

            plot_z()
            plot_traj()

            plt.show()