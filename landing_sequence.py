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
import keyboard

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

if len(sys.argv) > 1:
    URI = sys.argv[1]

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

def ascending_step():
    if (log_pos[-3][2]-log_pos[-1][2])> 0.05:
        print('ascending step')
        return True
    else:
        return False

def descending_step():
    if (log_pos[-3][2]-log_pos[-1][2])> 0.05:
        print('descending step')
        return True
    else:
        return False

def landing_sequence(vel_x,vel_y):
    print('landing sequence')
    VELOCITY = 0.5
    epsilon = 0.01

    # look for mid pos on first direction (pos_m1)
    motion_commander.start_linear_motion(vel_x, vel_y, 0)
    pos_1 = [log_pos[-1][0],log_pos[-1][1]]
    while not descending_step():
        #keep flying
        time.sleep(0.1)
    pos_2 = [log_pos[-1][0],log_pos[-1][1]]
    pos_m1 = [(pos_1[0]-pos_2[0])/2,(pos_1[1]-pos_2[1])/2]

    #go to pos_m1
    motion_commander.start_linear_motion(-vel_x, -vel_y, 0)
    while ((log_pos[-1][0]-pos_m1[0])**2+(log_pos[-1][1]-pos_m1[1])**2)**0.5 > epsilon:
        #keep flying
        time.sleep(0.1)

    # look for mid pos on second direction (pos_m2)
    motion_commander.start_linear_motion(-vel_y, vel_x, 0)
    while not descending_step():
        #keep flying
        time.sleep(0.1)
    pos_1 = [log_pos[-1][0],log_pos[-1][1]]
    motion_commander.start_linear_motion(-vel_x, -vel_y, 0)
    while not descending_step():
        #keep flying
        time.sleep(0.1)
    pos_2 = [log_pos[-1][0],log_pos[-1][1]]
    pos_m2 = [(pos_1[0]-pos_2[0])/2,(pos_1[1]-pos_2[1])/2]

    motion_commander.start_linear_motion(-vel_x, -vel_y, 0)
    while ((log_pos[-1][0]-pos_m2[0])**2+(log_pos[-1][1]-pos_m2[1])**2)**0.5 > epsilon:
        #keep flying
        time.sleep(0.1)
    motion_commander.start_linear_motion(0, 0, 0)
    motion_commander.land()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')

    # Connect callbacks from the Crazyflie API
    #cf.connected.add_callback(connected)
    #cf.disconnected.add_callback(self.disconnected)
    #lpos = connected(URI)

    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multi_ranger:

                keep_flying = True
                state = "goal following"
                print("switched to" + state)
                start_position_printing(scf)
                epsilon = 0.01
                # current velocity
                velocity_x = 0.0
                velocity_y = 0.0

                while keep_flying:

                    landing_state = 0

                    #print(state)
                    if keyboard.read_key() == "c":
                        sys.exit()

                    if state == "obstacle avoidance":
                        #print(log_pos[-1]) #[0]) len(log_pos)
                        VELOCITY = 0.5

                        if is_close(multi_ranger.front):
                            velocity_x = 0.0
                            velocity_y = -VELOCITY

                        if is_close(multi_ranger.back):
                            velocity_x = VELOCITY

                        if is_close(multi_ranger.left):
                            velocity_y = -VELOCITY
                        if is_close(multi_ranger.right):
                            velocity_y = VELOCITY

                        if not is_close(multi_ranger.front):
                            if log_pos[-1][0] > 1.0:
                                state = "grid search"
                                print("switched to" + state)
                            else:
                                state = "goal following"
                                print("switched to" + state)


                    if state == "goal following":
                        velocity_x = 0.5
                        velocity_y = 0.0
                        if log_pos[-1][0] > 1.0:
                            state = "grid search"
                            print("switched to" + state)

                        if is_close(multi_ranger.front) or is_close(multi_ranger.left) or is_close(multi_ranger.back) or is_close(multi_ranger.right):
                            state = "obstacle avoidance"
                            print("switched to" + state)

                    if state == "grid search":
                        #print("grid search")
                        keep_flying = False

                    if is_close(multi_ranger.up):
                        keep_flying = False

                    if (log_pos[-3][2]-log_pos[-1][2])> 0.05 or state == "landing":
                        #motion_commander.land()
                        #landing_sequence(velocity_x,velocity_y)

                        if landing_state == 0:
                            print('base detectée')
                            state = "landing"
                            landing_state = 1
                            pos_1 = [log_pos[-1][0],log_pos[-1][1]]

                        if landing_state == 1:
                            # look for mid pos on first direction (pos_m1)
                            #motion_commander.start_linear_motion(vel_x, vel_y, 0)
                            if descending_step():
                                pos_2 = [log_pos[-1][0],log_pos[-1][1]]
                                pos_m1 = [(pos_1[0]-pos_2[0])/2,(pos_1[1]-pos_2[1])/2]
                                landing_state = 2
                                #motion_commander.start_linear_motion(-vel_x, -vel_y, 0)
                                velocity_y = -velocity_x
                                velocity_y = -velocity_y

                        #go to pos_m1
                        if landing_state == 1:
                            if ((log_pos[-1][0]-pos_m1[0])**2+(log_pos[-1][1]-pos_m1[1])**2)**0.5 > epsilon:
                                landing_state = 3
                                #motion_commander.start_linear_motion(-vel_x, vel_y, 0)
                                v_temp = velocity_x
                                velocity_x = -velocity_y
                                velocity_y = v_temp


                        # look for mid pos on second direction (pos_m2)
                        if landing_state == 3:
                            if descending_step():
                                pos_1 = [log_pos[-1][0],log_pos[-1][1]]
                                #motion_commander.start_linear_motion(vel_x, -vel_y, 0)
                                velocity_x = -velocity_x
                                velocity_y = -velocity_y
                                landing_state = 4

                        if landing_state == 4:
                            if descending_step():
                                pos_2 = [log_pos[-1][0],log_pos[-1][1]]
                                pos_m2 = [(pos_1[0]-pos_2[0])/2,(pos_1[1]-pos_2[1])/2]
                                #motion_commander.start_linear_motion(-vel_x, vel_y, 0)
                                velocity_x = -velocity_x
                                velocity_y = -velocity_y
                                landing_state = 5

                        if landing_state == 5:
                            if ((log_pos[-1][0]-pos_m2[0])**2+(log_pos[-1][1]-pos_m2[1])**2)**0.5 > epsilon:
                                #motion_commander.start_linear_motion(0, 0, 0)
                                velocity_x = 0.0
                                velocity_y = 0.0
                                motion_commander.land()
                                landing_state = 0
                                keep_flying = False
                                state = 'landed'

                    print('z: ', log_pos[-1][2])
                    print('x: ', log_pos[-1][0])


                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)

                    time.sleep(0.1)


            print('Demo terminated!')