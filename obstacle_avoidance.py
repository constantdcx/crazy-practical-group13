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
from turtle import width

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

def sweep(inlandingzone):
    distx=0
    disty=0
    width_land_zone=1
    length=0.5
    if inlandingzone :
        for i in range(2) :
            while dist < width_land_zone :
                motion_commander.start_linear_motion(0, 0.5, 0)
                distx = log_pos[-1][0]-log_pos[0][0]
            motion_commander.start_linear_motion(0.5, 0, 0)
        motion_commander.stop()
        motion_commander.land()
    else : 
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

                while keep_flying:
                    #print(state)
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

                    if (log_pos[-3][2]-log_pos[-1][2])> 0.05:
                        print('base detectée')
                        motion_commander.land()
                        keep_flying = False

                    print('z: ', log_pos[-1][2])
                    print('x: ', log_pos[-1][0])


                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)

                    time.sleep(0.1)


            print('Demo terminated!')