import numpy as np
import math 


# will accelerate to a predefined speed 
def drive_rover(Rover, vel, steer_deg):

    vel_set = np.clip(vel, 0, Rover.max_vel)
    Rover.brake = 0

    if Rover.vel < vel_set:
        Rover.throttle = Rover.throttle_set # Set throttle value to throttle setting
    elif Rover.vel > vel_set:
        Rover.throttle = -Rover.throttle_set 

    else:
        Rover.throttle = 0 # Coast 

    Rover.steer = np.clip(np.mean(steer_deg), -15, 15)

def stop_rover(Rover, brake_set):
    Rover.throttle = 0
    Rover.brake = brake_set
    Rover.steer = 0

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    
    # Check if we have vision data to make decisions with
    if Rover.n_pix_front is not None:
        # Check for Rover.mode status
        #print(len(Rover.n_pix_front))
        if Rover.mode == 'forward': 
            # go to align mode when a rock is seen
            if Rover.seeing_rock is True and Rover.rock_dist < 100:
                    Rover.mode = 'align'
            
            elif len(Rover.n_pix_front) > Rover.go_forward:
                nav_angles_deg = np.mean(Rover.nav_angles)*180/np.pi
                # Go a little faster if area ahead is big enougth
                if (abs(nav_angles_deg) < Rover.nav_ang_thres and np.mean(Rover.nav_dists) > Rover.nav_dis_thres):
                    drive_rover(Rover, Rover.max_vel, nav_angles_deg)
                # Otherwire, reduce speed
                else:
                    drive_rover(Rover, Rover.slow_vel, nav_angles_deg)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.n_pix_front) < Rover.stop_forward:
                    stop_rover(Rover, Rover.brake_set)
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop' or Rover.mode == 'align':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                stop_rover(Rover, Rover.brake_set)

            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2 and Rover.mode != 'align':
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.n_pix_front) < Rover.stop_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.n_pix_front) > Rover.go_forward:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
            elif Rover.mode == 'align':
                Rover.throttle = 0
                Rover.brake = 0
                
                if Rover.seeing_rock is True:
                    rock_ang_deg = Rover.rock_angle * 180/np.pi
                    Rover.steer = rock_ang_deg 
                    print("rock ang:" + str(rock_ang_deg))
                    if abs(rock_ang_deg) < 60.0:
                        Rover.mode = 'collect'   
                else:
                    Rover.steer = -10

        elif Rover.mode == 'collect':
            
            if Rover.seeing_rock is True:

                if Rover.rock_dist > 10:
                    Rover.steer = Rover.rock_angle * 180/np.pi
                    Rover.throttle = Rover.throttle_set / 2.0
                else:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                
            else:
                Rover.mode = 'forward' 


    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

