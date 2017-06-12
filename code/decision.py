import numpy as np



def get_steer_angle(Rover):
    max_steer_angle =  5 + 10*(1-np.clip(np.abs(Rover.vel)/Rover.max_vel, 0, 1))
    return int(max_steer_angle)

'''Abandoned helper functions for more complicated decision tree
def stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

    return Rover

def forward(Rover, throttle = None, steer_target = None):
    if throttle is None:
        throttle = Rover.throttle_set
    if steer_target is None:
        steer_target = np.mean(Rover.nav_angles[Rover.nav_dists < 10])
    max_steer = get_steer_angle(Rover)

    Rover.brake = 0
    Rover.throttle = (Rover.vel<Rover.max_vel)*throttle
    Rover.steer = np.clip(steer_target, -max_steer, max_steer)
    
    return Rover

def hard_turn(Rover, direction = -1, angle = 15):
    Rover.throttle = 0
    Rover.brake = 0
    Rover.steer = direction*angle

    return Rover

def back_up(Rover, throttle = None):
    if throttle is None:
        throttle = -Rover.throttle_set
    Rover.brake = 0
    Rover.throttle = throttle
    Rover.steer = 0

    return Rover
'''

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    print('Rover.samples_pos = ', Rover.samples_pos)
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    #example code:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            #if not moving even though throttle is set, move to unstick mode
            if Rover.vel == 0 and Rover.throttle > 0:
                Rover.mode = 'unstick'

            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if Rover.stopStartTime is None:
                #record start of stop mode to check if stuck
                Rover.stopStartTime = Rover.total_time
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles), -15, 15)
                    Rover.stopStartTime = None #reset since switching modes
                    Rover.mode = 'forward'
            #if stopped for too long, go to unstick mode
            if Rover.total_time - Rover.stopStartTime > 10:
                Rover.mode = 'unstick'
                Rover.action = 'Stopped and stuck!'
        
        elif Rover.mode == 'unstick':
            #initialization of mode
            if Rover.unstickStartPos is None:
                Rover.unstickStartPos = Rover.pos
                Rover.unstickStartYaw = Rover.yaw
                Rover.unstickStartTime = Rover.total_time
                Rover.action = 'Initialize Stuckness'
            #if double-stuck(!)
            elif Rover.vel == 0 and Rover.throttle != 0:
                Rover.throttle = 0
                Rover.brake = 0
                #turn hard left or right stochastically
                Rover.steer = -15#(np.random.random_sample()>0.5)*15
                Rover.action = 'Recover from Double Stuck!'
            #back up specified distance from startPos, then turn specified yaw difference
            else:
                #if not specified distance from stuck start position, back up
                if np.linalg.norm(np.subtract(Rover.pos,Rover.unstickStartPos))<0.5:
                    Rover.throttle = -0.2*(Rover.vel + 0.3 > 0) #accel only if vel >-0.2
                    Rover.brake = 0
                    Rover.steer = 0
                    Rover.action = 'Back that thing up!'
                #if not specified angle from start yaw, turn (right)
                elif Rover.unstickStartYaw - Rover.yaw < 10:
                    print(Rover.yaw)
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                    Rover.action = 'Turn Hard Right!'
                else:
                    Rover.throttle = 0
                    Rover.steer = 0
                    Rover.brake = 1.0
                    Rover.action = 'Resume Stop!'
                    Rover.mode = 'stop'
                    Rover.unstickStartYaw = None #reset since switching modes
                    Rover.unstickStartPos = None #reset since switching modes
                    Rover.unstickStartTime = 0

            if Rover.total_time - Rover.unstickStartTime > 15:
                #resume forward motion
                Rover.mode = 'stop'
                Rover.brake = 1.0
                Rover.steer = 0
                Rover.action = 'Unstick Timeout!'
                Rover.unstickStartYaw = None #reset since switching modes
                Rover.unstickStartPos = None #reset since switching modes
                Rover.unstickStartTime = 0


    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0
        # Release the brake to allow turning
        Rover.brake = 0
        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
        Rover.steer = -15
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover    

'''Abandoned code
# Example:
    if Rover.mode == 'acquire_sample':
        if Rover.near_sample:
            Rover = stop(Rover)
        #check if rock is no longer within view
        elif Rover.rock_dists is None or len(Rover.rock_dists)<=10 or np.min(Rover.rock_dists)>50:
            Rover.mode = 'forward'
            Rover = stop(Rover)
            Rover.action = 'Lost Sample Visual'
            Rover.send_pickup = False
        #if rock is in view move forward slowly
        elif np.mean(Rover.rock_angles) < 5 and np.mean(Rover.rock_dists) > 7:
            Rover = go_forward(Rover, throttle = 0.1, steer_target = np.mean(Rover.rock_angles))
            Rover.action = 'Approaching Sample'
        #otherwise, come to a stop, then turn toward rock
        else:
            Rover = hard_turn(Rover, direction = np.sign(np.mean(Rover.rock_angles)))
            Rover.action = 'Turning Toward Sample'
    # Check if we have vision data to make decisions with
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            #see if rock within view; if so, switch mode to acquire rock
            if Rover.rock_dists is not None and len(Rover.rock_dists)>10 and np.min(Rover.rock_dists)<50:
                Rover.mode = 'acquire_sample'
                Rover = stop(Rover)
                Rover.action = 'Stopping to Acquire Sample'
            #elif Rover.throttle != 0 and Rover.vel == 0:
            #    Rover = hard_turn(Rover, direction = np.sign(np.mean(Rover.nav_angles)))
            #    Rover.action = 'Turn Away from Obstacle'
            # Check the extent of navigable terrain
            elif len(Rover.nav_dists < 10) >= Rover.stop_forward: 
                # Set steering to average angle clipped to the range +/- max_steer
                Rover = go_forward(Rover, throttle = Rover.throttle_set)
                Rover.action = 'Continue'

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            # If attempting to move forward and not actually moving, then go to 'stop' mode
            else:
                Rover = stop(Rover)
                Rover.mode = 'stop'
                Rover.action = 'Stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover = stop(Rover)
                Rover.action = 'Brake'
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_dists < 10) < Rover.go_forward:
                    Rover = hard_turn(Rover)
                    Rover.action = 'Hard Turn'
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    Rover = go_forward(Rover, steer_target = np.mean(Rover.nav_angles[Rover.nav_dists<10]))
                    Rover.mode = 'forward'
                    Rover.action = 'Forward Resume'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover = hard_turn(Rover)
        Rover.action = 'No Nav Angles Nearby'
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.action = 'Picking Up Rock'

    return Rover
'''
'''Old code

    # Example:
    if Rover.mode == 'acquire_sample':
        if Rover.near_sample:
            if Rover.vel == 0:
                Rover.action = 'Picking Up Rock'
                Rover.send_pickup = True
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
        #check if rock is no longer within view
        elif Rover.rock_dists is None or len(Rover.rock_dists)<=20 or np.min(Rover.rock_dists)>50:
            Rover.mode = 'forward'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.action = 'Lost Sample Visual'
            Rover.send_pickup = False
        #if rock is in view move forward slowly
        elif np.mean(Rover.rock_angles) < 5 and np.mean(Rover.rock_dists) > 7:
            Rover.brake = 0
            if Rover.vel<.8:
                Rover.throttle = 0.2
            else:
                Rover.throttle = 0 #coast
            Rover.steer = np.mean(Rover.rock_angles)
            Rover.action = 'Approaching Sample'
        #otherwise, come to a stop, then turn toward rock
        else:
            Rover.brake = 0
            Rover.throttle = 0
            Rover.steer = np.sign(np.mean(Rover.rock_angles))*15
            Rover.action = 'Turning Toward Sample'
    # Check if we have vision data to make decisions with
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            #check if moving backwards for some reason
            if Rover.vel < 0:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.action = 'Stop Going Backwards'
            #see if rock within view; if so, switch mode to acquire rock
            elif Rover.rock_dists is not None and len(Rover.rock_dists)>20 and np.min(Rover.rock_dists)<50:
                Rover.mode = 'acquire_sample'
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.action = 'Stopping to Acquire Sample'
            # Check the extent of navigable terrain
            elif len(Rover.nav_dists < 8) >= Rover.stop_forward: 
                # Set steering to average angle clipped to the range +/- max_steer
                max_steer = get_steer_angle(Rover)
                 
                #Rover.throttle = (Rover.vel < Rover.max_vel)*Rover.throttle_set
                #Rover.brake = 0
                #Rover.steer = np.clip(np.mean(Rover.nav_angles[Rover.nav_dists<8]), -max_steer, max_steer)
                
                if np.sum(Rover.nav_dists <8) > 10:
                        angle = np.mean(Rover.nav_angles[Rover.nav_dists<8])
                        if np.abs(angle) < max_steer:
                            Rover.throttle = (Rover.vel < Rover.max_vel)*Rover.throttle_set
                            Rover.brake = 0
                            Rover.steer = angle
                        elif Rover.vel > 0.2:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                        else:
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = np.sign(angle)*15
                else:
                    #if there's no navigable terrain, turn
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                #Rover.steer = np.clip(Rover.obstacle_angles[np.argmin(Rover.obstacle_dists)]-20, -max_steer, max_steer)
                Rover.action = 'Continue'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            # If attempting to move forward and not actually moving, then go to 'stop' mode
            #else len(Rover.nav_dists < 8) < Rover.stop_forward:
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.action = 'Stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.action = 'Brake'
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_dists < 8) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    Rover.action = 'Hard Turn'
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_dists < 8) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    max_steer = get_steer_angle(Rover)
                    if np.sum(Rover.nav_dists <8) > 0:
                        Rover.steer = np.clip(np.max(Rover.nav_angles[Rover.nav_dists<8])-4, -max_steer, max_steer)
                    else:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles), -max_steer, max_steer)
                    #Rover.steer = np.clip(Rover.obstacle_angles[np.argmin(Rover.obstacle_dists)]-20, -max_steer, max_steer)
                    Rover.mode = 'forward'
                    Rover.action = 'Forward Resume'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = 0
        # Release the brake to allow turning
        Rover.brake = 0
        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
        Rover.steer = -15 # Could be more clever here about which way to turn
        Rover.action = 'Hard Turn'
        
    # If in a state where want to pickup a rock send pickup command
    #if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #    Rover.send_pickup = True

    return Rover
'''
