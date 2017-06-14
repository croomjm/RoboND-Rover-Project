import numpy as np

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
                Rover.stuckCounter += 1
                if Rover.stuckCounter > 5:
                    Rover.mode = 'unstick'
                    Rover.stuckCounter = 0
                    Rover.throttle = 0
                    Rover.steer = 0
                    Rover.brake = 0
                    Rover.action = 'Go to Unstick Mode'
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
                Rover.steer = np.clip(np.mean(Rover.nav_angles)-8, -15, 15)
                Rover.action = 'Driving Forward'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.action = 'Initialize Stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if Rover.stopStartTime is None:
                #record start of stop mode to check if stuck
                Rover.stopStartTime = Rover.total_time
                # If we're in stop mode but still moving keep braking

            #if stopped for too long, go to unstick mode
            if Rover.total_time - Rover.stopStartTime > 10:
                Rover.stopStartTime = None
                Rover.mode = 'unstick'
                Rover.action = 'Stopped and stuck!'

            if abs(Rover.vel) > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.action = 'Braking'
            elif abs(Rover.vel) <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                    Rover.action = 'Turn to Navigable Path'
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = 0
                    Rover.stopStartTime = None #reset since switching modes
                    Rover.mode = 'forward'
                    Rover.action = 'End Stop, Go Forward'
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

            if Rover.total_time - Rover.unstickStartTime > 20:
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