import numpy as np
import cv2

# Identify pixels between threshhold upper and lower limits
def color_thresh(img, rgb_limits):
    rgb_min, rgb_max = rgb_limits
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be between both upper and lower threshold
    # tuples on all three threshold values in RGB.
    # within_thresh_values will now contain a boolean array with "True"
    # where threshold was met
    within_thresh_limits = (img[:,:,0] > rgb_min[0]) \
                & (img[:,:,1] > rgb_min[1]) \
                & (img[:,:,2] > rgb_min[2]) \
                & (img[:,:,0] < rgb_max[0]) \
                & (img[:,:,1] < rgb_max[1]) \
                & (img[:,:,2] < rgb_max[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[within_thresh_limits] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    dist, angles = [None, None]
    if len(x_pixel)>0:
        # Convert (x_pixel, y_pixel) to (distance, angle) 
        # in polar coordinates in rover space
        # Calculate distance to each pixel
        dist = np.sqrt(x_pixel**2 + y_pixel**2)
        # Calculate angle away from vertical for each pixel
        angles = np.degrees(np.arctan2(y_pixel, x_pixel))
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw = yaw*np.pi/180
    # Apply rotation to rover coordinates
    xpix_rotated = xpix*np.cos(yaw) - ypix*np.sin(yaw)
    ypix_rotated = xpix*np.sin(yaw) + ypix*np.cos(yaw)
    
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Scale and translate image pixels to rover coords
    xpix_translated = np.int_(xpos + xpix_rot/scale)
    ypix_translated = np.int_(ypos + ypix_rot/scale)
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Define a function to determine if roll/pitch values valid for updating map
def valid_orientation(pitch,roll):
    pitch_thresh = .5
    roll_thresh = .5
    if pitch<360-pitch_thresh and pitch>pitch_thresh:
        return False
    if roll<360-roll_thresh and roll>roll_thresh:
        return False
    return True

# Define a function to update worldmap given trust radius
def trusted(world_y, world_x, dists, trust_radius):
    if dists is not None:
        world_y = world_y[dists < trust_radius]
        world_x = world_x[dists < trust_radius]
    else:
        print('dists == None')
    
    return world_y, world_x       

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
        # Define calibration box in source (actual) and destination (desired) coordinates
        # These source and destination points are defined to warp the image
        # to a grid where each 10x10 pixel square represents 1 square meter
        # The destination box will be 2*dst_size on each side
    dst_size = 5 
        # Set a bottom offset to account for the fact that the bottom of the image 
        # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
        [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
        [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
        [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
        ])
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
        #initialize threshholds for various targets
    ground_thresh = ((160, 160, 160),(256,256,256))
    rock_thresh = ((90,80,0), (210,210,70))
    obstacle_thresh = ((-1,-1,-1), (110,110,110))
    
        #get threshholded images
    threshed_ground = color_thresh(Rover.img, ground_thresh)
    threshed_rock = color_thresh(Rover.img, rock_thresh)
    threshed_obstacle = color_thresh(Rover.img, obstacle_thresh)
        #eliminate overlap on segmented obstacle and rock images
    threshed_rock[threshed_obstacle==1]=0
    
    # 2) Apply perspective transform
        #warp threshholded images
    warped_ground = perspect_transform(threshed_ground, source, destination)
    warped_rock = perspect_transform(threshed_rock, source, destination)
    warped_obstacle = perspect_transform(threshed_obstacle, source, destination)

    # 5) Convert map image pixel values to rover-centric coords
    ground_rover_x, ground_rover_y = rover_coords(warped_ground)
    rock_rover_x, rock_rover_y = rover_coords(warped_rock)
    obstacle_rover_x, obstacle_rover_y = rover_coords(warped_obstacle)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10 #scaling factor from rover coord space to global map space (e.g. 10m square maps to 1m square)
    args = (Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.img.shape[0], scale)
    ground_world_x, ground_world_y = pix_to_world(ground_rover_x, ground_rover_y, *args)
    rock_world_x, rock_world_y = pix_to_world(rock_rover_x, rock_rover_y, *args)
    obstacle_world_x, obstacle_world_y = pix_to_world(obstacle_rover_x, obstacle_rover_y, *args)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles to rocks and navigable terrain
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(ground_rover_x, ground_rover_y)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_rover_x, rock_rover_y)
    Rover.obstacle_dists, Rover.obstacle_angles = to_polar_coords(obstacle_rover_x, obstacle_rover_y)
    '''
        #Update Rover.vision_image (unwarped) with untrusted data, half intensity
    Rover.vision_image[:,:,0] = threshed_obstacle*125
    Rover.vision_image[:,:,1] = threshed_rock*125
    Rover.vision_image[:,:,2] = threshed_ground*125

        # apply trust radius to threshholded rover imagery for display on screen
    threshed_ground = trusted(threshed_ground[0], threshed_ground[1], Rover.nav_dists, Rover.trustR['nav'])
    threshed_obstacle = trusted(threshed_obstacle[0], threshed_obstacle[1], Rover.obstacle_dists, Rover.trustR['obs'])
    threshed_ground = trusted(threshed_rock[0], threshed_rock[1], Rover.rock_dists, Rover.trustR['rock'])
    '''
        #Update Rover.vision_image (unwarped) with trusted data, full intensity
    Rover.vision_image[:,:,0] = threshed_obstacle*255
    Rover.vision_image[:,:,1] = threshed_rock*255
    Rover.vision_image[:,:,2] = threshed_ground*255

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # only update if rover is in a valid orientation
    if valid_orientation(Rover.pitch, Rover.roll):
        obstacle_world_y, obstacle_world_x = trusted(obstacle_world_y, obstacle_world_x, Rover.obstacle_dists, Rover.trustR['obs'])
        ground_world_y, ground_world_x = trusted(ground_world_y, ground_world_x, Rover.nav_dists, Rover.trustR['nav'])
        rock_world_y, rock_world_x = trusted(rock_world_y, rock_world_x, Rover.rock_dists, Rover.trustR['rock'])

        #add pixels in worldmap for detected obstacles, rocks, and ground
        Rover.worldmap[obstacle_world_y, obstacle_world_x, 0] = 255
        Rover.worldmap[rock_world_y, rock_world_x, 1] = 255
        Rover.worldmap[ground_world_y, ground_world_x, 2] = 255

        #subtract overlapping pixels to favor more accurate mapping
        #remove obstacle pixels from navigable pixels
        Rover.worldmap[obstacle_world_y, obstacle_world_x, 2] = 0
        #remove ground pixels from obstacle pixels
        Rover.worldmap[ground_world_x, ground_world_y, 0] = 0


    # restrict nav dists, rock dists, obstacle dists used for navigation to trusted radius
    Rover.obstacle_dists, Rover.obstacle_angles = trusted(Rover.obstacle_dists, Rover.obstacle_angles, Rover.obstacle_dists, Rover.trustR['obs'])
    Rover.nav_dists, Rover.nav_angles = trusted(Rover.nav_dists, Rover.nav_angles, Rover.nav_dists, Rover.trustR['nav'])
    Rover.rock_dists, Rover.rock_angles = trusted(Rover.rock_dists, Rover.rock_angles, Rover.rock_dists, Rover.trustR['rock'])
    
    return Rover