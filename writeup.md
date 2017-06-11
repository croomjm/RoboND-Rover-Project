## Project: Search and Sample Return 
[//]: # (Image References)
[image4]: ./misc/segmented_rock_img2.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis

Rather than enumerate the code blocks I used in the notebook to establish my technique in the test notebook, I've included detailed  explanations of each step of the final implementation of my autonomous rover code in the section below.

### Autonomous Navigation and Mapping

#### Perception Step Approach:
 
 Using the following method, I was able to achieve mapping accuracy at or near 90% for all test trials.
 
 _**tldr: I used RGB thresholds (upper and lower limits) to identify feature classes. I then used the suggested methods to convert to map pixels. I then navigated and updated the map only with pixels observed within a "trust radius" around the rover. I also made sure that any pixels marked as both obstacle and navigable terrain were ignored in map updates.**_

 ##### 1. Apply a color threshhold using `color_thresh()` for each feature type using both floor and ceiling values.
 
 I was able to tuples for each feature class that provided adequate segmentation without the need to explore alternate color spaces or more advanced segmentation methods. I chose the RGB threshholds based on a combination of trial and error using sample images along with sampling the RGB colors within sample images using the Mac OS digital color meter.
 
 ```python
 ground_thresh = ((160, 160, 160),(256,256,256))
 rock_thresh = ((90,80,0), (210,210,70))
 obstacle_thresh = ((-1,-1,-1), (110,110,110))
 
 def color_thresh(img, rgb_limits):
     rgb_min, rgb_max = rgb_limits
     # Create an array of zeros same xy size as img, but single channel
     color_select = np.zeros_like(img[:,:,0])
     # Require that each pixel be between both upper and lower threshold
     # tuples on all three threshold values in RGB.
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
 ```
 I applied each threshhold separately to obtain separate binary masks for each feature type.
 
 ```python
 threshed_ground = color_thresh(Rover.img, ground_thresh)
 threshed_rock = color_thresh(Rover.img, rock_thresh)
 threshed_obstacle = color_thresh(Rover.img, obstacle_thresh)
 ```
 
 
 ##### 2. Warp the threshholded image from camera space to physical space using `perspect_transform()`.
 
 I performed this step second so I wouldn't have to worry about having black pixels in the warped image being interpreted as obstacle pixels. Image warping was performed as suggested in the tutorial.
 
 ```python
 def perspect_transform(img, src, dst):       
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

        return warped
 ```
 
 I transformed each threshed binary array separately again, e.g.:
 ``` python
 warped_ground = perspect_transform(threshed_ground, source, destination)
 ```
 
 Sample threshholded image with warped perspective:
 ![Sample threshholded image][image4]
 
 ##### 3. Convert warped, threshholded pixels to rover-centric coordinates.
 
 This function was applied separately to each warped, threshholded, binary image array to translate image coords to rover coords.
 
 ```python
 def rover_coords(binary_img):
     # Identify nonzero pixels
     ypos, xpos = binary_img.nonzero()
     # Calculate pixel positions with reference to the rover position being at the 
     # center bottom of the image.  
     x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
     y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
     return x_pixel, y_pixel
 ```
 
 ```python
 ground_rover_x, ground_rover_y = rover_coords(warped_ground)
 rock_rover_x, rock_rover_y = rover_coords(warped_rock)
 obstacle_rover_x, obstacle_rover_y = rover_coords(warped_obstacle)
 ```
 
 ##### 4. Convert rover-centric cartesian coordinates to world coordinates.
 
 ```python
 # Define a function to apply rotation and translation (and clipping)
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
 ```
 
 ```python
 #Convert rover-centric pixel values to world coordinates
 scale = 10 #scaling factor from rover coord space to global map space (e.g. 10m square maps to 1m square)
 args = (Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.img.shape[0], scale)
 ground_world_x, ground_world_y = pix_to_world(ground_rover_x, ground_rover_y, *args)
 rock_world_x, rock_world_y = pix_to_world(rock_rover_x, rock_rover_y, *args)
 obstacle_world_x, obstacle_world_y = pix_to_world(obstacle_rover_x, obstacle_rover_y, *args)
 ```
 
 ##### 5. Convert rover-centric cartesian coordinates to rover-centric polar coordinates.
 
 Note that unlike the example code, I chose to go ahead and convert to degrees rather that in multiple places throughout the program.
 
 ```python
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
 ```
 ```python
 # Update Rover pixel distances and angles to rocks and navigable terrain
 Rover.nav_dists, Rover.nav_angles = to_polar_coords(ground_rover_x, ground_rover_y)
 Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_rover_x, rock_rover_y)
 Rover.obstacle_dists, Rover.obstacle_angles = to_polar_coords(obstacle_rover_x, obstacle_rover_y)
 ```
 
 ##### 6. Update the worldmap.
 
 Here, I applied a couple of techniques to get more accurate mapping information. First, I tested whether the rover was rolled or pitched too far to update the world map. I chose an arbitrary threshhold of 0.5 degrees in either pitch or roll. I chose an RSS combination of both pitch and roll to account for combinations of the two during high acceleration maneuvers.
 
 ```python
 def valid_orientation(pitch,roll):
    pitch_thresh = .5
    roll_thresh = .5
    #find square-sum threshhold
    SS_thresh = pitch_thresh**2 + roll_thresh**2

    #find distance from flat given angle range from 0 to 360
    pitch = min(360-pitch, pitch)
    roll = min(360-roll, roll)

    #determine if measured orientation is valid for map update
    if pitch**2 + roll**2 < SS_thresh:
        return True
    return False
 ```
 
 Next, I filtered the features that would be updated on the map based on whether they were within an arbitrarily defined "trust radius" from of the rover. Essentially, pixels farther away from the rover were deemed to be less accurate (e.g. due to horizon effects) and were thus discarded before updating the map.
 
 Based on testing, I used the following trust radii (defined within the Rover class). It's not particularly important to impose a trust radius on the rock pixels given that the simulation checks them against known rock locations, but the trust radius along with a cluster size metric would be useful supposing that information didn't exist. This is clearly the case for a real sample acquisition mission.
 
 ```python
 self.trustR = {'rock': 100, 'obs': 50, 'nav': 50} #define trust radii for all features
 ```
 
 ```python
 # Define a function to update worldmap given trust radius
 def trusted(world_y, world_x, dists, trust_radius):
    if dists is not None:
        world_y = world_y[dists < trust_radius]
        world_x = world_x[dists < trust_radius]
    else:
        print('dists == None')

    return world_y, world_x
 ```
 
 Finally, I updated the map with these parameters, and also made sure any pixels that were identified as both obstacle and navigable terrain were discarded before the map update.
 
 ```python
 if valid_orientation(Rover.pitch, Rover.roll):
    obstacle_world_y, obstacle_world_x = \
        trusted(obstacle_world_y, obstacle_world_x, Rover.obstacle_dists, Rover.trustR['obs'])
        
    ground_world_y, ground_world_x = \
        trusted(ground_world_y, ground_world_x, Rover.nav_dists, Rover.trustR['nav'])
        
    rock_world_y, rock_world_x = trusted(rock_world_y, rock_world_x, Rover.rock_dists, Rover.trustR['rock'])

    #add pixels in worldmap for detected obstacles, rocks, and ground
    Rover.worldmap[obstacle_world_y, obstacle_world_x, 0] = 255
    Rover.worldmap[rock_world_y, rock_world_x, 1] = 255
    Rover.worldmap[ground_world_y, ground_world_x, 2] = 255

    #subtract overlapping pixels to favor more accurate mapping
    #remove obstacle pixels from navigable pixels
    Rover.worldmap[obstacle_world_y, obstacle_world_x, 2] = 0
    #remove navigable pixels from obstacle pixels
    Rover.worldmap[ground_world_x, ground_world_y, 0] = 0
 ```
 
 ##### 7. Apply trust radius to nav angles and nav dists.
 
 I also filtered out the navigation information used in the decision loop according to the same trust radius used in the world map update step. This helped to prevent the rover from considering navigable terrain far in the distance when deciding the steering angle or what mode to choose. Note that this trust radius is not reflected in the segmented image shown in the bottom left corner of the rover test video.

#### Decision Step

_**tldr: I added additional behaviors to the decision step, but the branching if/else tree was too difficult to debug. I learned my lesson and will reasearch finite state machines for future work. The unmodified decision step code allowed me to map sufficient area with ~90% accuracy.**_

I attempted to make some modifications to the decision step file as follows:
 1. Modularize some of the typical behaviors (e.g. hard turn, stop, continue forward, etc.)
 2. Add additional case logic and modes for the rover in case it's stuck, sees a rock, etc.
 3. Limit steering angles and acceleration based on observed obstacles, navigable terrain, etc.
 4. Attempt to follow obstacle walls since rocks tend to be near them.

Unfortunately, the structure of the program became so needlessly complex that it was very difficult to debug and mentally model state flow. After viewing the live stream of the successful submissions, I realized that I wasn't the only one that found this approach untenable. As a result, I reverted to the original code and generated my final submission with that.

#### Mapping Results and Future Improvements

Here's a [video](https://youtu.be/5h85v-0rbQs) of a test run I performed. I tested the rover in 1024x640 with "Good" image quality.

If I were able to keep working on the project, I'd make a few changes:
 1. Make a solid finite state machine to manage the decision_step() state flow
 2. Add in logic for wall following, picking up rocks, backing up away once stuck, etc.
 3. Implement an algorithm to determined which areas still need to be explored. This could be done by looking for navigable terrain in the worldmap that is separated from obstacles or rocks by a distance larger than some sensitivity radius, then planning a path to that location using A\* search or similar.
 
My autonomous rover code is definitely sensitive to getting stuck in overhanging rock areas right now. To make the runs more repeatable, implementing code for backing up when stuck would be critical.

I also noticed a bug I wasn't able to diagnose in during the project. When navigating to the northern-most portion of the map, the worldmap stopped updating. I noticed this both in my test notebook and in my autonomous navigation code. I wanted to dig into this more, but I ran out of time.