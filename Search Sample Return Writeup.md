## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

I teach high school robotics and we have 5 design principles that use when we make robots for FIRST competitions.  The 5 principles are: reliable, simple, fast, adaptable, and continuous improvement. You will see these principles guiding my approach to this project.
 
What I am submitting reliably archives ~68% mapped / >73% fidelity / pickup 3-4 rocks / in ~413 sec (was not able how to efficiently make it back to home).  From what I see from others videos, my robot goes almost twice the speed of others. It is a balance between the challenge goals and how fast it can be done. I was able to achieve the 40% mapped / 60% fidelity / find 1 rock by the due date but did not want to submit until I went as far as I could go with the 100% mapped / 60% fidelity / pickup 6 rocks / return home. This is an example of continuous improvement - I kept working on the project as long as I was making significant improvements. It is also an example of adaptible, meaning it does tries to address most of the possible tasks. Later in the write-up, I will address a specific example how I kept it simple. 
 
This is my first NanoDegree and the fact that I have been out of college for more than 20 years (when online college didn’t exist) so I’m a little rusty. I’m looking to refresh my skills in a more formal way than DIY projects. Any feedback is appreciated.


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
I used notebook input cells 5 and 6 to develop rock identification. The threshold to find rocks could not be set by bounding just an upper limit.  I modified the color threshold function to filter between a range that bounded by a lower and upper limit (rgb_low, rgb_high). 
 
    def color_thresh(img, rgb_low, rgb_high):
          above_thresh = (img[:,:,0] > rgb_low[0]) \
        			& (img[:,:,1] < rgb_high[0]) \
                    & (img[:,:,1] > rgb_low[1]) \
                    & (img[:,:,1] < rgb_high[1]) \
                    & (img[:,:,2] > rgb_low[2]) \
                    & (img[:,:,2] < rgb_high[2])
 
Using the example_rock image as a test, I adjusted the values of RGB up and down to find just the rock. The function call shows the resulting narrow range.
 
    rock_threshed = color_thresh(rock_img, rgb_low=(120, 100, 0), rgb_high=(200, 200, 50))
 

 
Obstacle identification was determined with the rationale that if navigable path and sky are above the threshold of 160 for RGB, then anything below 159 must be an obstacle. 
 
    obs_threshed = color_thresh(warped, rgb_low=(0, 0, 0), rgb_high=(159, 159, 159))


![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
Using what was developed from the lessons and the previous notebook cells, I made the following changes to the process_image() in input cell 9.  The comments in the code explains what each does. Processing was done terrain to calculate where to drive, on obstacles to where to avoid, and on rocks to locate and drive to them and pick them up.
 
```
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],])
    
    # 2) Apply perspective transform
        # Let's create more images to add to the mosaic, first a warped image
    warped = perspect_transform(img, source, destination)
        # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped
 
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples    
    obs_threshed = color_thresh(warped, rgb_low=(0, 0, 0), rgb_high=(159, 159, 159))
    rock_threshed = color_thresh(rock_warped, rgb_low=(120, 100, 0), rgb_high=(200, 200, 50))
    threshed = color_thresh(warped, rgb_low=(160, 160, 160), rgb_high=(255, 255, 255))
 
    # 4) Convert thresholded image pixel values to rover-centric coords
    obs_xpix, obs_ypix = rover_coords(obs_threshed)
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
    xpix, ypix = rover_coords(threshed)
 
    # 5) Convert rover-centric pixel values to world coords 
    scale = 10
    # Get navigable pixel positions in world coords 
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, data.xpos[data.count],  
 data.ypos[data.count], data.yaw[data.count], 
 data.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, data.xpos[data.count], 
 data.ypos[data.count], data.yaw[data.count], 
 data.worldmap.shape[0], scale)
    x_world, y_world = pix_to_world(xpix, ypix, data.xpos[data.count], data.ypos[data.count], 
                                    	 data.yaw[data.count], data.worldmap.shape[0], scale)
 
    # 6) Update worldmap (to be displayed on right side of screen)
    data.worldmap[obs_y_world, obs_x_world, 0] += 1
    data.worldmap[rock_y_world, rock_x_world, 1] += 1
    data.worldmap[y_world, x_world, 2] += 1
    
    # 7) Make a mosaic image, below is some example code
        # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 2, data.ground_truth, .5, 0)   
        # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add) 
```


![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


The functions rotate_pix() and translate_pix() were completed from the notebook at the top of perception.py.
 
```
# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))               
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated
 
# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated
```
 
The perception_step() was first filled in with adding all tested code from notebook - transform, threshold, coordinates for vision image - rover - rock - obs - worldmap. This code has already been noted in the notebook section writeup. Then code was added to support the left vision image map.
 
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs_threshed*255   #RED
    Rover.vision_image[:,:,1] = rock_threshed*255   #GREEN
    Rover.vision_image[:,:,2] = threshed*255   #BLUE
 
 
 
To improve fidelity above 60%, the worldmap was limited to update only when roll and pitch where near 0. 
 
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if (Rover.roll < 0.1 or Rover.roll >359.9 or Rover.pitch < 0.1 or Rover.pitch >359.9):
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1     #RED
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1   #GREEN
        Rover.worldmap[y_world, x_world, 2] += 1             #BLUE
 
In the Slack forum, some students were able to develop great functions to morphological the pixel array or to limit the distance of returns. I found good results by just widening and lengthening the source by ~3% to improve fidelity, this condensed the far points limiting them from extending beyond the map. This is an example of simple design principle that I mentioned earlier.
 
    #original values source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    source = np.float32([[14, 140], [301 ,140],[205, 94], [123, 94]])
 
Last I calculated the polar coordinates for rock and obstacle. This enable the rover to be able steer to the rocks when discovered.
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.obs_dist, Rover.obs_angles = to_polar_coords(obs_xpix, obs_ypix)
    Rover.rock_dist, Rover.rock_angles = to_polar_coords(rock_xpix, rock_ypix)
    Rover.nav_dist, Rover.nav_angles = to_polar_coords(xpix, ypix)
 
I made a few changes to drive_rover.py. I changed initial mode to ‘start and added Rover.start_pos so it can be stored.  Also Rover.obs_angles & Rover.obs_dist and Rover.rock_angles & Rover.rock_dist were added to support finding and picking up rocks. I increased the Rover.max_vel from 2 to 4 and increased Rover.throttle_set from 0.2 to 0.3 to improve overall time. Due to the increased speed, I had to increased Rover.stop_forward from 50 to 75 and decreased Rover.go_forward from 500 to 450 for better performance.
 
Following is a description of the changes to decision_step() to add three new capabilities:
Added more states - start, rockin, FINISHED
4WD when stuck
Pickup rocks
 
Added ‘start’ mode to record initial position and start the rover moving. When all 6 rocks are found the mode is set to ‘FINISHED’.
```
        if Rover.mode == 'start':
            Rover.start_pos = Rover.pos
            Rover.throttle == Rover.throttle_set 
            Rover.mode = 'forward'
            
 
        elif Rover.mode == 'forward':                  
            if Rover.samples_found == 6:
                Rover.mode = 'FINISHED'
 ```           
 
Check if rover is stuck and 4 wheel drive turn.
```
                if Rover.vel <= 0.05 and Rover.throttle != 0:
                    print('@@@@@@@@@ STUCK  @@@@@@@')
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # 4-wheel turning
                    Rover.steer = 15 #
                elif Rover.vel < Rover.max_vel:
 ```
Search for rocks and change mode to ‘rockin’ if found.

                if len(Rover.rock_dist) > 0 : # or (Rover.rock_angles is not None):
                    #print('rockin called',len(Rover.rock_dist)) 
                    Rover.mode = 'rockin'      
 
Added Pickup rocks mode. #THROTTLE section controls speed to advance at a slower speed for better control. #TURNING section steers rover to rock. If rock is passed up, 4 wheel drive turn until rock is found or until mapped area is open and return to ‘forward’ mode. #PICKING section stops and loads the rock and then returns to ‘forward’ mode when complete.
```
        #Go to and Pickup rocks
        elif Rover.mode == 'rockin':
            #THROTTLE
            if Rover.vel > 2:  
                Rover.throttle = 0
                Rover.brake = Rover.brake_set*0.5 #slow not stop
            else:
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set*0.5 #half speed
            #TURNING
            if len(Rover.rock_dist) > 0: #prevent sending an empty array causing telemetry to freeze
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = 15  #TURN TILL FOUND OR GO FORWARD IF OPEN AREA
                if len(Rover.nav_angles) >= Rover.go_forward*1.5:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.mode = 'forward'
            #PICKING
            if Rover.near_sample and not Rover.picking_up:
                Rover.throttle = 0
                Rover.steer = 0
                Rover.brake = Rover.brake_set
                if Rover.vel == 0:
                    Rover.send_pickup = True
            if Rover.picking_up and not Rover.near_sample:
                Rover.send_pickup = False
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set
                Rover.mode = 'forward'
   
#FINISHED mode is for future development to return back to starting position.         
        #Future Return to starting position
        elif Rover.mode == 'FINISHED':
            Rover.brake = Rover.brake_set
            Rover.steer = 0
```


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

A sampling of 7 runs gives me an average of ~68% mapped / >73% fidelity / pickup 3-4 rocks / in ~413 sec
 
Run #1 ~70.3% mapped / >75.9% fidelity / pickup 5 rocks / in ~490 sec
Run #2 ~57.9% mapped / >75.8% fidelity / pickup 3 rocks / in ~280 sec 
Run #3 ~79.7% mapped / >72.3% fidelity / pickup 6 rocks / in ~527 sec  
Run #4 ~45.5% mapped / >80.2% fidelity / pickup  1 rocks / in ~160 sec
Run #5 ~45.7% mapped / >75.4% fidelity / pickup 1 rock / in ~269 sec 
Run #6 ~84.7% mapped / >54.1% fidelity / pickup 5 rocks / in ~744 sec
Run #7 ~92.3% mapped / >79.4% fidelity / pickup  4 rocks / in ~418 sec 
 
 

@FPS: 31-49
 
The areas I would like to make improvements are following closer along wall, searching better in nooks, eliminate repeating areas of the map, and to return home when finished. After submitting, I'm looking forward to hearing from the panel of students their solutions as well as DM some students to learn their approach.


![alt text][image3]



