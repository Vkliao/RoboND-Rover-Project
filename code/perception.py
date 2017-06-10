import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_low, rgb_high):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_low[0]) \
                & (img[:,:,1] < rgb_high[0]) \
                & (img[:,:,1] > rgb_low[1]) \
                & (img[:,:,1] < rgb_high[1]) \
                & (img[:,:,2] > rgb_low[2]) \
                & (img[:,:,2] < rgb_high[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

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

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):        
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
        # Define calibration box in source (actual) and destination (desired) coordinates
        # These source and destination points are defined to warp the image
        # to a grid where each 10x10 pixel square represents 1 square meter
        # The destination box will be 2*dst_size on each side
    dst_size = 5 
        # Set a bottom offset to account for the fact that the bottom of the image 
        # is not the position of the rover but a bit in front of it
        # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    #source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    source = np.float32([[14, 140], [301 ,140],[205, 94], [123, 94]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obs_threshed = color_thresh(warped, rgb_low=(1, 1, 1), rgb_high=(80, 80, 80))
    rock_threshed = color_thresh(warped, rgb_low=(120, 100, 0), rgb_high=(200, 200, 50))
    threshed = color_thresh(warped, rgb_low=(160, 160, 160), rgb_high=(255, 255, 255))
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs_threshed*255   #RED
    Rover.vision_image[:,:,1] = rock_threshed*255   #GREEN
    Rover.vision_image[:,:,2] = threshed*255   #BLUE
    
    # 5) Convert map image pixel values to rover-centric coords
    obs_xpix, obs_ypix = rover_coords(obs_threshed)
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
    xpix, ypix = rover_coords(threshed)
    
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale) 
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale) 
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale) 
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if (Rover.roll < 0.1 or Rover.roll >359.9 or Rover.pitch < 0.1 or Rover.pitch >359.9):
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1     #RED
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1   #GREEN
        Rover.worldmap[y_world, x_world, 2] += 1             #BLUE
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.obs_dist, Rover.obs_angles = to_polar_coords(obs_xpix, obs_ypix)
    Rover.rock_dist, Rover.rock_angles = to_polar_coords(rock_xpix, rock_ypix)
    Rover.nav_dist, Rover.nav_angles = to_polar_coords(xpix, ypix)
   
    return Rover