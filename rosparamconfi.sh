#!/bin/bash


rosparam set /original_image_name /stereo_camera/left/image_rect
rosparam set /image_mask_name /stereo_camera/free_map
rosparam list 
