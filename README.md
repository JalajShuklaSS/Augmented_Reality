# AR VR project through PnP and P3P Algorithm Implementation
## Overview
This project aims to demonstrate the use of computer vision techniques to create an augmented reality (AR) experience. Augmented reality seamlessly integrates virtual elements into the real-world environment, enhancing user perception and interaction.


![VR_AR](Result.gif)
## Project Components
## Homography Estimation
The est_homography function calculates the homography matrix H between two planes, enabling the transformation of 2D points in one plane to corresponding points in the other. This functionality is crucial for aligning virtual and real-world coordinates.

## Camera Pose Estimation
The PnP (Perspective-n-Point) algorithm is implemented in the PnP function. It estimates the camera pose using 2D-3D point correspondences, allowing us to determine the camera's orientation (R) and translation (t) in the world.

## P3P Solver
The P3P function utilizes the Perspective-3-Point algorithm to estimate camera pose based on correspondences. This function handles the case where three 3D points and their corresponding 2D projections are available.

Pixel-to-World Coordinate Estimation
The est_pixel_world function calculates the world coordinates of a point given its pixel coordinates, camera orientation, translation, and intrinsic matrix.

World Coordinate Estimation for April Tag Corners
The est_Pw function estimates the world coordinates of April tag corners, assuming a specific world setup.

## ImageClicker Class
The ImageClicker class facilitates interactive point selection in an image. This is useful for user-defined placement of virtual objects in the scene.

## Renderer Class
The Renderer class, adapted from the SMPLify-X project, provides rendering capabilities. It constructs a scene with virtual objects, a camera, and lighting, rendering the scene onto an image.

## Main Script
The main script orchestrates the entire AR process. It iterates through frames, estimates camera pose, renders virtual objects onto the real-world scene, and creates a GIF to showcase the AR experience.
