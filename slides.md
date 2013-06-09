Eye-Tracking Robotic Arm
========================

University of Leicester - Bioengineering Group

By James Reuss, Will Scott-Jackson, Amy Lymn, Joseph Ahuja & Ian Chapple


Contents
========

- Introduction
- Mechanical
- Electrical
- Control
	- Eye-Tracking
	- Object Detection
- Project Execution


Introduction
============

> Amy


Mechanical
==========

> Amy
> Joe


Electrical
==========

> Will


Control - Eye-Tracking
======================

> Ian


Control - Object Detection
==========================

Why is Object Detection Needed?
-------------------------------

At this point the robotic arm system is ready for use, however it is unable to detect its environment.
The system having the ability to detect its environment will aid it in many ways:
- It can determine if the user is looking at an object, the table surface or elsewhere.
- It can determine if a less direct route is required for moving to avoid an object.
- It can help self calibrate the arm position.

There are many different systems that implement this. For example, the method shown uses an array of infra-red emitters and cameras which can detect the position in 3D of reflective balls attached to the quadcopters.
![KMel Robotics Quadrotors & IR](kmel.jpg)

Why use Kinect?
---------------

However, these systems are very expensive and are not ideal for use in this system.

A more affordable 3D vision system is the Xbox Kinect sensor which was released in 2010 for around £100.

How does the Kinect work?
-------------------------

![Kinect with labels](img/kinectlabel.png)

The Kinect has a regular RGB camera which just detects the coloured light.
There is also an infra-red projector which projects a pseudo-random laser pattern on to the scene, the infra-red camera then detects this pattern and onboard electronics compares it to a reference, finally producing a depth image.

Both the RGB and depth images have a resolution of 640 by 480 pixels.

The images also have an organised structure which allows for improved processing efficiency. This organisation can be seen here.

The organisation means that you know the location of neighbouring pixels within the data without performing heavy algorithms.

Both the RGB and Depth images are structured in this way. It is also possible to get the depth data in millimetre units.

Algorithms Required to Detect Objects
-------------------------------------

To actually convert this depth image into a useable form

- Surface Normals
- Plane Segmentation
	- Point Clustering
	- Cluster Filtering
	- Dominant Plane
		- RANSAC
		- Convex Hull
- Point Inclusion
- K-Means Clustering - Object Cloud Clustering

*Overview*

Detailed Explanation of Algorithms
----------------------------------

In detail... foreach ... with photos of output ... and results/effectiveness of computation? ... how i made it better?

Final Results
-------------
 ... video ... time taken to calculate

Improvements & Future Work
--------------------------



Project Execution
=================

> Someone




