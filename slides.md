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

Using the Kinect's built in calibration data it is possible to convert the pixel location and depth to a 3D point with all values in millimetres.

The collection of all of these 3D points is what forms the point cloud, as shown here with the RGB image mapped on top.

Algorithms Required to Detect Objects
-------------------------------------

To actually convert this depth image into a usable form and then find the objects, several operations must be performed on the data. These are:

- Surface Normals
- Plane Segmentation
	- Point Clustering
	- Cluster Filtering
	- Dominant Plane
		- RANSAC
		- Convex Hull
- Point Inclusion
- K-Means Clustering - Object Cloud Clustering

### Surface Normals

The first task is to calculate the surface normal for each point in the depth data. This requires determining the 3D vertices for each point.

Then passing the current point, with its neighbours, through the surface normal equation.

### Plane Segmentation

It is now possible to segment all of these points into their relative planes.

#### Point Clustering

This is done using tags. Then by iterating through each point, if the current point has no tag, it is assigned a new one, then a scan is made around this point for all the neighbouring points that have a normal angle difference between a certain threshold. If the neighbouring point has a small difference angle then it is given the same tag as the original point. Each point, found to be within the tolerance, is then pushed back to the queue for further processing of its own neighbours.

This results in each 3D point having associated to it a tag.

#### Cluster Filtering

Now that each point has a tag it is now possible to filter out points that do not represent a planar surface.

- First count how many points make up each tag
- Add each point to a collection of other points with that tag
- Filter out all the clusters (tags) with a given threshold

#### Dominant Plane

We now have a collection of plane clusters, but we really need to know which one is the table.

We can look at each cluster and determine its confidence as being the table (assuming that there is a table in the scene)

The confidence is calculated by giving a weighting to both the point count and the centroid position relative to the camera. This way, the largest and most centre plane is selected.




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




