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

**RANSAC**

The RANSAC algorithm stands for Random Sample and Consensus.

It is a means by which it is possible to quickly approximate a desired value, and in this case it is used to approximate the plane coefficients of the table surface.

It works by:

- First selecting a number of random points
- Then group them into threes and determine their normal coefficients
- Then compare all the other 'non-random' points with each normal
- If the normal has a point nearby it (within a threshold) then increase its confidence
- The normal coefficients that have the highest confidence is the approximate of the table plane.

**Convex Hull**

Using the Convex Hull algorithm it is possible to determine the points which make up the outer edge of the table surface. This later allows for the processing of points that only lie within and on the table.

The simplest analogy that explains the convex hull is the rubber band. Imagine these points make up all the table surface points. Stretch a rubber band around all the points and let go. You will end up with the rubber band looking like this once it has contracted.

This is similar to how the convex hull algorithm works. You are then left with all the points that make up the outer ring of the collection.

### Point Inclusion

We now have a lot information available:

- The table plane coefficients (RANSAC)
- The table plane bounds (convex hull)
- The points that make up the table surface

So now we need to find out which points from the original point cloud make up the objects on the table.

To do this, it is possible to use the point-in-polygon algorithm. This was implemented using the Point Inclusion in Polygon Test created by W. Randolf Franklin.

It uses the method shown here to determine whether point is located within a given polygon. In this case, the polygon is all the points of the convex hull.

This algorithm works by:

- Running a semi-infinite ray out from a test point towards the tested point.
- As the ray crosses an edge of the polygon a count is incremented.
- When the ray crosses an edge it is switching between inside and outside.
- This is also known as the Jordan Curve Theorem.
- Depending upon the number of crosses the ray has made will determine the position of the point.
- An even value of switches indicates outside. An odd value indicates that the point is inside.

Performing this operation on all of the points will give a collection of points which make up all the objects on the table.

### Object Point Clustering

This is the final stage of the processing!

We have a collection of 3D points which represent **all** of the objects we want to detect.

Now, we just need to spilt up these groups of points into their individual objects. This is where the K-Means clustering algorithm comes in.

So what is the K-Means algorithm?

It is a process by which:

- All of the points are split up into 'k' groups.
- We then need to calculate the centroid of each group.
- Now iterate through each point and determine which group it is closest to.
- If it is closer to a different group other than its own, move it.
- Continue this process until there are no more movements of points.

The points will now be split into the 'k' clusters correctly.

Final Results
-------------

So how does this system work in practice?

After calibration of the object detection system there were some very good results.

Comment on each image.

Improvements
------------

One major limitation of the object detection system that has been implemented is that the K-Means algorithm requires that you specify how many objects there are before computation. This can be troublesome in a more practical application.

However, an alternative clustering method was discovered, but there was no time to implement it.

This method was to use the clustering technique previously used for planes, but instead of using the point surface normals as comparators, the distance of a point from a centroid would be used. In this way, it would be possible to implement a system which could support, theoretically an infinite amount of objects.

Future Work
-----------

There are many additional pieces of work which can be carried out on this section. These are:

- Shape fitting
- Integration of eye-tracker information
- Communication with electronic system
- Reduced processing time



Project Execution
=================

> Someone




