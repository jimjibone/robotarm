//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

void worldFromIndex_int(double* wx, double* wy, unsigned int index, int z) {
	unsigned int x = index % 640;
	unsigned int y = (index - x) / 480;
	*wx = (double)((x - 640/2.0)*(z + 0)*0.00174);
	*wy = (double)((y - 480/2.0)*(z + 0)*0.00174*-1.0);
}

// Initialisation Functions

ObjectDetection::ObjectDetection(uint newMaxPoints, double newMaxHeight, double newMinHeight)
{
	maxPoints = newMaxPoints;
	maxHeight = newMaxHeight;
	minHeight = newMinHeight;
	validDepthData = false;
}

void ObjectDetection::setPlaneCoefficients(double a, double b, double c, double d, bool invert)
{
	plane_table.setCoefficients(a, b, c, d);
	objectPointFinder.setPlane(a, b, c, d, 0, invert);
}

void ObjectDetection::addPreprocessedConvexHullPoint(PointXYZIJ aPoint)
{
	objectPointFinder.addPreprocessedConvexHullPoint(aPoint);
}




// Each 'New Frame' Functions
// Execute in order shown.

void ObjectDetection::updateDepthData(uint16_t *newDepth)
{
	// Iterate through the depth data that has been input and calculate the
	// relevent world x & y coordinates.
	
	uint16_t depthCount = 0;
	for (int i = 0; i < maxPoints; i++) {
		
		// Is the point valid?
		if (newDepth[i] > 0) {
			// Then add the point to the cloud.
			PointXYZ newPoint = PointXYZ(0, 0, newDepth[i]);
			//void worldFromIndex(double* wx, double* wy, unsigned int index, int z);
			worldFromIndex_int(&newPoint.x, &newPoint.y, i, newDepth[i]);
			cloud.addPoint(newPoint);
		}
		
		depthCount += newDepth[i];
	}
	if (depthCount > 0) {
		validDepthData = true;
	} else {
		validDepthData = false;
		printf("ObjectDetection::updateDepthData - data is not valid\n");
	}
	
}

void ObjectDetection::performObjectDetection()
{
	// could use the convex hull algorithm to determine whether the point is inside or outside of the hull.
	// if the point is included in the returned indices then the point is outside the hull, else it is inside!
	
	// 1. Compute the RANSAC of the scene to develop the plane coefficients of the table. (Done in obj-c)
	// 2. Compute the Convex Hull of the scene with respect to the plane coefficients. (Done in obj-c)
	// 3. Send the convex hull points of the table edges to the objectPointFinder and create a cloud_objects point cloud.
	// 4.
	
	
	// Reset
	cloud_objects.eraseAll();
	
	
	if (validDepthData == true) {
		
		// Compute the RANSAC algorithm to find the plane coefficients of the table plane.
		
		// Compute the Convex Hull of the scene with respect to the plane found.
		
		// Send the cloud points to convexHull and determine which points are located on the table surface.
		for (int i = 0; i < cloud.points.size(); i+=1000) {
			PointXYZIJ newPoint;
			bool isInside = objectPointFinder.processPointWithPreprocessedHull(cloud.points[i], &newPoint, maxHeight, minHeight);	// true = inside, false = outside.
			if (isInside == true) {
				cloud_objects.addPoint(newPoint);
			}
		}
		
		// Next...
	}
}

void ObjectDetection::prepareForNewData()
{
	cloud.eraseAll();
}


