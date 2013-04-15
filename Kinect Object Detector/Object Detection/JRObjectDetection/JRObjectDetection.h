//
//  JRObjectDetection.h
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Kinect_Object_Detector__JRObjectDetection__
#define __Kinect_Object_Detector__JRObjectDetection__

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "JRConvexHull.h"
#include "../JRPointTypes.h"
#include "JRKinectHelpers.h"

#define OBJECT_MAX_HEIGHT 500	// mm above table
#define OBJECT_MIN_HEIGHT 10	// mm above table

class ObjectDetection {
	ConvexHull objectPointFinder;	// To help find objects that lie on the table plane within the table surface.
	
	uint maxPoints;					// The maximum number of points in the data.
	double maxHeight;				// The max height a point can be above the table.
	double minHeight;				// The min height a point can be above the table.
	bool validDepthData;			// Is true when the input depth data actually contains data. Saves the algorithm from killing itself.
	
	PlaneCoefficients plane_table;	// The plane coefficients for the table found using RANSAC.
	PointCloud<PointXYZ> cloud;		// The complete cloud of valid data points.
	
public:
	PointCloud<PointXYZIJ> cloud_objects;	// The cloud containing all points that lie within the table surface.
	
	ObjectDetection(uint newMaxPoints, double newMaxHeight, double newMinHeight);
	void setPlaneCoefficients(double a, double b, double c, double d, bool invert);
	void addPreprocessedConvexHullPoint(PointXYZIJ aPoint);
	
	void updateDepthData(uint16_t *newDepth);
	void performObjectDetection();
	void prepareForNewData();
	
	
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
