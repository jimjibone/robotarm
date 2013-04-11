//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

void ObjectDetection::prepareForNewData()
{
	cloud_objects.eraseAll();
}

void ObjectDetection::setPlaneCoefficients(double a, double b, double c, double d)
{
	plane_table.setCoefficients(a, b, c, d);
}

void ObjectDetection::addPoint(double x, double y, double z)
{
	cloud_objects.addPoint(x, y, z);
}

void ObjectDetection::addConvexHullPoint(double x, double y, double z)
{
	
}





// MAIN PROCESSING
void ObjectDetection::determineHullCubeBounds()
{
	// could use the convex hull algorithm to determine whether the point is inside or outside of the hull.
	// if the point is included in the returned indices then the point is outside the hull, else it is inside!
}

