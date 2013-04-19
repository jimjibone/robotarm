//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"



PointXYZ worldFromIndex(unsigned int index, int z) {
	double x = index % FREENECT_FRAME_W;
	double y = (index - x) / FREENECT_FRAME_W;
	x = (double)((x - FREENECT_FRAME_W/2.0) * z * 0.00174);
	y = (double)((y - FREENECT_FRAME_H/2.0) * z * -0.00174);
	return PointXYZ(x, y, z);
}

// Function to get the plane equation from 3 points in 3D space.
PlaneCoefficients getPlaneCoefficients(Point a, Point b, Point c) {
	// First validate the input to check that Z values are not out-of-bounds of Kinect view.
	if (!a.isValid() || !b.isValid() || !c.isValid()) {
		return PlaneCoefficients(0, 0, 0, 0);
	}
	
	//http://keisan.casio.com/has10/SpecExec.cgi# or
	//http://www.easycalculation.com/analytical/cartesian-plane-equation.php
	double Pa = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
	double Pb = (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x);
	double Pc = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
	double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
	
	return Plane(Pa, Pb, Pc, Pd, true, 0);
}