//
//  JRPointInclusion.h
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Object_Detector__JRPointInclusion__
#define __Object_Detector__JRPointInclusion__

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include "JRPointTypes.h"

using namespace std;

class PointInclusion {
	
	vector<PointXYZ> *cloud;
	PlaneCoefficients *plane;
	vector<PointXY> hull;
	PointIndices excluded_indices;
	
	bool invert_plane;
	double min_distance;
	double distance_tolerance;
	
	bool pointWithinHull(PointXY point);
	
public:
	
	PointIndices included_indices;
	vector<PointXY> flattened_points;
	
	PointInclusion(double _tolerance) : distance_tolerance(_tolerance) { invert_plane = false; min_distance = 0; };
	//~PointInclusion();
	
	void setMinDistance(double newMin);
	void setCloud(vector<PointXYZ> *newCloud);
	void setPlane(PlaneCoefficients *newPlane);
	void setHull(PointIndices *newHull);
	void excludeIndices(PointIndices *exclude);
	void run();
	
};

#endif /* defined(__Object_Detector__JRPointInclusion__) */
