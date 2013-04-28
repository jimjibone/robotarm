//
//  JRConvexHull.h
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Object_Detector__JRConvexHull__
#define __Object_Detector__JRConvexHull__

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include "JRPointTypes.h"

using namespace std;

class ConvexHull {
	
	vector<PointXYZ> *cloud;
	PointIndices *indices;
	PlaneCoefficients *plane;
	
	double distance_tolerance;
	
	vector<PointNXYZIJ> projected_cloud;
	
	double cross(const PointNXYZIJ &o, const PointNXYZIJ &a, const PointNXYZIJ &b);
	void projectPoints();
	
public:
	
	PointIndices hull_indices;
	
	//ConvexHull();
	//~ConvexHull();
	
	void setData(vector<PointXYZ> *newCloud, PointIndices *newIndices, PlaneCoefficients *newPlane);
	void run();
	
};

#endif /* defined(__Object_Detector__JRConvexHull__) */
