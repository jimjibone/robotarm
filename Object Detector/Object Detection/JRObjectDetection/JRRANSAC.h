//
//  JRRANSAC.h
//  Object Detector
//
//  Created by James Reuss on 27/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Object_Detector__JRRANSAC__
#define __Object_Detector__JRRANSAC__

#define RANDOM_POINT_DIVISOR 10	// 10% of the points will be used for random points.

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include "JRPointTypes.h"

using namespace std;

template <class PointType>
class ransac {
	
	vector<PointType> *cloud;
	PointIndices *indices;
	double distance_tolerance;
	
	vector<size_t> random_points;	// This actually contains the points which should be accessed from indices.
	vector<size_t> non_random_points;	// These also contain the points which should be accessed from indices.
	
	void generatePoints();
	
public:
	
	void setCloud(vector<PointType> *newCloud);
	void setIndices(PointIndices *newIndices);
	void setData(vector<PointType> *newCloud, PointIndices *newIndices);
	void setDistanceTolerance(double newTolerance);
	void run();
	
};

#endif /* defined(__Object_Detector__JRRANSAC__) */