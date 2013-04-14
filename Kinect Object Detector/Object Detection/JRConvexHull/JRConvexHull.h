//
//  JRConvexHull.h
//  Convex Hull Small
//
//  Created by James Reuss on 01/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Convex_Hull_Small__JRConvexHull__
#define __Convex_Hull_Small__JRConvexHull__

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "../JRPointTypes.h"

using namespace std;

class ConvexHull {
	typedef PointXYZIJ PointCH;	// conversion to new point types
	typedef PlaneCoefficients PlaneCH; // conversion to new point types
	
	vector<PointXYZIJ> planePoints;
	PlaneCoefficients plane;
	double tolerance;
	
	double cross(const PointCH &o, const PointCH &a, const PointCH &b);
	
public:
	vector<PointXYZIJ> convexHullPoints;
	
	ConvexHull();
	~ConvexHull();
	void setPlane(double newA, double newB, double newC, double newD, double newTol);
	bool addPoint(double newX, double newY, double newZ);	// Returns bool indicating wasAdded. Relating to tolerance.
	void listPoints();
	void performConvexHull();
	void listConvexHullPoints();
	void resetConvexHull();
	
	// These next functions allow the processing a pre-processed convex hull and
	// individual points for the use of point-within-bounds calculations.
	void addPreprocessedConvexHullPoint(PointXYZIJ aPoint);
	bool processPointWithPreprocessedHull(double x, double y, double z);	// true = outside, false = inside.
};

#endif /* defined(__Convex_Hull_Small__JRConvexHull__) */
