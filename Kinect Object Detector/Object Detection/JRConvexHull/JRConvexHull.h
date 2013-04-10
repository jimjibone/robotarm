//
//  JRConvexHull.h
//  Convex Hull Small
//
//  Created by James Reuss on 01/02/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Convex_Hull_Small__JRConvexHull__
#define __Convex_Hull_Small__JRConvexHull__

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

class ConvexHull {
	struct PointCH {
		double x, y, z;	// 3D points from the original->projectd-to-plane data.
		double i, j;	// 2D points once 3D->2D conversion has been completed.
		bool operator <(const PointCH &point) const {
			return i < point.i || (i == point.i && j < point.j);
		}
		PointCH(double _x = 0, double _y = 0, double _z = 0, double _i = 0, double _j = 0) : x(_x), y(_y), z(_z), i(_i), j(_j) {};
	};
	struct PlaneCH {
		double a, b, c, d, tolerance;
		bool isSet() {
			return a || b || c || d || tolerance;
		}
		PlaneCH(double _a = 0, double _b = 0, double _c = 0, double _d = 0, double _tolerance = 0) : a(_a), b(_b), c(_c), d(_d), tolerance(_tolerance) {};
	};
	vector<PointCH> planePoints;
	PlaneCH plane;
	
	double cross(const PointCH &o, const PointCH &a, const PointCH &b);
	
public:
	vector<PointCH> convexHullPoints;
	
	ConvexHull();
	~ConvexHull();
	void setPlane(double newA, double newB, double newC, double newD, double newTol);
	bool addPoint(double newX, double newY, double newZ);	// Returns bool indicating wasAdded. Relating to tolerance.
	void listPoints();
	void performConvexHull();
	void listConvexHullPoints();
	void resetConvexHull();
};

#endif /* defined(__Convex_Hull_Small__JRConvexHull__) */
