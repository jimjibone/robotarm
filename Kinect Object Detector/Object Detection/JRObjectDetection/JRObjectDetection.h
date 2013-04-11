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

class ObjectDetection {
	template <typename PointT>
	struct PointCloud {
		std::vector<PointT> points;
		void eraseAll() {
			points.erase(points.begin(), points.end());
		}
		size_t addPoint(double _x, double _y, double _z) {
			size_t _index = points.size();
			points.emplace_back(_index, _x, _y, _z);
		}
	};
	
	ConvexHull convexHull;
	PointCloud<PointXYZ> cloud_objects;
	PlaneCoefficients plane_table;
	
public:
	ObjectDetection(double maxHeight);
	void prepareForNewData();
	void setPlaneCoefficients(double a, double b, double c, double d);
	void addPoint(double x, double y, double z);
	void addConvexHullPoint(double x, double y, double z);
	void determineHullCubeBounds();
	
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
