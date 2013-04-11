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

class JRObjectDetection {
	// Point Types
	typedef struct {
		double x, y, z;
	} PointXYZ;
	typedef struct {
		double a, b, c, d, confidence;
	} PlaneCoeffs;
	
	template <typename PointT>
	struct PointCloud {
		std::vector<size_t> indices;
		std::vector<PointT> points;
	};
	
	PointCloud<PointXYZ> cloud_objects;
	
public:
	void setCloudData(uint16_t* newDepthData);
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
