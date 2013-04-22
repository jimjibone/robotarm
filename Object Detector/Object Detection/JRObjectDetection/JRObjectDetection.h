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
#include "../JRPointTypes.h"
#include "JRKinectHelpers.h"

#define OBJECT_MAX_HEIGHT (500)	// mm above table
#define OBJECT_MIN_HEIGHT (10)	// mm above table
#define PI_VALUE (3.141592654)
#define NODE_UNASSIGNED (-1)

using namespace std;

class ObjectDetection {
	
	bool validDepthData;
	
	bool compareNormalAngle(size_t a, size_t b);
	
public:
	
	vector<PointXYZ> input_cloud;
	vector<PlaneCoefficients> input_cloud_normals;
	vector<PointIndices> plane_clusters;	//currently only used in old segmentation methods
	vector<int> plane_cluster_nodes;
	
	void setZPoints(uint16_t* zPoints, size_t size);
	void calculateSurfaceNormals();
	void segmentPlanes_old_old();
	void segmentPlanes_old();
	void segmentPlanes();
	void filterPlanes();
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
