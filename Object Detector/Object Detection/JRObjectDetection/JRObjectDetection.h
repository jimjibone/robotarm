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
#define DOMINANT_PLANE_UNASSIGNED (SIZE_T_MAX)

#define NORMAL_CALC_POINT_SPREAD		(10)			// 10 points
#define COMPARE_NORMALS_DISTANCE_THRESH	(10.0)			// 20 mm
#define COMPARE_NORMALS_ANGLE_THRESH	(PI_VALUE/26)	// 5 deg in radians
#define PLANE_NEIGHBOUR_SEARCH_DIST		(3)				// The distance the segmentation algorithm should search in each direction for neighbours.
#define PLANE_CLUSTER_THRESHOLD			(2000)			// The number of points a cluster should have in order to pass as a plane.

using namespace std;

class ObjectDetection {
	
	bool validDepthData;
	
	bool compareNormalAngle(size_t a, size_t b);
	
public:
	
	vector<PointXYZ> input_cloud;
	vector<PlaneCoefficients> input_cloud_normals;
	vector<PointIndices> plane_clusters;
	vector<PlaneCoefficients> plane_clusters_normals;
	size_t dominant_plane_index;
	
	void setZPoints(uint16_t* zPoints, size_t size);
	void calculateSurfaceNormals();
	void segmentPlanes();
	void findDominantPlane();
	
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
