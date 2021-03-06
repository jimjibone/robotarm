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
#define COMPARE_NORMALS_DISTANCE_THRESH	(10.0)			// 10 mm
#define COMPARE_NORMALS_ANGLE_THRESH	(PI_VALUE/26)	// 5 deg in radians
#define PLANE_NEIGHBOUR_SEARCH_DIST		(3)				// The distance the segmentation algorithm should search in each direction for neighbours.
#define PLANE_CLUSTER_THRESHOLD			(2000)			// The number of points a cluster should have in order to pass as a plane.
#define RANSAC_DISTANCE_TOLERANCE		(20.0)			// The max absolute distance that a point can be away from the current plane in order to increase confidence.
#define CONVEXHULL_DISTANCE_TOLERANCE	(20.0)			// The max distance a point can be from the plane to be included as a plane point.
#define INCLUSION_MAX_HEIGHT			(500.0)			// The maximum height a point can have of the dominant plane in order to be included as an object.
#define INCLUSION_MIN_HEIGHT			(15.0)			// The minimum height a point must have off the dominant plane in order to be included as an object.
#define KMEANS_CLUSTER_COUNT			(2)				// The number of clusters the k-means algorithm should look for.
#define KMEANS_FILTER_DISTANCE			(50.0)			// The max distance a point can be away from its centroid for it to be included.

using namespace std;

class ObjectDetection {
	
	bool validDepthData;
	
	bool compareNormalAngle(size_t a, size_t b);
	
public:
	
	vector<PointXYZ> input_cloud;
	vector<PlaneCoefficients> input_cloud_normals;
	vector<PointIndices> plane_clusters;
	
	struct {
		size_t index;
		int confidence;
		PlaneCoefficients coefficients;
		PointIndices hull;
	} dominant_plane;
	
	PointIndices objects_points;
	vector<Object> objects;
	
	void setZPoints(uint16_t* zPoints, size_t size);
	void calculateSurfaceNormals();
	void segmentPlanes();
	void findDominantPlane();
	void segmentObjects();
};

#endif /* defined(__Kinect_Object_Detector__JRObjectDetection__) */
