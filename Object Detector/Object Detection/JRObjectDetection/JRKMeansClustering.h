//
//  JRKMeansClustering.h
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __Object_Detector__JRKMeansClustering__
#define __Object_Detector__JRKMeansClustering__

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include "JRPointTypes.h"

// K-Means algorithm implemented by guidance from source: http://www.codeding.com/articles/k-means-algorithm

using namespace std;

class KMeansClustering {
	
	vector<PointXYZ> *cloud;
	PointIndices included_indices;
	
	size_t cluster_count;
	
	void updateCentroidForCluster(size_t cluster_index);	// must be used whenever a cluster point is changed.
	void updateRadiusForCluster(size_t cluster_index);
	double pointToPointDistance(PointXYZ a, PointXYZ b);
	size_t findNearestClusterToPoint(size_t index);
	
public:
	
	vector<PointIndices> clusters;
	vector<PointXYZ> clusters_centroids;
	vector<double> clusters_radii;
	
	KMeansClustering(vector<PointXYZ> *_cloud, PointIndices _included_indices, size_t _cluster_count) : cloud(_cloud), included_indices(_included_indices), cluster_count(_cluster_count) {};
	KMeansClustering() { cluster_count = 0; };
	//~KMeansClustering();
	
	void setCloud(vector<PointXYZ> *newCloud);
	void useIndices(PointIndices use);
	void setClusterCount(size_t newClusterCount);
	void run();
	void filterClusters(double max_distance);
	
};


#endif /* defined(__Object_Detector__JRKMeansClustering__) */
