//
//  JRKMeansClustering.cpp
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRKMeansClustering.h"
#include <boost/thread/thread_time.hpp>

#pragma mark -
#pragma mark Private Funcitons

void KMeansClustering::updateCentroidForCluster(size_t cluster_index)
{
	double avgX = 0.0, avgY = 0.0, avgZ = 0.0;
	for (size_t i = 0; i < clusters[cluster_index].indices.size(); i++) {
		avgX += (*cloud)[clusters[cluster_index].indices[i]].x;
		avgY += (*cloud)[clusters[cluster_index].indices[i]].y;
		avgZ += (*cloud)[clusters[cluster_index].indices[i]].z;
	}
	avgX /= clusters[cluster_index].indices.size();
	avgY /= clusters[cluster_index].indices.size();
	avgZ /= clusters[cluster_index].indices.size();
	clusters_centroids[cluster_index] = PointXYZ(avgX, avgY, avgZ);
}

void KMeansClustering::updateRadiusForCluster(size_t cluster_index)
{
	double max_dist = 0;
	for (size_t p = 0; p < clusters[cluster_index].indices.size(); p++) {
		double dist = pointToPointDistance(clusters_centroids[cluster_index], (*cloud)[clusters[cluster_index].indices[p]]);
		if (dist > max_dist) {
			max_dist = dist;
		}
	}
	clusters_radii[cluster_index] = max_dist;
}

double KMeansClustering::pointToPointDistance(PointXYZ a, PointXYZ b)
{
	// Find the euclidean distance.
	return (double)( sqrt( pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2) ) );
}

size_t KMeansClustering::findNearestClusterToPoint(size_t index)
{
	double min_distance = 0.0;
	size_t nearest_cluster_index = SIZE_T_MAX;
	
	for (size_t k = 0; k < clusters.size(); k++) {
		
		double distance = pointToPointDistance((*cloud)[index], clusters_centroids[k]);
		if (k == 0) {
			
			min_distance = distance;
			nearest_cluster_index = 0;
			
		} else if (min_distance > distance) {
			
			min_distance = distance;
			nearest_cluster_index = k;
			
		}
		
	}
	return nearest_cluster_index;
}





#pragma mark -
#pragma mark Setup Functions

void KMeansClustering::setCloud(vector<PointXYZ> *newCloud)
{
	cloud = newCloud;
}

void KMeansClustering::useIndices(PointIndices use)
{
	included_indices = use;
}

void KMeansClustering::setClusterCount(size_t newClusterCount)
{
	cluster_count = newClusterCount;
}

void KMeansClustering::filterClusters(double max_distance)
{
	// Run through all the clusters and remove the points that
	// have a distance > max_distance away from their centroid.
	
	for (size_t c = 0; c < clusters.size(); c++) {
		
		for (size_t p = 0; p < clusters[c].indices.size(); p++) {
			
			double dist = pointToPointDistance(clusters_centroids[c], (*cloud)[clusters[c].indices[p]]);
			if (dist > max_distance) {
				// This point is too far away! Remove it from the cluster.
				size_t the_point = clusters[c].indices[p];
				clusters[c].indices.erase(remove(clusters[c].indices.begin(), clusters[c].indices.end(), the_point), clusters[c].indices.end());
			}
			
		}
		
		// Then recalculate the centroid and radius.
		updateCentroidForCluster(c);
		updateRadiusForCluster(c);
		
	}
}





#pragma mark -
#pragma mark Main Function

void KMeansClustering::run()
{
	if (cluster_count > 1) {
		
		cout << "KMeansClustering::run(). Starting." << endl;
		boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
		
		clusters.erase(clusters.begin(), clusters.end());
		clusters_centroids.erase(clusters_centroids.begin(), clusters_centroids.end());
		clusters_centroids.resize(cluster_count, PointXYZ());
		clusters_radii.erase(clusters_radii.begin(), clusters_radii.end());
		clusters_radii.resize(cluster_count, 0.0);
		
		// First of all, divide all the points into almost equal clusters.
		
		size_t points_per_cluster = included_indices.indices.size() / cluster_count;
		size_t points_overflow = (included_indices.indices.size() % cluster_count) / cluster_count;
		size_t point_count = 0;
		
		for (size_t cluster = 0; cluster < cluster_count; cluster++) {
			
			// Find out how many points we should put into this cluster initially.
			// This depends upon how many objects_points there are and how many clusters
			// the user has specified.
			size_t points_size = (cluster != cluster_count-1) ? points_per_cluster+points_overflow+cluster : points_per_cluster;
			PointIndices thisCluster;
			
			for (size_t i = 0; i < points_size; i++) {
				thisCluster.indices.emplace_back(included_indices.indices[point_count]);
				point_count++;
			}
			
			clusters.emplace_back(thisCluster);
			updateCentroidForCluster(cluster);
		}
		
		
		// Now for some K-Means clustering.
		
		int movements = 1;
		while (movements > 0) {
			
			movements = 0;
			
			// Iterate through all the clusters.
			for (size_t c = 0; c < clusters.size(); c++) {
				
				// Iterate through all the points in this cluster.
				for (size_t p = 0; p < clusters[c].indices.size(); p++) {
					
					size_t nearest_cluster = findNearestClusterToPoint(clusters[c].indices[p]);
					if (nearest_cluster != c) {
						// Therefore the point has moved.
						
						if (clusters[c].indices.size() > 1) {
							// The cluster must have a minimum of one point.
							
							// And now let's move this point to the new cluster.
							size_t moving_point = clusters[c].indices[p];
							clusters[c].indices.erase(remove(clusters[c].indices.begin(), clusters[c].indices.end(), moving_point), clusters[c].indices.end());
							clusters[nearest_cluster].indices.emplace_back(moving_point);
							movements += 1;
						}
					}
					
				}
				
			}
			
		}
		
		// Then finally calculate the radii for all clusters.
		for (size_t c = 0; c < clusters.size(); c++) {
			updateRadiusForCluster(c);
		}
		
		// all done
		boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration diff = stop - start;
		cout << "KMeansClustering::run(). Ended with:" << endl;
		for (size_t i = 0; i < clusters.size(); i++) {
			cout << "\tcluster " << i << " containing " << clusters[i].indices.size() << " points." << endl;
		}
		cout << "\tTaken " << diff.total_microseconds() << " us." << endl;
	}
}

