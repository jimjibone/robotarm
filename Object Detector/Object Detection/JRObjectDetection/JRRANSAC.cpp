//
//  JRRANSAC.cpp
//  Object Detector
//
//  Created by James Reuss on 27/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRRANSAC.h"

// Function to get the plane equation from 3 points in 3D space.
PlaneCoefficients calculatePlane(PointXYZ a, PointXYZ b, PointXYZ c) {
	// First validate the input to check that Z values are not out-of-bounds of Kinect view.
	if (!a.isValid() || !b.isValid() || !c.isValid()) {
		return PlaneCoefficients(0, 0, 0, 0);
	}
	
	//http://keisan.casio.com/has10/SpecExec.cgi# or
	//http://www.easycalculation.com/analytical/cartesian-plane-equation.php
	double Pa = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
	double Pb = (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x);
	double Pc = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
	double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
	
	return PlaneCoefficients(Pa, Pb, Pc, Pd);
}

// Function to find the distance between a point and a plane.
double absoluteDistance(PointXYZ point, PlaneCoefficients plane) {
	// http://mathworld.wolfram.com/Point-PlaneDistance.html
	double distance = 0;
	distance  = plane.a * point.x;
	distance += plane.b * point.y;
	distance += plane.c * point.z;
	distance += plane.d;
	distance /= sqrt(pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2));
	distance  = distance >= 0 ? distance : -distance;
	return distance;
}

template <class PointType>
void RANSAC<PointType>::setCloud(vector<PointType> *newCloud)
{
	cloud = newCloud;
}

template <class PointType>
void RANSAC<PointType>::setIndices(PointIndices *newIndices)
{
	indices = newIndices;
}

template <class PointType>
void RANSAC<PointType>::setData(vector<PointType> *newCloud, PointIndices *newIndices)
{
	cloud = newCloud;
	indices = newIndices;
}

template <class PointType>
void RANSAC<PointType>::setDistanceTolerance(double newTolerance)
{
	distance_tolerance = newTolerance;
}

template <class PointType>
void RANSAC<PointType>::generatePoints()
{
	// http://en.wikipedia.org/wiki/RANSAC#The_algorithm
	// Also see Computer Vision book, pp. 305 (algorithm 10.4).
	
	// Generate the random access points.
	random_points.erase(random_points.begin(), random_points.end());
	srand((unsigned int)time(0));	// Seed the random number generator.
	
	// Get a better amount of random points.
	size_t max_points = indices->indices.size() / RANDOM_POINT_DIVISOR;
	max_points += max_points % 3;	// Make up the size to be a multiple of 3.
	
	for (size_t i = 0; i < max_points; i++) {
		
		size_t random_index = rand() % indices->indices.size()-1;
		
		// Check that the random number has not alreayd been used.
		// If it has then continue, have another go.
		bool isUsed = find(random_points.begin(), random_points.end(), random_index) != random_points.end();
		
		if (isUsed) {
			i--;		// Put the count back so we don't lose a point.
			continue;	// Try the process again.
		}
		
		// All went well so let's add the index to the vector.
		random_points.emplace_back(random_index);
	}
	
	// Sort the points lexicographically.
	sort(random_points.begin(), random_points.end());
	
	// Generate the indices for all the non_random_points.
	non_random_points.erase(non_random_points.begin(), non_random_points.end());
	for (size_t i = 0; i < indices->indices.size(); i++) {
		
		// Check if this index 'i' has already been used in the random points.
		bool isUsed = find(random_points.begin(), random_points.end(), i) != random_points.end();
		
		if (!isUsed) {
			non_random_points.emplace_back(i);
		}
		
	}
	
}

template <class PointType>
void RANSAC<PointType>::run()
{
	generatePoints();
	
	vector<PlaneCoefficients> planes;
	vector<uint> plane_confidences;
	
	// Grab 3 random points at a time and then determine the plane equation.
	for (size_t i = 0; i < random_points.size(); i+=3) {
		
		PlaneCoefficients thisPlane = calculatePlane(cloud->at(i), cloud->at(i+1), cloud->at(i+2));
		if (thisPlane.isSet()) {
			planes.emplace_back(thisPlane);
			plane_confidences.emplace_back(0);
		}
		
	}
	
	// Iterate through all the valid planes and process with all the
	// non random points. Determine the plane confidences.
	plane_confidences.resize(planes.size(), 0);
	int highest_confidence = INT_MIN;
	size_t highest_confidence_index = 0;
	for (size_t i = 0; i < planes.size(); i++) {
		
		// Now compare this plane with all the non random
		// points.
		
		for (size_t j = 0; j < non_random_points.size(); j++) {
			
			// Find the distance from point to plane to determine the
			// confidence.
			double distance = absoluteDistance(cloud[indices[non_random_points[j]]], planes[i]);
			
			if (distance <= distance_tolerance) {
				plane_confidences[i]++;
			} else {
				plane_confidences[i]--;
			}
			
		}
		
		// Is this the best plane so far?
		if (plane_confidences[i] > highest_confidence) {
			highest_confidence = plane_confidences[i];
			highest_confidence_index = i;
		}
	}
	
	// Now that the confident plane has been found, set the
	// confident plane.
	coefficients = planes[highest_confidence_index];
	
}
