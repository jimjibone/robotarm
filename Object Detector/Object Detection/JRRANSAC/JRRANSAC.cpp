//
//  JRRANSAC.cpp
//  RANSAC CPP
//
//  Created by James Reuss on 03/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRRANSAC.h"

RANSAC::RANSAC(uint newMaxPoints, uint newRandomPointCount, double newDistanceTolerance) {
	maxPoints = newMaxPoints;
	distanceTolerance = newDistanceTolerance;
	validDepthData = false;
	
	// The random point count must be a multiple of 3.
	int remainder = newRandomPointCount % 3;
	maxRandomPoints = newRandomPointCount + (3-remainder);
	
	// Randomly create the randomPoints.
	srand((unsigned int)time(0));	// Seed the random number generator.
	randomPoints.reserve(maxRandomPoints);
	for (uint i = 0; i < maxRandomPoints; i++) {
		unsigned int randomInt = irand(0, maxPoints-1);
		bool isUsed = false;
		// Check that the random number has not alreayd been used.
		// If it has then continue, making it have another go.
		for (Point const& point : randomPoints) {
			if (point.index == randomInt) {
				isUsed = true;
			}
		}
		if (isUsed) {
			i--;		// Put the count back so we don't lose a point.
			continue;	// Try the process again.
		}
		
		// All went well so let's add the index to the vector.
		randomPoints.emplace_back(randomInt);
		
	}
	// Sort the points lexicographically.
	sort(randomPoints.begin(), randomPoints.end());
	
	// Generate the indices for all the otherPoints.
	otherPoints.reserve(maxPoints - maxRandomPoints - 1);
	// Iterate through all points from 0 to maxPoints. If the point is in
	// randomPoints then don't add it to the vector.
	for (uint i = 0, j = 0; i < maxPoints; i++) {
		// Skip this point if it is in randomPoints.
		if (i == randomPoints[j].index) {
			j++;
			continue;
		}
		// Else, add it to the otherPoints.
		otherPoints.emplace_back(i);
	}
}
RANSAC::~RANSAC() {
	// Do nothing.
}
void RANSAC::listRandomPoints() {
	cout << "Listing the random points developed. Total number = " << randomPoints.size() << endl;
	uint count = 0;
	for (Point const& point : randomPoints) {
		cout << "\tPoint " << count << " : " << point.index << endl;
		count++;
	}
}
void RANSAC::listOtherPoints() {
	cout << "Listing the other points developed. Total number = " << otherPoints.size() << endl;
	uint count = 0;
	for (Point const& point : otherPoints) {
		cout << "\tPoint " << count++ << " : " << point.index << endl;
	}
}
void RANSAC::listFullSet() {
	cout << "Listing the random points developed. Total number = " << randomPoints.size() << endl;
	uint count = 0;
	for (Point const& point : randomPoints) {
		cout << "\tPoint " << count++ << " : " << point.index << " : X:" << point.x << " Y:" << point.y << " Z:" << point.z << endl;
	}
	cout << "Listing the other points developed. Total number = " << otherPoints.size() << endl;
	count = 0;
	for (Point const& point : otherPoints) {
		cout << "\tPoint " << count++ << " : " << point.index << " : X:" << point.x << " Y:" << point.y << " Z:" << point.z << endl;
	}
}
void RANSAC::listConfidentPlanes() {
	cout << "Listing the confident planes developed. Total number = " << confidentPlanes.size() << endl;
	uint count = 0;
	for (Plane const& plane : confidentPlanes) {
		cout << "\tPlane " << count++ << " : A:" << plane.a << " B:" << plane.b << " C:" << plane.b << " D:" << plane.d << " Confidence:" << plane.confidence << endl;
	}
}
void RANSAC::updateDepthData(uint16_t *newDepth) {
	// Iterate through the depth data that has been input and calculate the
	// relevent world x & y coordinates.
	
	uint16_t depthCount = 0;
	for (int i = 0, r = 0, o = 0; i < maxPoints; i++) {
		// Get the relevent Point from either randomPoints or otherPoints.
		Point *point = 0;
		
		if (i == randomPoints[r].index) {
			point = &randomPoints[r];
			r++;
		}
		else if (i == otherPoints[o].index) {
			point = &otherPoints[o];
			o++;
		}
		
		worldFromIndex(&point->x, &point->y, point->index, newDepth[i]);
		
		point->z = newDepth[i]+1.0;
		point->z -= 1.0;
		
		depthCount += newDepth[i];
	}
	if (depthCount > 0) {
		validDepthData = true;
	} else {
		validDepthData = false;
		printf("updateDepthData dataIsNotValid\n");
	}
	
}
void RANSAC::performRANSAC() {
	// http://en.wikipedia.org/wiki/RANSAC#The_algorithm
	// Also see Computer Vision book, pp. 305 (algorithm 10.4).
	
	// Only perform this algorithm if there is valid depth data.
	if (validDepthData == false) {
		return;
	}
	
	// Grab 3 randomPoints at a time and determine the plane equation.
	Plane currentPlane;
	for (int i = 0; i < randomPoints.size(); i+=3) {
		currentPlane = getPlane(randomPoints[i], randomPoints[i+1], randomPoints[i+2]);
		// Only add the plane to the planes vector if it is valid.
		if (currentPlane.isValid) {
			//planes.emplace_back(currentPlane.a, currentPlane.b, currentPlane.c, currentPlane.d, currentPlane.isValid, 0);
			planes.emplace_back(currentPlane);
		}
	}
	
	// Iterate through all the valid planes and process with all the otherPoints.
	// Determine the planes confidences.
	struct _confidentPlane {
		double confidence;
		uint index;
	} confidentPlane = {-999999, 0};
	for (int i = 0; i < planes.size(); i++/*Plane plane : planes*/) {
		for (int j = 0; j < otherPoints.size(); j+=100/*Point const& point : otherPoints*/) {
			// Determine the distance from point to plane and find the confidence.
			double distance = findAbsoluteDistance(otherPoints[j], planes[i]);
			if (distance <= distanceTolerance) {
				planes[i].confidence++;
			} else {
				planes[i].confidence--;
			}
		}
		if (planes[i].confidence > confidentPlane.confidence) {
			confidentPlane.confidence = planes[i].confidence;
			confidentPlane.index = i;
		}
	}
	
	// Now that the confident plane has been found, add it to the confidentPlanes vector and sort
	// the confident planes by confidence.
	confidentPlanes.emplace_back(planes[confidentPlane.index]);
	sort(confidentPlanes.begin(), confidentPlanes.end());
	
	// Erase all the planes used for this iteration of the function.
	planes.erase(planes.begin(), planes.end());
	
	// The user can now ask for the most confident plane by executing the function confidentPlane().
	// The user may also repeat calls of this function in order to process new frames and find more confidentPlanes.
	// This can help to increase the likelyhood of producing a good plane equation.
}
void RANSAC::prepareRANSAC() {
	// Find new random points and the other points.
	randomPoints.erase(randomPoints.begin(), randomPoints.end());
	otherPoints.erase(otherPoints.begin(), otherPoints.end());
	
	// Randomly create the randomPoints.
	srand((unsigned int)time(0));	// Seed the random number generator.
	randomPoints.reserve(maxRandomPoints);
	for (uint i = 0; i < maxRandomPoints; i++) {
		unsigned int randomInt = irand(0, maxPoints-1);
		bool isUsed = false;
		// Check that the random number has not alreayd been used.
		// If it has then continue, making it have another go.
		for (Point const& point : randomPoints) {
			if (point.index == randomInt) {
				isUsed = true;
			}
		}
		if (isUsed) {
			i--;		// Put the count back so we don't lose a point.
			continue;	// Try the process again.
		}
		
		// All went well so let's add the index to the vector.
		randomPoints.emplace_back(randomInt);
		
	}
	// Sort the points lexicographically.
	sort(randomPoints.begin(), randomPoints.end());
	
	// Generate the indices for all the otherPoints.
	otherPoints.reserve(maxPoints - maxRandomPoints - 1);
	// Iterate through all points from 0 to maxPoints. If the point is in
	// randomPoints then don't add it to the vector.
	for (uint i = 0, j = 0; i < maxPoints; i++) {
		// Skip this point if it is in randomPoints.
		if (i == randomPoints[j].index) {
			j++;
			continue;
		}
		// Else, add it to the otherPoints.
		otherPoints.emplace_back(i);
	}
}
void RANSAC::getConfidentPlane(double *A, double *B, double *C, double *D, double *confidence, bool *needsInvert) {
	// Returns (via pointer) the most confident plane from the current processed set.
	if (validDepthData == true) {
		// Determine the proper orientation of the calculated plane. For use with distance calcs.
		// Point with small z should give a positive value, assuming that the table is positioned
		// below the Kinect sensor physically.
		double dist = 0;
		// http://mathworld.wolfram.com/Point-PlaneDistance.html
		dist  = 0.0 * confidentPlanes.front().a;
		dist += 0.0 * confidentPlanes.front().b;
		dist += 100.0 * confidentPlanes.front().c;
		dist += confidentPlanes.front().d;
		dist /= sqrt(pow(confidentPlanes.front().a, 2) + pow(confidentPlanes.front().b, 2) + pow(confidentPlanes.front().c, 2));
		if (dist >= 0) {
			confidentPlanes.front().needsInvert = false;
		} else {
			confidentPlanes.front().needsInvert = true;
		}
		
		*A = confidentPlanes.front().a;
		*B = confidentPlanes.front().b;
		*C = confidentPlanes.front().c;
		*D = confidentPlanes.front().d;
		*confidence = confidentPlanes.front().confidence;
		*needsInvert = confidentPlanes.front().needsInvert;
	} else {
		*A = 0;
		*B = 0;
		*C = 0;
		*D = 0;
		*confidence = -1;
		*needsInvert = false;
	}
}
void RANSAC::resetRANSAC() {
	confidentPlanes.erase(confidentPlanes.begin(), confidentPlanes.end());
}

PointXYZ RANSAC::determinePlaneRotation()
{
	return PointXYZ();
}
