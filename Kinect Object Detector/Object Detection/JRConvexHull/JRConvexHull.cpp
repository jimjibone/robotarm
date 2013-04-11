//
//  JRConvexHull.cpp
//  Convex Hull Small
//
//  Created by James Reuss on 01/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRConvexHull.h"

ConvexHull::ConvexHull() {
	// Do nothing.
}
ConvexHull::~ConvexHull() {
	// Do nothing.
}
// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double ConvexHull::cross(const ConvexHull::PointCH &o, const ConvexHull::PointCH &a, const ConvexHull::PointCH &b) {
	return (a.i - o.i) * (double)(b.j - o.j) - (a.j - o.j) * (double)(b.i - o.i);
}
void ConvexHull::setPlane(double newA, double newB, double newC, double newD, double newTol) {
	plane.a = newA;
	plane.b = newB;
	plane.c = newC;
	plane.d = newD;
	tolerance = newTol;
}
bool ConvexHull::addPoint(double newX, double newY, double newZ) {
	bool state = false;
	double newI, newJ = 0;
	if (plane.isSet()) {
		// Process the points to get i and j.
		double dist, t, projX, projY, projZ = 0;
		
		// Find the distance from point to plane.
		// http://mathworld.wolfram.com/Point-PlaneDistance.html
		dist  = newX * plane.a;
		dist += newY * plane.b;
		dist += newZ * plane.c;
		dist += plane.d;
		dist /= sqrt(pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2));
		
		dist = (dist >= 0) ? dist : -dist;	// Absolute distance.
		if (dist > tolerance) return false;	// Return false as point is outside of tolerance.
		
		// Find the projected coordinates.
		// http://stackoverflow.com/questions/7565748/3d-orthogonal-projection-on-a-plane
		// t = (dot(P,n)+d)/dot(n,n)
		t  = plane.a * newX;
		t += plane.b * newY;
		t += plane.c * newZ;
		t += plane.d;
		t /= pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2);
		
		// Q = P - n*t
		projX = newX - plane.a * t;
		projY = newY - plane.b * t;
		projZ = newZ - plane.c * t;
		
		// Find the 2D x,y coordinates relating to the plane.
		// For this case just "flatten" by ignoring Z.
		newI = projX;//sqrt(pow(projX, 2) + z2);
		newJ = projY;//sqrt(pow(projY, 2) + z2);
		
		// Add the point to the end of the planePoints vector.
		planePoints.emplace_back(projX, projY, projZ, newI, newJ);
		
		state = true;	// Return true as point has been added to set, also is within tolerance.
		
	}
	return state;
}
void ConvexHull::listPoints() {
	cout << "List of points contained on the plane.\n";
	int count = 0;
	for (PointCH const& point : planePoints) {
		count++;
		printf("\tPoint %2d : X:%9.3f Y:%9.3f Z:%9.3f I:%9.3f J:%9.3f\n", count, point.x, point.y, point.z, point.i, point.j);
	}
}
// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
void ConvexHull::performConvexHull() {
	// Clear all the convex hull points to begin a new scan.
	convexHullPoints.erase(convexHullPoints.begin(), convexHullPoints.end());
	
	int n = (int)planePoints.size();
	int k = 0;
	bool error = false;
	convexHullPoints.resize(2*n);
	
	// Sort the points lexicographically.
	sort(planePoints.begin(), planePoints.end());
	
	// Build the lower hull.
	for (int i = 0; i < n; i++) {
		while (k >= 2 && cross(convexHullPoints[k-2], convexHullPoints[k-1], planePoints[i]) <= 0) {
			k--;
		}
		convexHullPoints[k++] = planePoints[i];
		if (k >= 640*480) { error = true; break; }
	}
	
	// Build the upper hull.
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && cross(convexHullPoints[k-2], convexHullPoints[k-1], planePoints[i]) <= 0) {
			k--;
		}
		convexHullPoints[k++] = planePoints[i];
		if (k >= 640*480) { error = true; break; }
	}
	
	convexHullPoints.resize(k);
	
	if (k >= 640*480 || error) {
		convexHullPoints.clear();
		cout << "Invalid data points. The plane may be erroneous. Deleted all convex hull data.\n";
	}
}
void ConvexHull::listConvexHullPoints() {
	cout << "\n\n\n\n\n\n\n\n\n\n\n\n\nList of points contained in the convex hull, in order.\n";
	int count = 0;
	for (PointCH const& point : convexHullPoints) {
		count++;
		printf("\tPoint %2d : X:%9.3f Y:%9.3f Z:%9.3f I:%9.3f J:%9.3f\n", count, point.x, point.y, point.z, point.i, point.j);
	}
	cout << "The final point is the same as the first point in the convex hull.\n";
}
void ConvexHull::resetConvexHull() {
	planePoints.erase(planePoints.begin(), planePoints.end());
	convexHullPoints.erase(convexHullPoints.begin(), convexHullPoints.end());
}




// These next functions allow the processing a pre-processed convex hull and
// individual points for the use of point-within-bounds calculations.
void ConvexHull::addPreprocessedConvexHullPoint(PointXYZIJ aPoint)
{
	
}
bool ConvexHull::processPointWithPreprocessedHull(double x, double y, double z)
{
	
}
