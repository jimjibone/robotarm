//
//  JRConvexHull.cpp
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRConvexHull.h"

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double ConvexHull::cross(const PointNXYZIJ &o, const PointNXYZIJ &a, const PointNXYZIJ &b) {
	return (a.i - o.i) * (double)(b.j - o.j) - (a.j - o.j) * (double)(b.i - o.i);
}

void ConvexHull::projectPoints()
{
	// Remove all the current points. Ready for all the
	// calculations below.
	projected_cloud.erase(projected_cloud.begin(), projected_cloud.end());
	
	// So, iterate through all the points that are indicated
	// by the indices.
	for (size_t i = 0; i < indices->indices.size(); i++) {
		
		// Process all the points to get i and j.
		double dist, t, projX, projY, projZ, newI, newJ = 0;
		
		// Find the distance from point to plane.
		// http://mathworld.wolfram.com/Point-PlaneDistance.html
		dist  = cloud->at(indices->indices[i]).x * plane->a;
		dist += cloud->at(indices->indices[i]).y * plane->b;
		dist += cloud->at(indices->indices[i]).z * plane->c;
		dist += plane->d;
		dist /= sqrt(pow(plane->a, 2) + pow(plane->b, 2) + pow(plane->c, 2));
		
		dist = (dist < 0) ? -dist : dist;	// Absolute distance please.
		
		if (dist <= distance_tolerance) {
			
			// The point is inside the tolerance! So let's add it
			// otherwise don't bother adding it to the point list.
			
			// Find the projected coordinates.
			// http://stackoverflow.com/questions/7565748/3d-orthogonal-projection-on-a-plane
			// t = (dot(P,n)+d)/dot(n,n)
			t  = plane->a * cloud->at(indices->indices[i]).x;
			t += plane->b * cloud->at(indices->indices[i]).y;
			t += plane->c * cloud->at(indices->indices[i]).z;
			t += plane->d;
			t /= pow(plane->a, 2) + pow(plane->b, 2) + pow(plane->c, 2);
			
			// Q = P - n*t
			projX = cloud->at(indices->indices[i]).x - plane->a * t;
			projY = cloud->at(indices->indices[i]).y - plane->b * t;
			projZ = cloud->at(indices->indices[i]).z - plane->c * t;
			
			// Find the 2D x,y coordinates relating to the plane.
			// For this case just "flatten" by ignoring Z.
			newI = projX;//sqrt(pow(projX, 2) + z2);
			newJ = projY;//sqrt(pow(projY, 2) + z2);
			
			// Add the point to the end of the planePoints vector.
			projected_cloud.emplace_back(indices->indices[i], projX, projY, projZ, newI, newJ);
			
		}
		
	}
	
}

void ConvexHull::setData(vector<PointXYZ> *newCloud, PointIndices *newIndices, PlaneCoefficients *newPlane)
{
	cloud = newCloud;
	indices = newIndices;
	plane = newPlane;
}

void ConvexHull::run()
{
	// Clear all the previously stored convex
	// hull points, if there are any.
	hull_indices.indices.erase(hull_indices.indices.begin(), hull_indices.indices.end());
	vector<PointNXYZIJ> hull_points;
	
	projectPoints();
	
	// Now for some convex hull specific things.
	int n = (int)projected_cloud.size();
	int k = 0;
	hull_points.resize(2*n);
	
	// Sort the points lexicographically.
	sort(projected_cloud.begin(), projected_cloud.end());
	
	// Build the lower hull.
	for (int i = 0; i < n; i++) {
		while (k >= 2 && cross(hull_points[k-2], hull_points[k-1], projected_cloud[i]) <= 0) {
			k--;
		}
		hull_points[k++] = projected_cloud[i];
	}
	
	// Build the upper hull.
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && cross(hull_points[k-2], hull_points[k-1], projected_cloud[i]) <= 0) {
			k--;
		}
		hull_points[k++] = projected_cloud[i];
	}
	
	hull_points.resize(k);
	sort(hull_points.begin(), hull_points.end());
	
	// Now we have all the hull points! So let's
	// fill the indices with the relevant positions.
	for (size_t i = 0; i < hull_points.size(); i++) {
		hull_indices.indices.emplace_back(hull_points[i].index);
	}
	
}
