//
//  JRPointInclusion.cpp
//  Object Detector
//
//  Created by James Reuss on 28/04/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRPointInclusion.h"

PointXY projectToPlane(PointXYZ point, PlaneCoefficients plane)
{
	double t, projX, projY/*, projZ*/ = 0;
	
	// Find the projected coordinates.
	// http://stackoverflow.com/questions/7565748/3d-orthogonal-projection-on-a-plane
	// t = (dot(P,n)+d)/dot(n,n)
	t  = plane.a * point.x;
	t += plane.b * point.y;
	t += plane.c * point.z;
	t += plane.d;
	t /= pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2);
	
	// Q = P - n*t
	projX = point.x - plane.a * t;
	projY = point.y - plane.b * t;
	//projZ = point.z - plane.c * t;	// not required.
	
	return PointXY(projX, projY);
}

double pointPlaneDistance(PointXYZ point, PlaneCoefficients plane)
{
	// Process all the points to get i and j.
	double dist = 0;
	
	// Find the distance from point to plane.
	// http://mathworld.wolfram.com/Point-PlaneDistance.html
	dist  = point.x * plane.a;
	dist += point.y * plane.b;
	dist += point.z * plane.c;
	dist += plane.d;
	dist /= sqrt(pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2));
	
	return dist;
}

/* -----------------------------------------------------------------------------------------------------------------
 FOR THE BELOW FUNCTION ONLY - Modified lightly for use in this program.
 
 SOURCE: http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 
 Copyright (c) 1970-2003, Wm. Randolph Franklin
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
 the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
 and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
 Redistributions of source code must retain the above copyright notice, this list of conditions and the 
 following disclaimers.
 Redistributions in binary form must reproduce the above copyright notice in the documentation and/or other 
 materials provided with the distribution.
 The name of W. Randolph Franklin may not be used to endorse or promote products derived from this Software 
 without specific prior written permission.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 IN THE SOFTWARE.
 ------------------------------------------------------------------------------------------------------------------- */
bool PointInclusion::pointWithinHull(PointXY point)
{
	bool result = false;
	double i, j = 0;
	
	for (i = 0, j = hull.size()-1; i < hull.size(); j = i++) {
		
		if ( ((hull[i].y > point.y) != (hull[j].y > point.y)) && ( point.x < (hull[j].x-hull[i].x) * (point.y-hull[i].y) / (hull[j].y-hull[i].y) + hull[i].x ) ) {
			result = !result;
		}
		
	}
	return result;
}

void PointInclusion::setCloud(vector<PointXYZ> *newCloud)
{
	cloud = newCloud;
	excluded_indices.indices.erase(excluded_indices.indices.begin(), excluded_indices.indices.end());
}

void PointInclusion::setPlane(PlaneCoefficients *newPlane)
{
	plane = newPlane;
	
	// Determine whether this plane gives distances in the
	// correct orientation and therefore, if the distances
	// should be inverted later on.
	// Give the equation a point that we know is very close
	// to the camera.
	double distance = pointPlaneDistance(PointXYZ(0.0, 0.0, 10.0), *plane);
	if (distance < 0) {
		invert_plane = true;
	} else {
		invert_plane = false;
	}
}

void PointInclusion::setHull(PointIndices *newHull)
{
	// These should already be excluded when the
	//plane indices are passed in to be excluded.
	//excludeIndices(newHull);
	
	// Find the projected 2D coordinates of all the hull points.
	hull.erase(hull.begin(), hull.end());
	for (size_t i = 0; i < (*newHull).indices.size(); i++) {
		hull.emplace_back(projectToPlane((*cloud)[(*newHull).indices[i]], *plane));
	}
}

void PointInclusion::excludeIndices(PointIndices *exclude)
{
	for (size_t i = 0; i < (*exclude).indices.size(); i++) {
		excluded_indices.indices.emplace_back((*exclude).indices[i]);
	}
	sort(excluded_indices.indices.begin(), excluded_indices.indices.end());
}

void PointInclusion::run()
{
	cout << "PointInclusion::run(). Starting." << endl;
	
	included_indices.indices.erase(included_indices.indices.begin(), included_indices.indices.end());
	
	// Iterate through every point in the cloud.
	// Check whether the current index is in the excluded indices.
	// If it is then ignore this point, otherwise find out if the point
	// lies within the hull.
	
	size_t exclude_i = 0;
	
	for (size_t i = 0; i < (*cloud).size(); i++) {
		
		// Is the index in excluded_indices?
		if (excluded_indices.indices[exclude_i] == i) {
			
			// This index is found in excluded_indices!
			// Don't process this one and increment the exclude counter.
			exclude_i++;
			
		} else {
			
			// This index was not found in excluded_indices. So let's see
			// if it is within the hull or not.
			
			// Project the point to the plane and get the 2D equivalent.
			PointXY thisPoint = projectToPlane((*cloud)[i], *plane);
			
			// If the point is within the distance tolerance then carry on.
			double dist = pointPlaneDistance((*cloud)[i], *plane);
			dist = (invert_plane == true) ? -dist : dist;
			
			if (dist >= 0 && dist < distance_tolerance) {
				
				// This point is within the tolerance and above the table!
				// Now find out if the projected point is within the hull.
				
				if (pointWithinHull(thisPoint)) {
					
					// This point is where we want it! So add it to the
					// included_indices.
					included_indices.indices.emplace_back(i);
					
				}
				
			}
			
		}
		
	}
	
	cout << "PointInclusion::run(). Ended with " << included_indices.indices.size() << " points inside the area." << endl;
}

