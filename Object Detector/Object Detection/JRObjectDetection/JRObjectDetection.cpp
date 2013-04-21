//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

#pragma mark - 
#pragma mark Helping Functions

PointXYZ worldFromIndex(size_t index, int z)
{
	double x = index % FREENECT_FRAME_W;
	double y = (index - x) / FREENECT_FRAME_W;
	x = (double)((x - FREENECT_FRAME_W/2.0) * z * 0.00174);
	y = (double)((y - FREENECT_FRAME_H/2.0) * z * -0.00174);
	return PointXYZ(x, y, z);
}

// Function to get the plane equation from 3 points in 3D space.
PlaneCoefficients getPlaneCoefficients(PointXYZ a, PointXYZ b, PointXYZ c)
{
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

double dot(PointXYZ a, PointXYZ b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double dot(PlaneCoefficients a, PlaneCoefficients b)
{
	return a.a*b.a + a.b*b.b + a.c*b.c;
}

// To normalise a normal equation you can do this
//	CombinedSquares = (Normal.x * Normal.x) +
//	(Normal.y * Normal.y) +
//	(Normal.z * Normal.z);
//	NormalisationFactor = sqrt(CombinedSquares);
//	Normal.x = Normal.x / NormalisationFactor;
//	Normal.y = Normal.y / NormalisationFactor;
//	Normal.z = Normal.z / NormalisationFactor;





#pragma mark -
#pragma mark Private Functions

bool ObjectDetection::compareAngle(size_t a, size_t b)
{
	const float thresholdDistance = 5; // 5mm.
	const float thresholdAngle = PI_VALUE/36; // 5 deg in radians.
	
	// Compare the difference between the d component of the two points.
	bool distance = fabs(input_cloud_normals[a].d - input_cloud_normals[b].d) < thresholdDistance;
	// Compare the angle between normal vectors by doing the dot product.
	// Also this but not used [ theta = acos ( a . b / ||a|| ||b|| ) ].
	bool angle = dot(input_cloud_normals[a], input_cloud_normals[b]) < thresholdAngle;
	return distance && angle;
}





#pragma mark -
#pragma mark Object Detection Functions

void ObjectDetection::setZPoints(uint16_t* zPoints, size_t size)
{
	// Remove all points from the input_cloud.
	input_cloud.erase(input_cloud.begin(), input_cloud.end());
	
	// Iterate through all the new points and add them to the input_cloud
	// also calculate the world x and y values on input.
	for (size_t i = 0; i < size; i++) {
		input_cloud.emplace_back(worldFromIndex(i, zPoints[i]));
	}
}

void ObjectDetection::calculateSurfaceNormals()
{
	// Iterate through all the points in the input_cloud and calcualte
	// the surface normals of each point.
	// First, remove all the current normals.
	input_cloud_normals.erase(input_cloud_normals.begin(), input_cloud_normals.end());
	
	// Some points will need to be skipped as the normals are calculated
	// by using 2 neighboring points to the right and below. Therefore the
	// right-most and bottom-most pixels will not have a complete set of
	// neighbours.
	for (size_t i = 0; i < input_cloud.size(); i++) {
		uint x = 0, y = 0;
		frameXYfromIndex(&x, &y, (uint)i);
		
		if (x == 0 && i > 0 // is right-hand side (and not i=0)
			&& y == FREENECT_FRAME_H // and is bottom
			) {
			// This point is out-of-bounds! Set the normal to 0.
			input_cloud_normals.emplace_back(PlaneCoefficients(0, 0, 0, 0));
		} else {
			// This point is within bounds. Calculate the normal.
			input_cloud_normals.emplace_back(getPlaneCoefficients(input_cloud[i],
																  input_cloud[i+1],
																  input_cloud[i+FREENECT_FRAME_W]));
		}
	}
}

void ObjectDetection::segmentPlanes()
{
	// Create a vector containing all the indices to be clustered.
	vector<size_t> non_clustered;
	for (size_t i = 0; i < input_cloud_normals.size(); i++) {
		non_clustered.emplace_back(i);
	}
	
	// Remove all the previous plane clusters.
	plane_clusters.erase(plane_clusters.begin(), plane_clusters.end());
	
	// Iterate through each point in non_clustered and find which cluster it belongs to.
	for (size_t i = 0; i < non_clustered.size(); i++) {
		// Add the first point to the plane_clusters
		PointIndices current_cluster;
		current_cluster.indices.emplace_back(non_clustered[i]);
		//non_clustered.erase(const_cast<uint>(i));
		non_clustered.erase(non_clustered.begin()+1);
	}
}


