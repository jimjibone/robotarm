//
//  JRRANSAC.h
//  RANSAC CPP
//
//  Created by James Reuss on 03/02/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef __RANSAC_CPP__JRRANSAC__
#define __RANSAC_CPP__JRRANSAC__

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

class RANSAC {
	struct Point {
		uint index;
		double x, y, z;
		bool isValid() { return (z > 0) && (z < 10000); }
		bool operator <(const Point &point) const {
			return index < point.index;
		}
		Point(uint _index = 0, double _x = 0, double _y = 0, double _z = 0) : index(_index), x(_x), y(_y), z(_z) {};
	};
	struct Plane {
		double a, b, c, d;
		bool isValid;
		double confidence;
		bool operator <(const Plane &plane) const {
			return confidence >= plane.confidence;
		}
		Plane(double _a = 0, double _b = 0, double _c = 0, double _d = 0, bool _isValid = false, double _confidence = 0) : a(_a), b(_b), c(_c), d(_d), isValid(_isValid), confidence(_confidence) {};
	};
	
	vector<Point> randomPoints;		// The points to be used for possible inliers.
	vector<Point> otherPoints;		// All the other points available from the Kinect.
	uint maxPoints;					// The maximum number of points in the data.
	uint maxRandomPoints;			// The maximum number of random points to be used.
	double distanceTolerance;		// The tolerance to use when deciding if a point is on a plane or not.
	vector<Plane> planes;			// The planes used in performRANSAC to find the most confident.
	vector<Plane> confidentPlanes;	// A vector of the most confident planes added after each call of performRANSAC.
	
	// Function to get uniformly distributed random integers.
	int irand(int min, int max) { return ((double)rand() / ((double)RAND_MAX + 1.0)) * (max - min + 1) + min; }
	// Function to get the world x & y coordinates using just pixel index and depth value.
	void worldFromIndex(double* wx, double* wy, unsigned int index, int z) {
		unsigned int x = index % 640;
		unsigned int y = (index - x) / 640.0;
		*wx = (double)((x - 640/2.0)*(z + 0.0)*0.00174);
		*wy = (double)((y - 480/2.0)*(z + 0.0)*0.00174*-1.0);
	}
	// Function to get the plane equation from 3 points in 3D space.
	Plane getPlane(Point a, Point b, Point c) {
		// First validate the input to check that Z values are not out-of-bounds of Kinect view.
		if (!a.isValid() || !b.isValid() || !c.isValid()) {
			return Plane(0, 0, 0, 0, false, 0);
		}
		
		//http://keisan.casio.com/has10/SpecExec.cgi# or
		//http://www.easycalculation.com/analytical/cartesian-plane-equation.php
		double Pa = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
		double Pb = (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x);
		double Pc = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
		double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
		
		return Plane(Pa, Pb, Pc, Pd, true, 0);
	}
	// Function to list the planes that are input via vector.
	void listPlanes(vector<Plane> *planes) {
		cout << "Listing planes. Total number = " << planes->size() << endl;
		uint count = 0;
		for (Plane const& plane : *planes) {
			cout << "\tPlane " << count << " : A:" << plane.a << " B:" << plane.b << " C:" << plane.c<< " D:" << plane.d << " isValid:" << plane.isValid << " confidence:" << plane.confidence << endl;
			count++;
		}
	}
	// Function to find the distance between a point and a plane.
	double findAbsoluteDistance(Point point, Plane plane) {
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
	
public:
	RANSAC(uint newMaxPoints, uint newRandomPointCount, double newDistanceTolerance = 0);
	~RANSAC();
	void listRandomPoints();
	void listOtherPoints();
	void listFullSet();
	void listConfidentPlanes();
	void updateDepthData(uint16_t* newDepth);
	void performRANSAC();
	void prepareRANSAC();
	void getConfidentPlane(double *A, double *B, double *C, double *D, double *confidence);
	void resetRANSAC();
};

#endif /* defined(__RANSAC_CPP__JRRANSAC__) */
