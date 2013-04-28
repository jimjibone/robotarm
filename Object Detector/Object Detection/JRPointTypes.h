//
//  JRPointTypes.h
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#ifndef Kinect_Object_Detector_JRPointTypes_h
#define Kinect_Object_Detector_JRPointTypes_h



#ifdef __cplusplus


#include <iostream>
#include <vector>
#include <libfreenect/libfreenect.h>

struct PointIndices {
	std::vector<size_t> indices;
	PointIndices(std::vector<size_t> _indices) : indices(_indices) {};
	PointIndices(size_t index) { indices.emplace_back(index); };
	PointIndices() { /*indices.erase(indices.begin(), indices.end());*/ };
};

struct PointIXYZ {
	uint index;
	double x, y, z;
	bool isValid() { return (z > 0) && (z < 10000); }
	bool operator <(const PointIXYZ &point) const {
		return index < point.index;
	}
	PointIXYZ(uint _index = 0, double _x = 0, double _y = 0, double _z = 0) : index(_index), x(_x), y(_y), z(_z) {};
	PointIXYZ(double _x = 0, double _y = 0, double _z = 0) : index(0), x(_x), y(_y), z(_z) {};
};

struct PointXYZ {
	double x, y, z;
	bool isValid() { return (z > FREENECT_DEPTH_MM_NO_VALUE) && (z < FREENECT_DEPTH_MM_MAX_VALUE); }
	PointXYZ(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {};
};

struct PointXYZIJ {
	double x, y, z;	// 3D points from the original->projectd-to-plane data.
	double i, j;	// 2D points once 3D->2D conversion has been completed.
	bool isValid() { return (z > FREENECT_DEPTH_MM_NO_VALUE) && (z < FREENECT_DEPTH_MM_MAX_VALUE); }
	bool operator <(const PointXYZIJ &point) const {
		return i < point.i || (i == point.i && j < point.j);
	}
	PointXYZIJ(double _x = 0, double _y = 0, double _z = 0, double _i = 0, double _j = 0) : x(_x), y(_y), z(_z), i(_i), j(_j) {};
	PointXYZIJ() { z = FREENECT_DEPTH_MM_NO_VALUE; };
};

struct PointNXYZIJ {
	uint index;
	double x, y, z;	// 3D points from the original->projectd-to-plane data.
	double i, j;	// 2D points once 3D->2D conversion has been completed.
	bool isValid() { return (z > FREENECT_DEPTH_MM_NO_VALUE) && (z < FREENECT_DEPTH_MM_MAX_VALUE); }
	bool operator <(const PointNXYZIJ &point) const {
		return i < point.i || (i == point.i && j < point.j);
	}
	PointNXYZIJ(uint _index = 0, double _x = 0, double _y = 0, double _z = FREENECT_DEPTH_MM_NO_VALUE, double _i = 0, double _j = 0) : index(_index), x(_x), y(_y), z(_z), i(_i), j(_j) {};
};

struct PlaneCoefficients {
	double a, b, c, d;
	bool isSet() {
		return (fabs(a) > 0) || (fabs(b) > 0) || (fabs(c) > 0) || (fabs(d) > 0);
	}
	double xFromYZ(double y, double z) {
		// ax + by + cz + d = 0
		// -(by + cz + d)/a = x
		return -(b*y + c*z + d)/a;
	}
	double yFromXZ(double x, double z) {
		// ax + by + cz + d = 0
		// -(ax + cz + d)/b = y
		return -(a*x + c*z + d)/b;
	}
	double zFromXY(double x, double y) {
		// ax + by + cz + d = 0
		// -(ax + by + d)/c = z
		return -(a*x + b*y + d)/c;
	}
	PlaneCoefficients(double _a = 0, double _b = 0, double _c = 0, double _d = 0) : a(_a), b(_b), c(_c), d(_d) {};
};

template <typename PointT>
struct PointCloud {
	std::vector<PointT> points;
	void eraseAll() {
		points.erase(points.begin(), points.end());
	}
	void addPoint(PointT aPoint) {
		points.emplace_back(aPoint);
	}
	size_t size() {
		printf("template PointCloud.size(); = %ld\n", points.size());
		return points.size();
	}
};


#else


typedef struct {
	//uint index;
	double x, y, z;
} PointXYZ;

typedef struct {
	double x, y, z;	// 3D points from the original->projectd-to-plane data.
	double i, j;	// 2D points once 3D->2D conversion has been completed.
} PointXYZIJ;

typedef struct {
	double a, b, c, d;
	bool isValid;
	double confidence;
	bool needsInvert;	// determines whether a calculated distance value should be inverted.
} PlaneCoefficients;


#endif

#endif
