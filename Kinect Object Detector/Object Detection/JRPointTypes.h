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

struct PointXYZ {
	uint index;
	double x, y, z;
	bool isValid() { return (z > 0) && (z < 10000); }
	bool operator <(const PointXYZ &point) const {
		return index < point.index;
	}
	PointXYZ(uint _index = 0, double _x = 0, double _y = 0, double _z = 0) : index(_index), x(_x), y(_y), z(_z) {};
};

struct PointXYZIJ {
	double x, y, z;	// 3D points from the original->projectd-to-plane data.
	double i, j;	// 2D points once 3D->2D conversion has been completed.
	bool isValid() { return (z > 0) && (z < 10000); }
	bool operator <(const PointXYZIJ &point) const {
		return i < point.i || (i == point.i && j < point.j);
	}
	PointXYZIJ(double _x = 0, double _y = 0, double _z = 0, double _i = 0, double _j = 0) : x(_x), y(_y), z(_z), i(_i), j(_j) {};
};

struct PlaneCoefficients {
	double a, b, c, d;
	bool isValid;
	double confidence;
	bool operator <(const PlaneCoefficients &plane) const {
		return confidence >= plane.confidence;
	}
	bool isSet() {
		return a || b || c || d;
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
	void setCoefficients(double _a, double _b, double _c, double _d) {
		a = _a;
		b = _b;
		c = _c;
		d = _d;
	}
	PlaneCoefficients(double _a = 0, double _b = 0, double _c = 0, double _d = 0, bool _isValid = false, double _confidence = 0) : a(_a), b(_b), c(_c), d(_d), isValid(_isValid), confidence(_confidence) {};
};



#endif
