//
//  JRObjectHelpers.h
//  Kinect Object Detector
//
//  Created by James Reuss on 20/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#ifndef Kinect_Object_Detector_JRObjectHelpers_h
#define Kinect_Object_Detector_JRObjectHelpers_h

#include <stdio.h>
#include <stdbool.h>
#include "JRKinectHelpers.h"

#define PI_VAL 3.141592654
//(103993/33102)

typedef struct {
	double a, b, c, d, confidence;
} PlaneCoefficients;
typedef PlaneCoefficients _ransacConfidentPlane;
typedef struct {XYZPoint point1; XYZPoint point2; XYZPoint point3; double a; double b; double c; double d; bool isValid;} ABCDPlane;

double zWorldFromPlaneAndWorldXY(_ransacConfidentPlane plane, double worldX, double worldY);
double xWorldFromPlaneAndWorldYZ(_ransacConfidentPlane plane, double worldY, double worldZ);
double yWorldFromPlaneAndWorldXZ(_ransacConfidentPlane plane, double worldX, double worldZ);

#endif
