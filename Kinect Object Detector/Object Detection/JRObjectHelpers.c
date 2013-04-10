//
//  JRObjectHelpers.c
//  Kinect Object Detector
//
//  Created by James Reuss on 20/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#include <stdio.h>
#include "JRObjectHelpers.h"

// ax + by + cz + d = 0
double zWorldFromPlaneAndWorldXY(_ransacConfidentPlane plane, double worldX, double worldY) {
	// z = -(ax + by + d)/c
	return -(plane.a*worldX + plane.b*worldY + plane.d)/plane.c;
}
double xWorldFromPlaneAndWorldYZ(_ransacConfidentPlane plane, double worldY, double worldZ) {
	// x = -(by + cz + d)/a
	return -(plane.b*worldY + plane.c*worldZ + plane.d)/plane.a;
}
double yWorldFromPlaneAndWorldXZ(_ransacConfidentPlane plane, double worldX, double worldZ) {
	// y = -(ax + cz + d)/b
	return -(plane.a*worldX + plane.c*worldZ + plane.d)/plane.b;
}
