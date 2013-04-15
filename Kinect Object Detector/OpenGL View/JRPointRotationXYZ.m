//
//  JRPointRotationXYZ.m
//  Kinect Object Detector
//
//  Created by James Reuss on 15/04/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRPointRotationXYZ.h"

@implementation JRPointRotationXYZ

- (id)initWithRotationX:(double)x Y:(double)y Z:(double)z
{
    self = [super init];
    if (self) {
		NSLog(@"%@", NSStringFromSelector(_cmd));
        rotation = (PointXYZ){x, y, z};
    }
    return self;
}
- (id)init
{
    return [self initWithRotationX:0 Y:0 Z:0];
}

- (void)rotateByX:(double)alphaY Y:(double)alphaX Z:(double)alphaZ
{
	NSLog(@"%@ x:%f  y:%f  z:%f", NSStringFromSelector(_cmd), alphaX, alphaY, alphaZ);
	// Determine the alphas by scaling to the unit circle.
	//double theta = tan(alphaY/alphaX);
	//alphaX = 1 * acos(theta);
	//alphaY = 1 * asin(theta);
	
	// Compute the rotation matrix
	// Source: http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Conversion_formulae_between_formalisms
	// ⎡ 11  12  13 ⎤	⎡ cos(ay)cos(az)	cos(ax)sin(az)+sin(ax)sin(ay)cos(az)	sin(ax)sin(az)-cos(ax)sin(ay)cos(az) ⎤
	// ⎜ 21  22  23 ⎟ = ⎜ -cos(ay)sin(az)	cos(ax)cos(az)-sin(ax)sin(ay)sin(az)	sin(ax)cos(az)-cos(ax)sin(ay)sin(az) ⎟
	// ⎣ 31  32  33 ⎦	⎣	sin(ay)					-sin(ax)cos(ay)							cos(ax)cos(ay)				 ⎦
	double matrix[3][3];
	matrix[0][0] =  cos(alphaY) * cos(alphaZ);
	matrix[0][1] =  cos(alphaX) * sin(alphaZ) + sin(alphaX) * sin(alphaY) * cos(alphaZ);
	matrix[0][2] =  sin(alphaX) * sin(alphaZ) - cos(alphaX) * sin(alphaY) * cos(alphaZ);
	matrix[1][0] = -cos(alphaY) * sin(alphaZ);
	matrix[1][1] =  cos(alphaX) * cos(alphaZ) - sin(alphaX) * sin(alphaY) * sin(alphaZ);
	matrix[1][2] =  sin(alphaX) * cos(alphaZ) - cos(alphaX) * sin(alphaY) * sin(alphaZ);
	matrix[2][0] =  sin(alphaY);
	matrix[2][1] = -sin(alphaX) * cos(alphaY);
	matrix[2][2] =  cos(alphaX) * cos(alphaY);
	
	printf("\t [0][0] = %f\t", matrix[0][0]);	printf("\t [0][1] = %f\t", matrix[0][1]);	printf("\t [0][2] = %f\n", matrix[0][2]);
	printf("\t [1][0] = %f\t", matrix[1][0]);	printf("\t [1][1] = %f\t", matrix[1][1]);	printf("\t [1][2] = %f\n", matrix[1][2]);
	printf("\t [2][0] = %f\t", matrix[2][0]);	printf("\t [2][1] = %f\t", matrix[2][1]);	printf("\t [2][2] = %f\n", matrix[2][2]);
	
	// Compute the new rotation values
	// ⎡ x ⎤   ⎡ 11  12  13 ⎤   ⎡ x ⎤
	// ⎜ y ⎟ = ⎜ 21  22  23 ⎟ * ⎜ y ⎟
	// ⎣ z ⎦   ⎣ 31  32  33 ⎦   ⎣ z ⎦
	//rotation.x += matrix[0][0]*rotation.x + matrix[0][1]*rotation.y + matrix[0][2]*rotation.z;
	//rotation.y += matrix[1][0]*rotation.x + matrix[1][1]*rotation.y + matrix[1][2]*rotation.z;
	//rotation.z += matrix[2][0]*rotation.x + matrix[2][1]*rotation.y + matrix[2][2]*rotation.z;
	
	//rotation.x = matrix[0][0] + matrix[0][1] + matrix[0][2];
	//rotation.y = matrix[1][0] + matrix[1][1] + matrix[1][2];
	//rotation.z = matrix[2][0] + matrix[2][1] + matrix[2][2];
	
#define divide(a, b) ((b == 0) ? 0 : (a / b))
	//rotation.x += matrix[0][0]*divide(1,rotation.x) + matrix[0][1]*divide(1,rotation.y) + matrix[0][2]*divide(1,rotation.z);
	//rotation.y += matrix[1][0]*divide(1,rotation.x) + matrix[1][1]*divide(1,rotation.y) + matrix[1][2]*divide(1,rotation.z);
	//rotation.z += matrix[2][0]*divide(1,rotation.x) + matrix[2][1]*divide(1,rotation.y) + matrix[2][2]*divide(1,rotation.z);
	
	//rotation.x += divide(alphaX, matrix[0][0]) + divide(alphaY, matrix[0][1]) + divide(alphaZ, matrix[0][2]);
	//rotation.y += divide(alphaX, matrix[1][0]) + divide(alphaY, matrix[1][1]) + divide(alphaZ, matrix[1][2]);
	//rotation.z += divide(alphaX, matrix[2][0]) + divide(alphaY, matrix[2][1]) + divide(alphaZ, matrix[2][2]);
	
	rotation.x += matrix[0][0]*alphaX + matrix[0][1]*alphaY + matrix[0][2]*alphaZ;
	rotation.y += matrix[1][0]*alphaX + matrix[1][1]*alphaY + matrix[1][2]*alphaZ;
	rotation.z += matrix[2][0]*alphaX + matrix[2][1]*alphaY + matrix[2][2]*alphaZ;
	
	printf("\t rotX = %f\n", rotation.x);
	printf("\t rotY = %f\n", rotation.y);
	printf("\t rotZ = %f\n", rotation.z);
}

- (PointXYZ)rotation
{
	//NSLog(@"%@", NSStringFromSelector(_cmd));
	return rotation;
}

@end
