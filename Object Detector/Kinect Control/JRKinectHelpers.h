//
//  JRKinectHelpers.h
//  KinectiCopter Mark 2
//
//  Created by James Reuss on 02/06/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#ifndef KinectiCopter_Mark_2_KCHelperFunctions_h
#define KinectiCopter_Mark_2_KCHelperFunctions_h

#include <stdio.h>
#include <math.h>
#include <libfreenect/libfreenect.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480
#define FREENECT_FRAME_PIX (FREENECT_FRAME_H*FREENECT_FRAME_W)

#define FREENECT_DEPTH_11BIT_SIZE (FREENECT_FRAME_PIX*sizeof(uint16_t))
#define FREENECT_VIDEO_RGB_SIZE (FREENECT_FRAME_PIX*3)


/* *** DEPRECIATED     */
#pragma depreciated(KCVector, KCLocation, KCBounds, MIN_DEPTH, SCALE_FACTOR)
typedef struct {
	double x, y, z;
} KCVector3;

typedef struct {
	KCVector3 roomLocation;
	KCVector3 estimateLocation;
	KCVector3 frameLocation;
	double yawAngle;
} KCLocation;

typedef struct {
	double left, right, top, bottom, back;
	double floor;
} KCBounds;

#define MIN_DEPTH    0
#define SCALE_FACTOR 0.00174
/*     DEPRECIATED *** */


typedef struct {double x; double y; double z;} XYZPoint;
typedef struct {double i; double j; double x; double y; double z;} IJXYZPoint;
typedef unsigned int uint;

#define REGISTERED_MIN_DEPTH    0
#define REGISTERED_SCALE_FACTOR 0.00174
double worldXfromFrame(int x, int z);
double worldYfromFrame(int y, int z);

unsigned int depthIndex(unsigned int x, unsigned int y);
unsigned int rgbIndex(unsigned int x, unsigned int y);
void frameXYfromIndex(unsigned int* x, unsigned int* y, unsigned int index);
void worldFromIndex(double* wx, double* wy, unsigned int index, int z);
void indexFromWorld(unsigned int* index, double wx, double wy, int z);
void frameFromWorld(unsigned int* x, unsigned int *y, double wx, double wy, int z);

void swapPtr16(uint16_t **firstPtr, uint16_t **secondPtr);
void swapPtr8(uint8_t **firstPtr, uint8_t **secondPtr);

#ifdef __cplusplus
}
#endif

#endif
