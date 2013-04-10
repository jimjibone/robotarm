//
//  JRObjectDetector.h
//  Kinect Object Detector
//
//  Created by James Reuss on 10/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "JRKinectController.h"
#import "GLView.h"
#import "JRObjectHelpers.h"

@protocol JRObjectDetectorDelegate
@required
- (void)objectDetectorDidCompleteRANSAC;
- (void)objectDetectorDidCompleteEdgeFinding;
@end

@class GLView;
@interface JRObjectDetector : NSObject {
	JRKinectController *kinectController;
	GLView *glView;
	
	BOOL _runMainMethod;
	BOOL _mainMethodCompleted;
	
	uint16_t *_kinectDepth;
	uint8_t  *_kinectRGB;
	
	
	
	// RANSAC
	struct {
		unsigned int *possibleInlierIndices;
		unsigned int *nonInlierIndices;
		ABCDPlane *possiblePlanes;
		unsigned int noValidPlanes;
		unsigned int confidentPlaneNo;		// Most confident plane number from the current depth frame
		int confidentPlaneVal;				// Most confident value from the current depth frame
		ABCDPlane *confidentPlanes;			// Most confident planes from the latest few depth frames
		ABCDPlane tablePlane;				// Finalised confident (table) plane after averaging
		unsigned int confidentPlanesCount;
	} _ransac;
	
	BOOL _showPlaneChanged;
	BOOL _showPlane;
	unsigned int _showPlaneNo;
	BOOL _resetRANSAC;
	
	
	// Convex Hull Data
	struct {
		unsigned int *tableIndices;		// Should match tableProjections.
		unsigned int tableIndiciesCount;
		//IJXYZPoint *tableProjections;	//should match tableIndices.
		XYZPoint *tablePoints;			// Should match tableIndices.
		unsigned int *cHullIndices;		// Should match tableProjections.
		unsigned int cHullIndexCount;
	} _cHull;
	
	BOOL _resetConvexHull;
}
@property (readonly) BOOL isRunning;
@property (nonatomic, retain) id <JRObjectDetectorDelegate> delegate;

- (void)start;
- (void)stop;

- (void)resetRANSACCapture;

- (void)showPlaneDataForSet:(unsigned int)setNo;
- (void)hidePlaneData;

- (void)setGLViewOutlet:(GLView*)newGLView;
- (void)setKinectControllerDelegate:(id<JRKinectControllerDelegate>)delegate;

@end
