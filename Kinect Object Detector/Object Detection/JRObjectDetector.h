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
#import "JRConvexHullWrapper.h"
#import "JRRANSACWrapper.h"

@class GLView;
@interface JRObjectDetector : NSObject {
	JRKinectController *kinectController;
	GLView *glView;
	
	BOOL _runMainMethod;
	BOOL _mainMethodCompleted;
	
	uint16_t *_kinectDepth;
	uint8_t  *_kinectRGB;
	
	// RANSAC
	JRRANSACWrapper *ransac;
	_ransacConfidentPlane ransacConfidentPlane;
	BOOL ransacReset;
	uint ransacIterations;
	BOOL ransacComplete;
	BOOL ransacShow;
	BOOL ransacShowChanged;
	
	// Convex Hull Data
	JRConvexHullWrapper *convexHull;
	BOOL convexHullReset;
	BOOL convexHullComplete;
	BOOL convexHullShow;
	BOOL convexHullShowChanged;
	
}
@property (readonly) BOOL isRunning;

- (void)start;
- (void)stop;

- (JRKinectController*)getKinectController;

- (void)resetTableDetection;
- (BOOL)setPlaneViewingState:(BOOL)aState;
- (BOOL)setConvexHullViewingState:(BOOL)aState;

- (void)setGLViewOutlet:(GLView*)newGLView;

@end
