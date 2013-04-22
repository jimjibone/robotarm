//
//  JRObjectDetector.h
//  Kinect Object Detector
//
//  Created by James Reuss on 10/11/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "JRKinectController.h"
#import "GLView.h"
#import "JRObjectHelpers.h"
#import "JRObjectDetectionWrapper.h"
#import "JRPointTypes.h"

@class GLView;
@interface JRObjectDetector : NSObject {
	JRKinectController *kinectController;
	GLView *glView;
	
	BOOL _runMainMethod;
	BOOL _mainMethodCompleted;
	
	uint16_t *_kinectDepth;
	uint8_t  *_kinectRGB;
	
	// Object Detection
	JRObjectDetectionWrapper *objectDetector;
	BOOL objectDetectionReset;
	BOOL objectDetectionComplete;
	
}
@property (readonly) BOOL isRunning;

- (void)start;
- (void)stop;

- (JRKinectController*)getKinectController;

- (void)resetTableDetection;

- (void)setGLViewOutlet:(GLView*)newGLView;

//- (NSArray*)getConvexHullPoints;

@end
