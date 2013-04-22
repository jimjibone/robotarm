//
//  JRObjectDetector.m
//  Kinect Object Detector
//
//  Created by James Reuss on 10/11/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRObjectDetector.h"
#import "JRConstants.h"

#define TIME_INTERVAL 0.1

@interface JRObjectDetector ()
//- (void)start;
- (void)addTimerForMainMethod;
//- (void)stop;

- (void)mainMethod;

// Data Management
- (BOOL)collectNewKinectFrames;
- (void)sendNewFramesToDisplay;

// Object Detection
- (void)initObjectDetection;
- (void)freeObjectDetection;
- (void)performFindObjectPoints;

// Helpers
@end


@implementation JRObjectDetector

#pragma mark - Init/Dealloc Methods
- (id)init {
    self = [super init];
    if (self) {
		// Setup and start the kinect controller
        kinectController = [[JRKinectController alloc] initWithLEDColour:LED_GREEN];
		
		// Allocate memory for the kinect and display data
		_kinectDepth	= (uint16_t*)malloc(FREENECT_DEPTH_11BIT_SIZE);
		_kinectRGB		= (uint8_t*)malloc(FREENECT_VIDEO_RGB_SIZE);
        
		// Init Object Detection
		[self initObjectDetection];
		
		// Setup the rest of the program ready to start
		_runMainMethod = NO;
		_mainMethodCompleted = YES;
    }
    return self;
}
- (void)start {
	if (_runMainMethod == NO) {
		// Start the main method and to rest of the class
		_runMainMethod = YES;
		_mainMethodCompleted = YES;
		[self addTimerForMainMethod];
	}
}
- (void)addTimerForMainMethod {
	if (_runMainMethod) {
		[[NSRunLoop currentRunLoop] addTimer:[NSTimer timerWithTimeInterval:TIME_INTERVAL
																	 target:self
																   selector:@selector(mainMethod)
																   userInfo:nil
																	repeats:NO]
									 forMode:NSDefaultRunLoopMode];
		_isRunning = YES;
	} else {
		_isRunning = NO;
	}
}
- (void)stop {
	if (_runMainMethod == YES) {
		_runMainMethod = NO;
	}
	[kinectController stopKinectImmediately];
}
- (void)dealloc {
	// Stop the main method from running
	_runMainMethod = NO;
	
	// Stop and release kinect controller
    [kinectController stopKinectImmediately];
	[kinectController release];
	
	// Delete the kinect and display data
	free(_kinectDepth);
	free(_kinectRGB);
	_kinectDepth  = NULL;
	_kinectRGB    = NULL;
	
	// Dealloc RANSAC
	[self freeObjectDetection];
	
    [super dealloc];
}




#pragma mark - Main Methods
- (void)mainMethod {
	
	if (_runMainMethod && _mainMethodCompleted) {
		_mainMethodCompleted = NO;
		
		if ([self collectNewKinectFrames]) {
			
			// Object detection methods
			[self performFindObjectPoints];
			
			[self sendNewFramesToDisplay];
		} else {
			// An error occurred during collecting frames.
			// Not the correct location for this notification!
			/*[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nObjectDetectorDidFailTableDetection
																object:self
															  userInfo:@{dObjectDetectorDidFailTableDetectionString: @"Could not get new Kinect frames"}];*/
		}
	}
	
	if (_runMainMethod) {
		[self addTimerForMainMethod];
	}
	
	_mainMethodCompleted = YES;
}




#pragma mark - Data Management
- (BOOL)collectNewKinectFrames {
	BOOL newData = NO;
	newData = [kinectController swapDepthData:&_kinectDepth];
	newData |= [kinectController swapRGBData:&_kinectRGB];
	return newData;
}
- (void)sendNewFramesToDisplay {
	[glView swapInNewDepthFrame:&_kinectDepth RGBFrame:&_kinectRGB];
}




#pragma mark - 
#pragma mark Object Detection Methods

#define OBJECT_MAX_HEIGHT 500	// mm above table
#define OBJECT_MIN_HEIGHT 10	// mm above table
- (void)initObjectDetection
{
	objectDetector = [[JRObjectDetectionWrapper alloc] init];
	objectDetectionReset = NO;
	objectDetectionComplete = NO;
}
- (void)freeObjectDetection
{
	[objectDetector release];
}
- (void)performFindObjectPoints
{
	if (objectDetectionReset) {
		objectDetectionReset = NO;
		objectDetectionComplete = NO;
		
		[objectDetector setZPoints:_kinectDepth size:FREENECT_FRAME_PIX];
		
		[objectDetector calculateSurfaceNormals];
		[objectDetector segmentPlanes];
		
		objectDetectionComplete = YES;
	}
}




#pragma mark - Private Methods




#pragma mark - Instance Methods
- (JRKinectController*)getKinectController {
	return kinectController;
}
- (void)resetTableDetection {
	objectDetectionReset = YES;
}
- (void)setGLViewOutlet:(GLView*)newGLView {
	if (glView) {
		[glView release];
		glView = NULL;
	}
	glView = newGLView;
}
/*- (NSArray*)getConvexHullPoints {
	NSMutableArray *newArray = [[NSMutableArray alloc] init];
	
	PointXYZ newPoint = (PointXYZ){0, 0, 0};
	
	for (unsigned int i = 0; i < [convexHull convexHullPointsCount]; i++) {
		[convexHull getConvexHullPointNo:i X:&newPoint.x Y:&newPoint.y Z:&newPoint.z];
		[newArray addObject:@{@"point": @(i), @"xValue": @(newPoint.x), @"yValue": @(newPoint.y), @"zValue": @(newPoint.z)}];
	}
	
	return newArray;
}*/


@end
