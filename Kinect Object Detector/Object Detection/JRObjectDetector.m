//
//  JRObjectDetector.m
//  Kinect Object Detector
//
//  Created by James Reuss on 10/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRObjectDetector.h"
#import "JRConstants.h"

#define MAIN_TIME_INTERVAL 0.1

@interface JRObjectDetector ()
//- (void)start;
- (void)addTimerForMainMethod;
//- (void)stop;

- (void)mainMethod;

// Data Management
- (BOOL)collectNewKinectFrames;
- (void)sendNewFramesToDisplay;

// RANSAC
- (void)initRANSAC;
- (void)freeRANSAC;
- (void)performRANSAC;

// Convex Hull
- (void)initConvexHull;
- (void)freeConvexHull;
- (void)performConvexHull;
- (void)sendConvexHullPointsToView;

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
        
		// Init RANSAC
		[self initRANSAC];
		[self initConvexHull];
		
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
		[[NSRunLoop currentRunLoop] addTimer:[NSTimer timerWithTimeInterval:MAIN_TIME_INTERVAL
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
	[self freeRANSAC];
	[self freeConvexHull];
	
    [super dealloc];
}




#pragma mark - Main Methods
- (void)mainMethod {
	
	if (_runMainMethod && _mainMethodCompleted) {
		_mainMethodCompleted = NO;
		
		if ([self collectNewKinectFrames]) {
			[self performRANSAC];
			[self performConvexHull];
			
			[self sendConvexHullPointsToView];
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




#pragma mark - RANSAC
#define noRandomPoints 2000
#define pointToTableTolerance 10
#define noRansacIterations 30
- (void)initRANSAC {
	ransac = [[JRRANSACWrapper alloc] initWithMaxPoints:FREENECT_FRAME_PIX RandomPoints:noRandomPoints DistanceTolerance:pointToTableTolerance];
	
	ransacConfidentPlane = (_ransacConfidentPlane){0, 0, 0, 0, 0};
	
	ransacReset = NO;
	ransacIterations = 0;
	ransacComplete = NO;
	ransacShow = YES;
	ransacShowChanged = YES;
	[glView showPlane:NO];
}
- (void)freeRANSAC {
	// Does nothing.
}
- (void)performRANSAC {
	
	if (ransacReset) {
		ransacReset = NO;
		ransacIterations = 0;
		ransacComplete = NO;
		[ransac resetRANSAC];
	}
	
	if (ransacIterations < noRansacIterations) {
		uint16_t *newDepth = (uint16_t*)malloc(FREENECT_FRAME_PIX * sizeof(uint16_t));
		memcpy(newDepth, _kinectDepth, FREENECT_FRAME_PIX * sizeof(uint16_t));
		[ransac updateDepthData:_kinectDepth];
		
		[ransac performRANSAC];
		
		[ransac getConfidentPlaneA:&ransacConfidentPlane.a
								 B:&ransacConfidentPlane.b
								 C:&ransacConfidentPlane.c
								 D:&ransacConfidentPlane.d
						Confidence:&ransacConfidentPlane.confidence];
		[glView setPlaneData:ransacConfidentPlane];
		//[ransac listConfidentPlanes];
		
		ransacShowChanged = YES;
		ransacIterations++;
		
		if (ransacShowChanged && ransacIterations > 0) [glView showPlane:YES];
		//printf("Perform RANSAC: Done iteration %d\n", ransacIterations);
		
		// Prepare the RANSAC object for another round of RANSAC'ing.
		[ransac prepareRANSAC];
		
		// Update any listeners with the progress.
		double percent = ransacIterations / (noRansacIterations - 1.0) * 100.0;
		[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nObjectDetectorDidUpdateTableDetectProgress
															object:self
														  userInfo:@{dObjectDetectorDidUpdateTableDetectProgressValue: @(percent)}];
	}
		
	if (ransacIterations == noRansacIterations && ransacComplete ==  NO) {
		ABCDPlane plane;
		double confidence;
		[ransac getConfidentPlaneA:&plane.a B:&plane.b C:&plane.c D:&plane.d Confidence:&confidence];
		[glView setPlaneData:ransacConfidentPlane];
		//[ransac listConfidentPlanes];
		
		// Post notifications on the result!
		[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nObjectDetectorDidUpdateTableDetectProgress
															object:self
														  userInfo:@{dObjectDetectorDidUpdateTableDetectProgressValue: @(-1.0)}];
		
		ransacComplete = YES;
		ransacShowChanged = YES;
		
		if (ransacShowChanged && ransacIterations > 0) [glView showPlane:YES];
	}
	
	if (ransacIterations > noRansacIterations) {
		NSLog(@"confidentPlanesCount has overflown (%d) NOT GOOD!", ransacIterations);
	}
	
}




#pragma mark - Convex Hull
// http://en.wikipedia.org/wiki/Convex_hull_algorithms
// http://www.cs.umd.edu/~mount/754/Lects/754lects.pdf
- (void)initConvexHull {
	convexHull = [[JRConvexHullWrapper alloc] init];
	convexHullReset = YES;
	convexHullComplete = NO;
	convexHullShowChanged = YES;
}
- (void)freeConvexHull {
	// Not needed.
}
- (void)performConvexHull {
	
	if (convexHullReset) {
		[convexHull resetConvexHull];
		[glView resetConvexHullPoints];
		
		convexHullReset = NO;
		convexHullComplete = NO;
		convexHullShowChanged = YES;
	}
	
	if (convexHullComplete == NO) {
		
		if (ransacIterations >= noRansacIterations) {
			
			// Set the plane for the convex hull algorithm to use. This should be the
			// confident plane found by RANSAC.
			[convexHull setPlaneA:ransacConfidentPlane.a
								B:ransacConfidentPlane.b
								C:ransacConfidentPlane.c
								D:ransacConfidentPlane.d
						Tolerance:pointToTableTolerance];
			
			// Iterate through all points and send them for Convex Hull'ing.
			double wx, wy = 0;
			for (unsigned int i = 0; i < FREENECT_FRAME_PIX; i++) {
				if (_kinectDepth[i] > 0) {
					worldFromIndex(&wx, &wy, i, _kinectDepth[i]);
					[convexHull addPointX:wx Y:wy Z:_kinectDepth[i]];
				}
			}
			
			// Perform Convex Hull.
			[convexHull performConvexHull];
			
			// List the convex hull points for the time being.
			//[convexHull listConvexHullPoints];
			
			convexHullComplete = YES;
			convexHullShowChanged = YES;
			
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nObjectDetectorDidCompleteTableDetection
																			  object:self];
			
		} // END _ransac.confidentPlanesCount >= noConfidentPlanes
	} // END convexHullComplete == NO

}
- (void)sendConvexHullPointsToView {
	static BOOL sentShow = NO;
	static BOOL sentPoints = NO;
	
	if (convexHullShowChanged) {
		convexHullShowChanged = NO;
		sentShow = NO;
		sentPoints = NO;
	}
	
	if (convexHullComplete && !sentPoints) {
		
		// Send the found convex hull points to the view.
		
		PointCH *newPoints = (PointCH*)malloc([convexHull convexHullPointsCount] * sizeof(PointCH));
		
		for (unsigned int i = 0; i < [convexHull convexHullPointsCount]; i++) {
			[convexHull getConvexHullPointNo:i X:&newPoints[i].x Y:&newPoints[i].y Z:&newPoints[i].z];
		}
		
		[glView setConvexHullPoints:newPoints Count:[convexHull convexHullPointsCount]];
		sentPoints = YES;
	}
	if (convexHullComplete && !sentShow && [convexHull convexHullPointsCount] > 0) {
		sentShow = YES;
		[glView showConvexHull:convexHullComplete];
	}
}




#pragma mark - Private Methods




#pragma mark - Instance Methods
- (JRKinectController*)getKinectController {
	return kinectController;
}
- (void)resetTableDetection {
	ransacReset = YES;
	convexHullReset = YES;
}
- (BOOL)setPlaneViewingState:(BOOL)aState {
	ransacShow = aState;
	ransacShowChanged = YES;
	return ransacShow;
}
- (BOOL)setConvexHullViewingState:(BOOL)aState {
	convexHullShow = aState;
	convexHullShowChanged = YES;
	return convexHullShow;
}
- (void)setGLViewOutlet:(GLView*)newGLView {
	if (glView) {
		[glView release];
		glView = NULL;
	}
	glView = newGLView;
}


@end
