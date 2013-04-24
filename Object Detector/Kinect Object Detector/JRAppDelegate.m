//
//  JRAppDelegate.m
//  Kinect Object Detector
//
//  Created by James Reuss on 19/11/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRAppDelegate.h"
#import "JRConstants.h"

@interface JRAppDelegate ()
@property (nonatomic, assign) NSMutableArray *convexHullPoints;

- (void)fileNotifications;
- (void)removeNotifications;
- (void)objectDetectorNotificationWasReceived:(NSNotification*)aNotification;
@end

@implementation JRAppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    // File for notifications.
	[self fileNotifications];
	
	// Alloc things.
	_convexHullPoints = [[NSMutableArray alloc] init];
	[self.convexHullPointsController setSelectsInsertedObjects:NO];
	
	// Set view modes.
	[glView setDrawMode:MODE_POINTS];
	[gl2DView setDrawMode:MODE_2D];
	
	// Set up and start the Object Detector.
	isKinectConnected = NO;
	[self.connectKinectBtn setEnabled:NO];
    objectDetector = [[JRObjectDetector alloc] init];
	[objectDetector setGLViewOutlet:glView];
	[objectDetector setGL2DViewOutlet:gl2DView];
	[objectDetector start];
	
	[_resetTableDetectBtn setEnabled:YES];
}
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender {
    return YES;
}
- (void)applicationWillTerminate:(NSNotification *)notification {
	[self removeNotifications];
	if (objectDetector != NULL) {
		[objectDetector stop];
		[objectDetector release];
		objectDetector = NULL;
	}
}
- (void)dealloc {
	[self removeNotifications];
	if (_convexHullPoints != nil) {
		[_convexHullPoints release];
		_convexHullPoints = nil;
	}
	if (objectDetector != nil) {
		[objectDetector stop];
		[objectDetector release];
		objectDetector = nil;
	}
    [super dealloc];
}




#pragma mark - Notification Registration & Handling
- (void)fileNotifications {
	// Object Detection Notifications
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectorDidCompleteTableDetection
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectorDidFailTableDetection
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectorDidUpdateTableDetectProgress
															 object:nil];
	
	// Kinect Connection Notification
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(kinectControllerNotificationWasReceived:)
															   name:nKinectControllerDidUpdateStatus
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(kinectControllerNotificationWasReceived:)
															   name:nKinectControllerDidError
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(kinectControllerNotificationWasReceived:)
															   name:nKinectControllerDidUpdateHardware
															 object:nil];
}
- (void)removeNotifications {
	// Object Detection Notifications
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectorDidCompleteTableDetection
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectorDidFailTableDetection
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectorDidUpdateTableDetectProgress
																object:nil];
	
	// Kinect Controller Notifications
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nKinectControllerDidUpdateStatus
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nKinectControllerDidError
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nKinectControllerDidUpdateHardware
																object:nil];
}
- (void)objectDetectorNotificationWasReceived:(NSNotification*)aNotification {
	static bool fieldsNeedReset = YES;
	if ([[aNotification name] isEqualToString:nObjectDetectorDidCompleteTableDetection])
	{
		fieldsNeedReset = YES;
		[self.resetTableDetectBtn setEnabled:YES];
		
		if ([self.tableDetectProgress isIndeterminate]) {
			[self.tableDetectProgress stopAnimation:self];
			[self.tableDetectProgress setIndeterminate:NO];
		}
		[self.tableDetectProgress setDoubleValue:0.0];
		
		/*PlaneCoefficients coefficients = [objectDetector getPlaneCoefficients];
		[self.tablePlaneEquation setStringValue:[NSString stringWithFormat:
												 @"%.2fx + %.2fy + %.2fz + %.2f = 0",
												 coefficients.a,
												 coefficients.b,
												 coefficients.c,
												 coefficients.d]];
		[self.tablePlaneConfidence setStringValue:[NSString stringWithFormat:@"%.0f", coefficients.confidence]];
		
		[self.convexHullPointsController removeObjects:self.convexHullPoints];
		[self.convexHullPoints removeAllObjects];
		[self.convexHullPoints addObjectsFromArray:[objectDetector getConvexHullPoints]];
		[self.convexHullPointsController addObjects:self.convexHullPoints];*/
	}
	if ([[aNotification name] isEqualToString:nObjectDetectorDidFailTableDetection])
	{
		fieldsNeedReset = YES;
		// Display a notification or something.
		NSLog(@"%@ %@ Notification", NSStringFromSelector(_cmd), [aNotification name]);
		[self.tablePlaneEquation setStringValue:@"No equation"];
		[self.tablePlaneConfidence setStringValue:@":("];
		[self.convexHullPointsController removeObjects:self.convexHullPoints];
		[self.convexHullPoints removeAllObjects];
	}
	if ([[aNotification name] isEqualToString:nObjectDetectorDidUpdateTableDetectProgress])
	{
		double value = [[[aNotification userInfo] objectForKey:dObjectDetectorDidUpdateTableDetectProgressValue] doubleValue];
		if (value > 0) {
			// The Table Detection process has started. Update the user with the value.
			if ([self.tableDetectProgress isIndeterminate]) {
				[self.tableDetectProgress stopAnimation:self];
				[self.tableDetectProgress setIndeterminate:NO];
			}
			[self.tableDetectProgress setDoubleValue:value];
		} else if (value < 0) {
			// The Table Detection has got to an indeterminate length process.
			// Change the appearance of the progress bar to suit.
			[self.tableDetectProgress setIndeterminate:YES];
			[self.tableDetectProgress startAnimation:self];
		}
		
		if (fieldsNeedReset) {
			fieldsNeedReset = NO;
			[self.tablePlaneEquation setStringValue:@""];
			[self.tablePlaneConfidence setStringValue:@""];
			[self.convexHullPointsController removeObjects:self.convexHullPoints];
			[self.convexHullPoints removeAllObjects];
		}
	}
}
- (void)kinectControllerNotificationWasReceived:(NSNotification*)aNotification {
	if ([[aNotification name] isEqualToString:nKinectControllerDidUpdateStatus]) {
		if ([[[aNotification userInfo] objectForKey:dKinectControllerStatusState] boolValue]) {
			// Kinect is connected.
			[self.connectKinectBtn setEnabled:NO];
			//[objectDetector resetTableDetection];
		} else {
			// Kinect is not connected.
			[self.connectKinectBtn setEnabled:YES];
		}
	}
	if ([[aNotification name] isEqualToString:nKinectControllerDidError]) {
		[self.connectKinectPopover showRelativeToRect:self.connectKinectBtn.frame ofView:self.window.contentView preferredEdge:NSMinYEdge];
	}
	if ([[aNotification name] isEqualToString:nKinectControllerDidUpdateHardware]) {
		[self.kinectTilt setFloatValue:[[[aNotification userInfo] objectForKey:dKinectControllerOrientation] floatValue]];
	}
}




#pragma mark - Interface Actions
- (IBAction)connectKinect:(id)sender {
	if (!isKinectConnected) {
		[[objectDetector getKinectController] startKinectSoon];
		//[self.connectKinectBtn setEnabled:NO];//original
		[self.connectKinectBtn setEnabled:YES];
		[self.connectKinectBtn setTitle:@"Disconnect"];
	} else {
		[[objectDetector getKinectController] stopKinectSoon];
		[self.connectKinectBtn setEnabled:YES];
		[self.connectKinectBtn setTitle:@"Connect"];
	}
}
- (IBAction)resetTableDetect:(id)sender {
	[self.resetTableDetectBtn setEnabled:NO];
	[objectDetector resetTableDetection];
}
- (IBAction)changeKinectTilt:(id)sender {
	[[objectDetector getKinectController] setKinectTilt:[sender floatValue]];
}


@end
