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
															   name:nObjectDetectorDidCompleteObjectDetection
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectionDidCompletePlaneClusterDetection
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectionDidCompleteDominantPlaneDetection
															 object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] addObserver:self
														   selector:@selector(objectDetectorNotificationWasReceived:)
															   name:nObjectDetectionDidCompleteObjectClustering
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
																  name:nObjectDetectorDidCompleteObjectDetection
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectionDidCompletePlaneClusterDetection
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectionDidCompleteDominantPlaneDetection
																object:nil];
	[[[NSWorkspace sharedWorkspace] notificationCenter] removeObserver:self
																  name:nObjectDetectionDidCompleteObjectClustering
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
	if ([[aNotification name] isEqualToString:nObjectDetectorDidCompleteObjectDetection])
	{
		fieldsNeedReset = YES;
		[self.resetTableDetectBtn setEnabled:YES];
	}
	if ([[aNotification name] isEqualToString:nObjectDetectionDidCompletePlaneClusterDetection])
	{
		fieldsNeedReset = YES;
		[self.planeClustersFound setStringValue:[[[aNotification userInfo] objectForKey:@"clustersFound"] stringValue]];
		
		[self.convexHullPointsController removeObjects:self.convexHullPoints];
		[self.convexHullPoints removeAllObjects];
	}
	if ([[aNotification name] isEqualToString:nObjectDetectionDidCompleteDominantPlaneDetection])
	{
		fieldsNeedReset = YES;
		[self.dominantPlaneConfidence setStringValue:[[[aNotification userInfo] objectForKey:@"confidence"] stringValue]];
		[self.dominantPlaneHullPoints setStringValue:[[[aNotification userInfo] objectForKey:@"hullPointsCount"] stringValue]];
	}
	if ([[aNotification name] isEqualToString:nObjectDetectionDidCompleteObjectClustering])
	{
		NSArray *clusters = [[aNotification userInfo] objectForKey:@"clusters"];
		[self.kmeansClusters setStringValue:[@([clusters count]) stringValue]];
		NSTextView *textView = [self.kmeansText documentView];
		
		NSString *text = @"";
		for (NSDictionary *dict in clusters) {
			text = [text stringByAppendingFormat:@"Cluster %ld contains %ld points and radius is %.1f mm.\n",
					[[dict objectForKey:@"index"] integerValue],
					[[dict objectForKey:@"count"] integerValue],
					[[dict objectForKey:@"radius"] doubleValue]];
		}
		
		[textView setString:text];
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
	//[[objectDetector getKinectController] setKinectTilt:[sender floatValue]];
}


@end
