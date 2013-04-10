//
//  JRAppDelegate.m
//  Kinect Object Detector
//
//  Created by James Reuss on 19/11/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRAppDelegate.h"
#import "JRConstants.h"

@interface JRAppDelegate ()
- (void)fileNotifications;
- (void)removeNotifications;
- (void)objectDetectorNotificationWasReceived:(NSNotification*)aNotification;
@end

@implementation JRAppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    // File for notifications.
	[self fileNotifications];
	
	// Set up and start the Object Detector.
	isKinectConnected = NO;
	[self.connectKinectBtn setEnabled:NO];
    objectDetector = [[JRObjectDetector alloc] init];
	[objectDetector setGLViewOutlet:glView];
	[objectDetector start];
	
	[_resetTableDetectBtn setEnabled:NO];
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
	if (objectDetector != NULL) {
		[objectDetector stop];
		[objectDetector release];
		objectDetector = NULL;
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
	if ([[aNotification name] isEqualToString:nObjectDetectorDidCompleteTableDetection])
	{
		[self.resetTableDetectBtn setEnabled:YES];
		
		if ([self.tableDetectProgress isIndeterminate]) {
			[self.tableDetectProgress stopAnimation:self];
			[self.tableDetectProgress setIndeterminate:NO];
		}
		[self.tableDetectProgress setDoubleValue:0.0];
		
		//[speech startSpeakingString:@"I found the table for you."];
	}
	if ([[aNotification name] isEqualToString:nObjectDetectorDidFailTableDetection])
	{
		// Display a notification or something.
		NSLog(@"%@ %@ Notification", NSStringFromSelector(_cmd), [aNotification name]);
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
	}
}
- (void)kinectControllerNotificationWasReceived:(NSNotification*)aNotification {
	if ([[aNotification name] isEqualToString:nKinectControllerDidUpdateStatus]) {
		if ([[[aNotification userInfo] objectForKey:dKinectControllerStatusState] boolValue]) {
			[self.connectKinectBtn setEnabled:NO];
		} else {
			[self.connectKinectBtn setEnabled:YES];
		}
	}
	if ([[aNotification name] isEqualToString:nKinectControllerDidError]) {
		NSLog(@"Kinect Error: %@", [[aNotification userInfo] objectForKey:dKinectControllerErrorString]);
		//[speech startSpeakingString:[[aNotification userInfo] objectForKey:dKinectControllerErrorString]];
	}
	if ([[aNotification name] isEqualToString:nKinectControllerDidUpdateHardware]) {
		[self.kinectTilt setFloatValue:[[[aNotification userInfo] objectForKey:dKinectControllerOrientation] floatValue]];
	}
}




#pragma mark - Interface Actions
- (IBAction)connectKinect:(id)sender {
	if (!isKinectConnected) {
		[[objectDetector getKinectController] startKinectSoon];
		[self.connectKinectBtn setEnabled:NO];
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
