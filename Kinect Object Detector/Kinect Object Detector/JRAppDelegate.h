//
//  JRAppDelegate.h
//  Kinect Object Detector
//
//  Created by James Reuss on 19/11/2012.
//	jamesreuss.co.uk
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import "JRObjectDetector.h"
#import "JRKinectController.h"
#import "GLView.h"

@class GLView;
@interface JRAppDelegate : NSObject <NSApplicationDelegate> {
    JRObjectDetector *objectDetector;
	BOOL isKinectConnected;
	IBOutlet GLView *glView;
}

@property (assign) IBOutlet NSWindow *window;
@property (assign) IBOutlet NSButton *connectKinectBtn;
@property (assign) IBOutlet NSPopover *connectKinectPopover;
@property (assign) IBOutlet NSButton *resetTableDetectBtn;
@property (assign) IBOutlet NSProgressIndicator *tableDetectProgress;
@property (assign) IBOutlet NSSlider *kinectTilt;

// Object detection pane
@property (assign) IBOutlet NSTextField *tablePlaneEquation;
@property (assign) IBOutlet NSTextField *tablePlaneConfidence;
@property (assign) IBOutlet NSArrayController *convexHullPointsController;

- (IBAction)connectKinect:(id)sender;
- (IBAction)resetTableDetect:(id)sender;
- (IBAction)changeKinectTilt:(id)sender;

@end
