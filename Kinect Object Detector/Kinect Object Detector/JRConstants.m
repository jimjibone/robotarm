//
//  JRConstants.m
//  Kinect Object Detector
//
//  Created by James Reuss on 05/02/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRConstants.h"



@implementation JRConstants

// Kinect Controller Notifications
NSString *const nKinectControllerDidUpdateStatus	= @"nKinectControllerDidUpdateStatus";
NSString *const dKinectControllerStatusString		= @"dKinectControllerStatusString";
NSString *const dKinectControllerStatusState		= @"dKinectControllerStatusState";

NSString *const nKinectControllerDidError		= @"nKinectControllerDidError";
NSString *const dKinectControllerErrorString	= @"dKinectControllerErrorString";

NSString *const nKinectControllerDidUpdateHardware	= @"nKinectControllerDidUpdateHardware";
NSString *const dKinectControllerDepthFPS				= @"dKinectControllerDepthFPS";
NSString *const dKinectControllerRGBFPS				= @"dKinectControllerRGBFPS";
NSString *const dKinectControllerOrientation			= @"dKinectControllerOrientation";
NSString *const dKinectControllerLEDColour			= @"dKinectControllerLEDColour";



// Object Detector Notifications
NSString *const nObjectDetectorDidCompleteTableDetection			= @"nObjectDetectorDidCompleteTableDetection";

NSString *const nObjectDetectorDidFailTableDetection				= @"nObjectDetectorDidFailTableDetection";
NSString *const dObjectDetectorDidFailTableDetectionString		= @"dObjectDetectorDidFailTableDetectionString";

NSString *const nObjectDetectorDidUpdateTableDetectProgress		= @"nObjectDetectorDidUpdateTableDetectProgress";
NSString *const dObjectDetectorDidUpdateTableDetectProgressValue	= @"dObjectDetectorDidUpdateTableDetectProgressValue";

@end
