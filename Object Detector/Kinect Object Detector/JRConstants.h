//
//  JRConstants.h
//  Kinect Object Detector
//
//  Created by James Reuss on 05/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>




@interface JRConstants : NSObject

// Kinect Controller Notifications
FOUNDATION_EXPORT NSString *const nKinectControllerDidUpdateStatus;
FOUNDATION_EXPORT NSString *const dKinectControllerStatusString;
FOUNDATION_EXPORT NSString *const dKinectControllerStatusState;

FOUNDATION_EXPORT NSString *const nKinectControllerDidError;
FOUNDATION_EXPORT NSString *const dKinectControllerErrorString;

FOUNDATION_EXPORT NSString *const nKinectControllerDidUpdateHardware;
FOUNDATION_EXPORT NSString *const dKinectControllerDepthFPS;
FOUNDATION_EXPORT NSString *const dKinectControllerRGBFPS;
FOUNDATION_EXPORT NSString *const dKinectControllerOrientation;
FOUNDATION_EXPORT NSString *const dKinectControllerLEDColour;

// Object Detector Notifications
FOUNDATION_EXPORT NSString *const nObjectDetectorDidCompleteObjectDetection;

FOUNDATION_EXPORT NSString *const nObjectDetectionDidCompletePlaneClusterDetection;

@end
