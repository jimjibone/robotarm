//
//  JRKinectController.h
//  Simple Kinect Viewer
//
//  Created by James Reuss on 25/10/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <libfreenect/libfreenect.h>
#import "JRKinectHelpers.h"

@interface JRKinectController : NSObject {
    freenect_device *_kinectDevice;
    BOOL _haltKinect;
    
    uint16_t *_depthBack, *_depthFront;
    BOOL _depthUpdated;
    
    uint8_t *_rgbBack, *_rgbFront;
    BOOL _rgbUpdated;
    
    int _depthCount, _rgbCount;
    float _depthFPS, _rgbFPS;
	NSDate *_lastFPSDate;
    
    freenect_led_options _led;
    float _tilt;
    BOOL _irMode;
	double _angle;
}
@property (assign, nonatomic) NSMutableDictionary *kinectStatusDict;
@property (assign, nonatomic) NSMutableDictionary *kinectHardwareDict;

- (id)initWithLEDColour:(freenect_led_options)ledColour initialTilt:(float)initTilt;
- (id)initWithLEDColour:(freenect_led_options)ledColour;
- (id)init;

- (BOOL)swapDepthData:(uint16_t**)swapData;
- (BOOL)swapRGBData:(uint8_t**)swapData;
- (uint16_t*)createDepthData;
- (uint8_t*)createRGBData;

- (void)stopKinectSoon;
- (void)stopKinectImmediately;
- (void)startKinectSoon;

- (void)setKinectTilt:(float)tilt;
- (void)setKinectLED:(freenect_led_options)led;

@end
