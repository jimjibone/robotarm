//
//  JRKinectController.m
//  Simple Kinect Viewer
//
//  Created by James Reuss on 25/10/2012.
//  Copyright (c) 2012 James Reuss. All rights reserved.
//

#import "JRKinectController.h"
#import "JRConstants.h"

@interface JRKinectController ()
- (void)stopIO;
- (void)startIO;
- (void)ioThread;
- (void)depthCallback:(uint16_t*)depth;
- (void)rgbCallback:(uint8_t*)rgb;
- (void)irCallback:(uint8_t*)ir;
- (void)calculateFPS;
@end

@implementation JRKinectController

#pragma mark - Private Functions
//----------------------------------------------------------------------------------
// Private Methods
//----------------------------------------------------------------------------------
// C-type callback wrappers to Obj-C
static void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
    [(JRKinectController*)freenect_get_user(dev) depthCallback:(uint16_t*)depth];
}
static void rgbCallback(freenect_device *dev, void *video, uint32_t timestamp) {
    [(JRKinectController*)freenect_get_user(dev) rgbCallback:(uint8_t*)video];
}
static void irCallback(freenect_device * dev, void *video, uint32_t timestamp) {
    [(JRKinectController*)freenect_get_user(dev) irCallback:(uint8_t*)video];
}


#pragma mark - Private Methods
// Obj-C Methods
- (void)stopIO {
	_haltKinect= YES;
	while (_kinectDevice != NULL) usleep(10000);	// Crude! Wait till it stops.
	[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
														object:self
													  userInfo:@{
								 dKinectControllerStatusString: @"Kinect Stopped",
								  dKinectControllerStatusState: @NO}];
}
- (void)startIO {
	[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
														object:self
													  userInfo:@{
								 dKinectControllerStatusString: @"Kinect Starting",
								  dKinectControllerStatusState: @YES}];
	_haltKinect = NO;
	[NSThread detachNewThreadSelector:@selector(ioThread) toTarget:self withObject:nil];
}
- (id)initWithLEDColour:(freenect_led_options)ledColour initialTilt:(float)initTilt {
	self = [super init];
    if (self) {
        // Initialise Kinect data
		_depthFront = (uint16_t*)malloc(FREENECT_DEPTH_11BIT_SIZE);
		_depthBack = (uint16_t*)malloc(FREENECT_DEPTH_11BIT_SIZE);
		_rgbFront = (uint8_t*)malloc(FREENECT_VIDEO_RGB_SIZE);
		_rgbBack = (uint8_t*)malloc(FREENECT_VIDEO_RGB_SIZE);
		
		_lastFPSDate = [[NSDate date] retain];
		[[NSRunLoop mainRunLoop] addTimer:[NSTimer timerWithTimeInterval:1.0 target:self selector:@selector(calculateFPS) userInfo:nil repeats:YES] forMode:NSDefaultRunLoopMode];
		
		_led = ledColour;
		if (initTilt) _tilt = initTilt;
		
		// initialise the kinectStatusDict
		self.kinectStatusDict = [[NSMutableDictionary alloc] initWithObjectsAndKeys:@"Not String", @"statusString",
								 @NO, @"status",
								 @NO, @"device",
								 @NO, @"error", nil];
		self.kinectHardwareDict = [[NSMutableDictionary alloc] initWithObjectsAndKeys:@0, @"depthfps",
								   @0, @"rgbfps",
								   @0, @"orientation",
								   @(_led), @"ledcolour", nil];
		
		[self startIO];
    }
    return self;
}
- (id)initWithLEDColour:(freenect_led_options)ledColour {
	return [self initWithLEDColour:ledColour initialTilt:0];
}
- (id)init{
    return [self initWithLEDColour:LED_GREEN initialTilt:0];
}
- (void)dealloc {
    [self stopIO];
	[_lastFPSDate release];
	
	free(_depthFront);
	free(_depthBack);
	free(_rgbFront);
	free(_rgbBack);
	_depthFront = NULL;
	_depthBack  = NULL;
	_rgbFront   = NULL;
	_rgbBack    = NULL;
	
    [super dealloc];
}
- (void)ioThread {
	freenect_context *_context;
	if (freenect_init(&_context, NULL) >= 0) {
		if (freenect_num_devices(_context) == 0) {
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidError
																object:self
															  userInfo:@{dKinectControllerErrorString: @"No Devices Found"}];
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
																object:self
															  userInfo:@{
										 dKinectControllerStatusString: @"No Devices Found",
										  dKinectControllerStatusState: @NO}];
		} else if (freenect_open_device(_context, &_kinectDevice, 0) >= 0) {
			freenect_set_user(_kinectDevice, self);
			freenect_set_depth_callback(_kinectDevice, depthCallback);
			freenect_set_video_callback(_kinectDevice, rgbCallback);
			freenect_set_video_mode(_kinectDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_IR_8BIT));
			freenect_set_depth_mode(_kinectDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
			freenect_set_depth_buffer(_kinectDevice, _depthBack);
			freenect_set_video_buffer(_kinectDevice, _rgbBack);
			freenect_start_depth(_kinectDevice);
			freenect_start_video(_kinectDevice);
			
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
																object:self
															  userInfo:@{
										 dKinectControllerStatusString: @"Running",
										  dKinectControllerStatusState: @YES}];
			
			freenect_set_led(_kinectDevice, LED_YELLOW);
			
			BOOL lastIRMode = _irMode;
			freenect_led_options lastLED = _led;
			float lastTilt = _tilt;
			
			while (!_haltKinect && freenect_process_events(_context) >= 0) {
				if (_irMode != lastIRMode) {
					lastIRMode = _irMode;
					freenect_stop_video(_kinectDevice);
					freenect_set_video_callback(_kinectDevice, lastIRMode?irCallback:rgbCallback);
					freenect_set_video_mode(_kinectDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, lastIRMode?FREENECT_VIDEO_IR_8BIT:FREENECT_VIDEO_RGB));
					freenect_start_video(_kinectDevice);
				}
				
				if (_led != lastLED) {
					lastLED = _led;
					freenect_set_led(_kinectDevice, lastLED);
				}
				
				if (_tilt != lastTilt) {
					lastTilt = _tilt;
					freenect_set_tilt_degs(_kinectDevice, lastTilt);
				}
				
				freenect_update_tilt_state(_kinectDevice);
				freenect_raw_tilt_state *state = freenect_get_tilt_state(_kinectDevice);
				_angle = freenect_get_tilt_degs(state);
			}
			
			freenect_set_led(_kinectDevice, LED_RED);
			freenect_close_device(_kinectDevice);
			_kinectDevice = NULL;
			
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
																object:self
															  userInfo:@{
										 dKinectControllerStatusString: @"Stopped",
										  dKinectControllerStatusState: @NO}];
		} else {
			[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidError
																object:self
															  userInfo:@{dKinectControllerErrorString: @"Failed Opening Device"}];
		}
		
		freenect_shutdown(_context);
		[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateStatus
															object:self
														  userInfo:@{
									 dKinectControllerStatusString: @"Shutdown Libfreenect",
									  dKinectControllerStatusState: @NO}];
	} else {
		[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidError
															object:self
														  userInfo:@{dKinectControllerErrorString: @"Failed to Initialise Libfreenect"}];
	}
}
- (void)depthCallback:(uint16_t*)depth {
	// Update the back buffer, then when it is safe, swap it with the front.
	memcpy(_depthBack, depth, FREENECT_DEPTH_11BIT_SIZE);
	@synchronized (self) {
		uint16_t *buffer = _depthBack;
		_depthBack = _depthFront;
		_depthFront = buffer;
		_depthCount++;
		_depthUpdated = YES;
	}
}
- (void)rgbCallback:(uint8_t*)rgb {
	// Update the back buffer, then when it is safe, swap it with the front.
	memcpy(_rgbBack, rgb, FREENECT_VIDEO_RGB_SIZE);
	@synchronized (self) {
		uint8_t *buffer = _rgbBack;
		_rgbBack = _rgbFront;
		_rgbFront = buffer;
		_rgbCount++;
		_rgbUpdated = YES;
	}
}
- (void)irCallback:(uint8_t*)ir {
	// Update the back buffer, then when it is safe, swap it with the front.
	for (int i = 0; i < FREENECT_FRAME_PIX; i++) {
		int pval = ir[i];
		_rgbBack[3*i+0] = (pval-10 >=0)?pval-10:0;
		_rgbBack[3*i+1] = (pval-10 >=0)?pval-10:0;
		_rgbBack[3*i+2] = pval;
	}
	@synchronized (self) {
		uint8_t *buffer = _rgbBack;
		_rgbBack = _rgbFront;
		_rgbFront = buffer;
		_rgbCount++;
		_rgbUpdated = YES;
	}
}
- (void)calculateFPS {
	// As well as calculating the FPS's, update the kinectAngle too.
	NSDate *date = [NSDate date];
	NSTimeInterval dt = [date timeIntervalSinceDate:_lastFPSDate];
	if (dt > 0.5) {
		[_lastFPSDate release];
		_lastFPSDate = [date retain];
		
		int depthcount, rgbcount;
		double kinectangle;
		freenect_led_options kinectled;
		@synchronized (self) {
			depthcount = _depthCount;
			rgbcount = _rgbCount;
			_depthCount = 0;
			_rgbCount = 0;
			kinectangle = _angle;
			kinectled = _led;
		}
		[[[NSWorkspace sharedWorkspace] notificationCenter] postNotificationName:nKinectControllerDidUpdateHardware
															object:self
														  userInfo:@{
										 dKinectControllerDepthFPS: @(depthcount/dt),
										   dKinectControllerRGBFPS: @(rgbcount/dt),
									  dKinectControllerOrientation: @(kinectangle),
										dKinectControllerLEDColour: @(kinectled)}];
	}
}


#pragma mark - Instance Methods
//----------------------------------------------------------------------------------
// Data Collection Methods
//----------------------------------------------------------------------------------
- (BOOL)swapDepthData:(uint16_t**)swapData {
	// Get the pointer from the user and swap the data around.
	BOOL status = NO;
	@synchronized (self) {
		if (_depthUpdated) {
			status = YES;
			_depthUpdated = NO;
			swapPtr16(swapData, &_depthFront);
		}
	}
	return status;
}
- (BOOL)swapRGBData:(uint8_t**)swapData {
	// Get the pointer from the user and swap the data around.
	BOOL status = NO;
	@synchronized (self) {
		if (_rgbUpdated) {
			status = YES;
			_rgbUpdated = NO;
			swapPtr8(swapData, &_rgbFront);
		}
	}
	return status;
}
- (uint16_t*)createDepthData {
	// The data created by this method must be freed once used!
	uint16_t *buffer = NULL;
	@synchronized (self) {
		if (_depthUpdated) {
			_depthUpdated = NO;
			buffer = _depthFront;
			_depthFront = (uint16_t*)malloc(FREENECT_DEPTH_11BIT_SIZE);
		}
	}
	return buffer;
}
- (uint8_t*)createRGBData {
	// The data created by this method must be freed once used!
	uint8_t *buffer = NULL;
	@synchronized (self) {
		if (_rgbUpdated) {
			_rgbUpdated = NO;
			buffer = _rgbFront;
			_rgbFront = (uint8_t*)malloc(FREENECT_VIDEO_RGB_SIZE);
		}
	}
	return buffer;
}
- (void)stopKinectSoon {
	[[NSRunLoop mainRunLoop] addTimer:[NSTimer timerWithTimeInterval:0.1 target:self selector:@selector(stopIO) userInfo:nil repeats:NO]
							  forMode:NSDefaultRunLoopMode];
	
}
- (void)stopKinectImmediately {
	[self stopIO];
}
- (void)startKinectSoon {
	[[NSRunLoop mainRunLoop] addTimer:[NSTimer timerWithTimeInterval:0.1 target:self selector:@selector(startIO) userInfo:nil repeats:NO]
							  forMode:NSDefaultRunLoopMode];
}
- (void)setKinectTilt:(float)tilt {
	@synchronized (self) { _tilt = tilt; }
}
- (void)setKinectLED:(freenect_led_options)led {
	@synchronized (self) { _led = led; }
}

@end
