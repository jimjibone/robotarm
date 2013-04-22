//
//  JRObjectDetectionWrapper.m
//  Kinect Object Detector
//
//  Created by James Reuss on 14/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRObjectDetectionWrapper.h"
#import "JRObjectDetection.h"

@interface JRObjectDetectionWrapper ()
@property (nonatomic, readwrite, assign) JRObjectDetectionWrapperOpaque *cpp;
@end

@implementation JRObjectDetectionWrapper
@synthesize cpp = _cpp;

struct JRObjectDetectionWrapperOpaque {
public:
	JRObjectDetectionWrapperOpaque() {};
	ObjectDetection::ObjectDetection wrapper;
};




#pragma mark -
#pragma mark Objective-C Methods

- (id)init
{
	self = [super init];
	if (self) {
		self.cpp = new JRObjectDetectionWrapperOpaque();
	}
	return self;
}

- (void)dealloc
{
	delete _cpp;
	_cpp = NULL;
	
    [super dealloc];
}




#pragma mark -
#pragma mark Each 'New Frame' Functions
// Execute in order shown.

- (void)setZPoints:(uint16_t*)zPoints size:(size_t)size
{
	self.cpp->wrapper.setZPoints(zPoints, size);
}
- (void)calculateSurfaceNormals
{
	self.cpp->wrapper.calculateSurfaceNormals();
}
- (void)segmentPlanes
{
	self.cpp->wrapper.segmentPlanes();
}




@end
