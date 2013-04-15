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
	JRObjectDetectionWrapperOpaque(uint newMaxPoints, double newMaxHeight, double newMinHeight) : wrapper(newMaxPoints, newMaxHeight, newMinHeight) {};
	ObjectDetection::ObjectDetection wrapper;
};

#pragma mark -
#pragma mark Objective-C Methods

- (id)initWithMaxPoints:(uint)newMaxPoints MaxHeight:(double)newMaxHeight MinHeight:(double)newMinHeight
{
	self = [super init];
	if (self) {
		self.cpp = new JRObjectDetectionWrapperOpaque(newMaxPoints, newMaxHeight, newMinHeight);
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
#pragma mark Initialisation Methods

- (void)setPlaneCoefficientsA:(double)a B:(double)b C:(double)c D:(double)d Invert:(bool)invert
{
	self.cpp->wrapper.setPlaneCoefficients(a, b, c, d, invert);
}
- (void)addPreprocessedConvexHullPointX:(double)x Y:(double)y Z:(double)z I:(double)i J:(double)j
{
	PointXYZIJ aPoint (x, y, z, i, j);
	self.cpp->wrapper.addPreprocessedConvexHullPoint(aPoint);
}




#pragma mark -
#pragma mark Each 'New Frame' Functions
// Execute in order shown.

- (void)updateDepthData:(uint16_t*)newDepth
{
	self.cpp->wrapper.updateDepthData(newDepth);
}
- (void)performObjectDetection
{
	self.cpp->wrapper.performObjectDetection();
}
- (unsigned int)objectsCloudPointsCount
{
	return (unsigned int)self.cpp->wrapper.cloud_objects.points.size();
}
- (void)getObjectsCloudPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ
{
	*outX = self.cpp->wrapper.cloud_objects.points[i].x;
	*outY = self.cpp->wrapper.cloud_objects.points[i].y;
	*outZ = self.cpp->wrapper.cloud_objects.points[i].z;
}
- (void)prepareForNewData
{
	self.cpp->wrapper.prepareForNewData();
}

@end
