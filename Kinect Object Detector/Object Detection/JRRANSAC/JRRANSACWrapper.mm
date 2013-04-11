//
//  JRRANSACWrapper.m
//  Kinect Object Detector
//
//  Created by James Reuss on 03/02/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import "JRRANSACWrapper.h"
#import "JRRANSAC.h"

@interface JRRANSACWrapper ()
@property (nonatomic, readwrite, assign) JRRANSACWrapperOpaque *cpp;
@end

@implementation JRRANSACWrapper
@synthesize cpp = _cpp;

struct JRRANSACWrapperOpaque {
public:
	JRRANSACWrapperOpaque(uint newMaxPoints, uint newRandomPointCount, double newDistanceTolerance) : wrapper(newMaxPoints, newRandomPointCount, newDistanceTolerance) {};
	RANSAC::RANSAC wrapper;
};

#pragma mark - Objective-C Methods
- (id)initWithMaxPoints:(uint)maximumPoints RandomPoints:(uint)randomPoints DistanceTolerance:(double)distanceTolerance {
    self = [super init];
    if (self) {
        self.cpp = new JRRANSACWrapperOpaque(maximumPoints, randomPoints, distanceTolerance);
    }
    return self;
}
- (void)dealloc {
    delete _cpp;
	_cpp = NULL;
	
    [super dealloc];
}




#pragma mark - Point Listing Methods
- (void)listRandomPoints {
	self.cpp->wrapper.listRandomPoints();
}
- (void)listOtherPoints {
	self.cpp->wrapper.listOtherPoints();
}
- (void)listFullPointSet {
	self.cpp->wrapper.listFullSet();
}
- (void)listConfidentPlanes {
	self.cpp->wrapper.listConfidentPlanes();
}




#pragma mark - Main Class Methods
- (void)updateDepthData:(uint16_t*)depthData {
	self.cpp->wrapper.updateDepthData(depthData);
}
- (void)performRANSAC {
	self.cpp->wrapper.performRANSAC();
}
- (void)prepareRANSAC {
	self.cpp->wrapper.prepareRANSAC();
}
- (void)getConfidentPlaneA:(double*)a B:(double*)b C:(double*)c D:(double*)d Confidence:(double*)confidence {
	self.cpp->wrapper.getConfidentPlane(a, b, c, d, confidence);
}
- (void)resetRANSAC {
	self.cpp->wrapper.resetRANSAC();
}

@end
