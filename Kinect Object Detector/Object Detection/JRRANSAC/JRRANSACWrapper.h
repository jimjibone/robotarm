//
//  JRRANSACWrapper.h
//  Kinect Object Detector
//
//  Created by James Reuss on 03/02/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>

struct JRRANSACWrapperOpaque;

@interface JRRANSACWrapper : NSObject {
	struct JRRANSACWrapperOpaque *_cpp;
}

- (id)initWithMaxPoints:(uint)maximumPoints RandomPoints:(uint)randomPoints DistanceTolerance:(double)distanceTolerance;

- (void)listRandomPoints;
- (void)listOtherPoints;
- (void)listFullPointSet;
- (void)listConfidentPlanes;

- (void)updateDepthData:(uint16_t*)depthData;
- (void)performRANSAC;
- (void)prepareRANSAC;
- (void)getConfidentPlaneA:(double*)a B:(double*)b C:(double*)c D:(double*)d Confidence:(double*)confidence;
- (void)resetRANSAC;

@end
