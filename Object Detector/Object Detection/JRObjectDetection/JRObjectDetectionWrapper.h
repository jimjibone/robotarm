//
//  JRObjectDetectionWrapper.h
//  Kinect Object Detector
//
//  Created by James Reuss on 14/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "../JRPointTypes.h"

struct JRObjectDetectionWrapperOpaque;

@interface JRObjectDetectionWrapper : NSObject {
	struct JRObjectDetectionWrapperOpaque *_cpp;
}

- (id)initWithMaxPoints:(uint)newMaxPoints MaxHeight:(double)newMaxHeight MinHeight:(double)newMinHeight;

- (void)setPlaneCoefficientsA:(double)a B:(double)b C:(double)c D:(double)d Invert:(bool)invert;
- (void)addPreprocessedConvexHullPointX:(double)x Y:(double)y Z:(double)z I:(double)i J:(double)j;

- (void)updateDepthData:(uint16_t*)newDepth;
- (void)performObjectDetection;
- (unsigned int)objectsCloudPointsCount;
- (void)getObjectsCloudPointNo:(unsigned int)i X:(double*)outX Y:(double*)outY Z:(double*)outZ;
- (void)prepareForNewData;

@end
