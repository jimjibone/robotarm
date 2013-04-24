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

- (id)init;

- (void)setZPoints:(uint16_t*)zPoints size:(size_t)size;
- (void)calculateSurfaceNormals;
- (void)segmentPlanes;

- (size_t)getNumberOfPlaneClusters;
- (size_t)getNumberOfIndicesInPlaneCluster:(size_t)cluster;
- (void)getX:(double*)x Y:(double*)y Z:(double*)z forPoint:(size_t)point forPlaneCluster:(size_t)cluster;
- (void)getA:(double*)a B:(double*)b C:(double*)c D:(double*)d forPlaneCluster:(size_t)cluster;

@end