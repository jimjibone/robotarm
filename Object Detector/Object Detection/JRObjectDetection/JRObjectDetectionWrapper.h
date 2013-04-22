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

@end
