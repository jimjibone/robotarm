//
//  JRPointRotationXYZ.h
//  ; Object Detector
//
//  Created by James Reuss on 15/04/2013.
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "JRPointTypes.h"

@interface JRPointRotationXYZ : NSObject {
	PointXYZ rotation;
}

- (id)init;
- (id)initWithRotationX:(double)x Y:(double)y Z:(double)z;

- (void)rotateByX:(double)alphaX Y:(double)alphaY Z:(double)alphaZ;

- (PointXYZ)rotation;

@end
