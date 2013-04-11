//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

void JRObjectDetection::setCloudData(uint16_t *newDepthData)
{
	// Iterate through the depth data that has been input and calculate the
	// relevent world x & y coordinates.
	
	uint16_t depthCount = 0;
	for (int i = 0, r = 0, o = 0; i < maxPoints; i++) {
		// Get the relevent Point from either randomPoints or otherPoints.
		Point *point = 0;
		
		if (i == randomPoints[r].index) {
			point = &randomPoints[r];
			r++;
		}
		else if (i == otherPoints[o].index) {
			point = &otherPoints[o];
			o++;
		}
		
		worldFromIndex(&point->x, &point->y, point->index, newDepth[i]);
		
		point->z = newDepth[i]+1.0;
		point->z-=1.0;
		
		depthCount += newDepth[i];
	}
	if (depthCount > 0) {
		validDepthData = true;
	} else {
		validDepthData = false;
		printf("updateDepthData dataIsNotValid\n");
	}
}