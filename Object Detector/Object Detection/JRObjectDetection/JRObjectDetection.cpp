//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"
#include "JRRANSAC.h"
#include "JRConvexHull.h"
#include "JRPointInclusion.h"
#include "JRKMeansClustering.h"
#include <boost/thread/thread_time.hpp>

#pragma mark - 
#pragma mark Helping Functions

PointXYZ worldFromIndex(size_t index, int z)
{
	double x = index % FREENECT_FRAME_W;
	double y = (index - x) / FREENECT_FRAME_W;
	x = (double)((x - FREENECT_FRAME_W/2.0) * z * 0.00174);
	y = (double)((y - FREENECT_FRAME_H/2.0) * z * -0.00174);
	return PointXYZ(x, y, z);
}

// Function to get the plane equation from 3 points in 3D space.
PlaneCoefficients getPlaneCoefficients(PointXYZ a, PointXYZ b, PointXYZ c)
{
	// First validate the input to check that Z values are not out-of-bounds of Kinect view.
	if (!a.isValid() || !b.isValid() || !c.isValid()) {
		return PlaneCoefficients(0, 0, 0, 0);
	}
	
	// http://keisan.casio.com/has10/SpecExec.cgi# or
	// http://www.easycalculation.com/analytical/cartesian-plane-equation.php or
	// Weisstein, Eric W. "Plane." From MathWorld--A Wolfram Web Resource. http://mathworld.wolfram.com/Plane.html
	double Pa = (b.y - a.y)*(c.z - a.z) - (c.y - a.y)*(b.z - a.z);
	double Pb = (b.z - a.z)*(c.x - a.x) - (c.z - a.z)*(b.x - a.x);
	double Pc = (b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y);
	double Pd = -(Pa*a.x + Pb*a.y + Pc*a.z);
	
	return PlaneCoefficients(Pa, Pb, Pc, Pd);
}

double dot(PointXYZ a, PointXYZ b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double dot(PlaneCoefficients a, PlaneCoefficients b)
{
	return a.a*b.a + a.b*b.b + a.c*b.c;
}

double mod(PlaneCoefficients a)
{
	return sqrt( a.a*a.a + a.b*a.b + a.c*a.c );
}

double mod(PointXYZ a)
{
	return sqrt( a.x*a.x + a.y*a.y + a.z*a.z );
}

double theta(PlaneCoefficients a, PlaneCoefficients b)
{
	// theta = acos ( a . b / ||a|| ||b|| )
	double moda = mod(a);
	double modb = mod(b);
	return (moda == 0 || modb == 0) ? 0 : acos( dot(a, b) / (moda * modb) );
}

// To normalise a normal equation you can do this
//	CombinedSquares = (Normal.x * Normal.x) +
//	(Normal.y * Normal.y) +
//	(Normal.z * Normal.z);
//	NormalisationFactor = sqrt(CombinedSquares);
//	Normal.x = Normal.x / NormalisationFactor;
//	Normal.y = Normal.y / NormalisationFactor;
//	Normal.z = Normal.z / NormalisationFactor;





#pragma mark -
#pragma mark Private Functions

bool ObjectDetection::compareNormalAngle(size_t a, size_t b)
{
	
	// Compare the difference between the d component of the two points.
	//double distanceVal = fabs(input_cloud_normals[a].d - input_cloud_normals[b].d);	// Distance between the d coefficient of 2 point normals.
	//double distanceVal = fabs(input_cloud[a].z - input_cloud[b].z);					// Distance between the Z components of 2 3D points.
	double distanceVal = fabs( mod(input_cloud[a]) - mod(input_cloud[b] ));				// Distance between 2 3D points.
	//double distanceVal = 0;
	bool distance = distanceVal < COMPARE_NORMALS_DISTANCE_THRESH;
	
	// Compare the angle between normal vectors by doing the dot product.
	//double angleVal = dot(input_cloud_normals[a], input_cloud_normals[b]);
	double angleVal = theta(input_cloud_normals[a], input_cloud_normals[b]);
	bool angle = angleVal < COMPARE_NORMALS_ANGLE_THRESH;
	
	bool isValid = input_cloud_normals[a].isSet() && input_cloud_normals[b].isSet();
	
	/*static size_t count = 0;
	if (count > 1000) {
		count = 0;
		cout << "\tcompareNormalAngle  :  distance = " << distanceVal << " (" << distance << ")  " <<
				"angle = " << angleVal << " (" << angle << ")  " <<
				"isValid = " << isValid << "." << endl;
	} else {
		count++;
	}*/
	
	/*if (distance && angle && isValid) {
		cout << "\tcompareNormalAngle  :  distance = " << distanceVal << " (" << distance << ")  " <<
				"angle = " << angleVal << " (" << angle << ")  " <<
				"isValid = " << isValid << "." << endl;
	}*/
	
	
	return distance && angle && isValid;
}





#pragma mark -
#pragma mark Object Detection Functions

void ObjectDetection::setZPoints(uint16_t* zPoints, size_t size)
{
	// Remove all points from the input_cloud.
	input_cloud.erase(input_cloud.begin(), input_cloud.end());
	
	uint16_t depthCount = 0;
	// Iterate through all the new points and add them to the input_cloud
	// also calculate the world x and y values on input.
	for (size_t i = 0; i < size; i++) {
		depthCount += zPoints[i];
		input_cloud.emplace_back(worldFromIndex(i, zPoints[i]));
	}
	if (depthCount > 0) {
		validDepthData = true;
	} else {
		validDepthData = false;
		printf("updateDepthData dataIsNotValid\n");
	}
}

void ObjectDetection::calculateSurfaceNormals()
{
	if (validDepthData) {
		cout << "calculateSurfaceNormals started." << endl;
		boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
		
		// Iterate through all the points in the input_cloud and calcualte
		// the surface normals of each point.
		// First, remove all the current normals.
		input_cloud_normals.erase(input_cloud_normals.begin(), input_cloud_normals.end());
		
		// Some points will need to be skipped as the normals are calculated
		// by using 2 neighboring points to the right and below. Therefore the
		// right-most and bottom-most pixels will not have a complete set of
		// neighbours.
		//size_t count = 0;
		for (size_t i = 0; i < input_cloud.size(); i++) {
			uint x = 0, y = 0;
			frameXYfromIndex(&x, &y, (uint)i);
			
			if (   x >= FREENECT_FRAME_W-NORMAL_CALC_POINT_SPREAD-1	// is right-hand side
				&& y >= FREENECT_FRAME_H*NORMAL_CALC_POINT_SPREAD-2	// and is bottom
				) {
				// This point is out-of-bounds! Set the normal to 0.
				input_cloud_normals.emplace_back(PlaneCoefficients(0, 0, 0, 0));
			} else {
				// This point is within bounds. Calculate the normal.
				input_cloud_normals.emplace_back(getPlaneCoefficients(input_cloud[i],
																	  input_cloud[i+NORMAL_CALC_POINT_SPREAD],
																	  input_cloud[i+FREENECT_FRAME_W*NORMAL_CALC_POINT_SPREAD]));
			}
			//if (count > 10000) {
				//cout << "\tNormal coefficients of point " << i << " are A = " << input_cloud_normals[i].a << "  B = " << input_cloud_normals[i].b << "  C = " << input_cloud_normals[i].c << "  D = " << input_cloud_normals[i].d << "." << endl;
			//	count = 0;
			//} else { count++; }
		}
		
		boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration diff = stop - start;
		cout << "calculateSurfaceNormals completed." << " Taken " << diff.total_microseconds() << " us." << endl;
	}/* END validDepthData */
}

void ObjectDetection::segmentPlanes()
{
	// Mark all nodes unassigned.
	//
	// ITERATE through all nodes:
	//		IF node unassigned:
	//			ASSIGN node to a new component id C
	//			DO a continuous-search for all nodes connected to this one:
	//				mark them with same component id C
	
	if (validDepthData) {
		cout << "segmentPlanes started." << endl;
		boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
		
		// Remove all the current points from the plane_cluster_nodes and reserve the size we need.
		// Also, set all the node values to -1 to show that they are all unassigned.
		vector<int> plane_cluster_nodes;
		plane_cluster_nodes.assign(FREENECT_FRAME_PIX, NODE_UNASSIGNED);
		
		int current_segment_id = -1;
		
		// Iterate through all nodes.
		for (size_t i = 0; i < FREENECT_FRAME_PIX; i++) {
			
			// Check if the current node is unassigned.
			if (plane_cluster_nodes[i] == NODE_UNASSIGNED) {
				
				//cout << "\tCurrent node is UNASSIGNED. Node = " << i << "   Current Segment = " << current_segment_id << "." << endl;
				
				// The node is unassigned then assign it a new segment_id.
				current_segment_id++;
				
				// Create a process vector to hold all the points that require processing with this segment_id.
				vector<size_t> unprocessed_points;
				
				// Add the current point to the unprocessed_points and begin the neighbour search.
				unprocessed_points.emplace_back(i);
				plane_cluster_nodes[i] = current_segment_id;
				
				while (unprocessed_points.size() > 0) {
					
					// Get the current point.
					size_t point = *unprocessed_points.begin();
					unprocessed_points.erase(unprocessed_points.begin());
					
					// Get the position of the current point.
					uint currentX, currentY = 0;
					frameXYfromIndex(&currentX, &currentY, (uint)point);
					
					// Now search for neighbours for a distance defined in PLANE_NEIGHBOUR_SEARCH_DIST
					for (size_t spread = 0; spread < PLANE_NEIGHBOUR_SEARCH_DIST+1; spread++) {
						
						// Get point indices.
						size_t above = point-FREENECT_FRAME_W*spread;
						size_t right = point+1*spread;
						size_t below = point+FREENECT_FRAME_W*spread;
						size_t left  = point-1*spread;
						
						const size_t minY = spread, maxY = FREENECT_FRAME_H-1-spread;
						const size_t minX = spread, maxX = FREENECT_FRAME_W-1-spread;
						
						// Find out if its neighbouring points qualify.
						bool aboveIsNeighbour = ((currentY > minY) && (plane_cluster_nodes[above] == NODE_UNASSIGNED)) ? compareNormalAngle(point, above) : false;
						bool rightIsNeighbour = ((currentX < maxX) && (plane_cluster_nodes[right] == NODE_UNASSIGNED)) ? compareNormalAngle(point, right) : false;
						bool belowIsNeighbour = ((currentY < maxY) && (plane_cluster_nodes[below] == NODE_UNASSIGNED)) ? compareNormalAngle(point, below) : false;
						bool leftIsNeighbour  = ((currentX > minX) && (plane_cluster_nodes[left]  == NODE_UNASSIGNED)) ? compareNormalAngle(point, left)  : false;
						
						// If there is a neighbour then add it to the list to be processed and set its segment_id.
						if (aboveIsNeighbour) {
							unprocessed_points.emplace_back(above);
							plane_cluster_nodes[above] = current_segment_id;
						}
						if (rightIsNeighbour) {
							unprocessed_points.emplace_back(right);
							plane_cluster_nodes[right] = current_segment_id;
						}
						if (belowIsNeighbour) {
							unprocessed_points.emplace_back(below);
							plane_cluster_nodes[below] = current_segment_id;
						}
						if (leftIsNeighbour) {
							unprocessed_points.emplace_back(left);
							plane_cluster_nodes[left] = current_segment_id;
						}
						
						/*cout << "\tProcessing point " << point << " with index " << i << " (" << currentX << ", " << currentY <<
						 ")    Neighbours: above=" << aboveIsNeighbour << "  right=" << rightIsNeighbour <<
						 "  below=" << belowIsNeighbour << "  left=" << leftIsNeighbour <<
						 ".    Current segment_id=" << current_segment_id << "  Points unprocessed=" << unprocessed_points.size() << "." << endl;*/
						
					}
					
				}
				
			}
			
		}
		
		boost::posix_time::ptime mid = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration middiff = mid - start;
		cout << "segmentPlanes completed. Found " << current_segment_id << "." << " Taken " << middiff.total_microseconds() << " us." << endl;
		
		// FILTERING
		
		// Run through all the points in plane_cluster_nodes and count how many
		// points belong to each one.
		vector<PointIndices> all_clusters;
		all_clusters.resize(current_segment_id);
		
		for (size_t i = 0; i < plane_cluster_nodes.size(); i++) {
			size_t segment_id = (size_t)plane_cluster_nodes[i];
			//cout << "\tassigning " << i << " to segment_id " << segment_id << ". all_clusters.size = " << all_clusters.size() << "." << endl;
			
			if (segment_id >= all_clusters.size()) {
				PointIndices newIndices;
				newIndices.indices.emplace_back(i);
				all_clusters.emplace_back(newIndices);
			} else {
				all_clusters[segment_id].indices.emplace_back(i);
			}
		}
		
		// Find all the clusters that have a point count greater than the threshold.
		plane_clusters.erase(plane_clusters.begin(), plane_clusters.end());
		size_t bigClusterCount = 0;
		for (size_t i = 0; i < all_clusters.size(); i++) {
			if (all_clusters[i].indices.size() > PLANE_CLUSTER_THRESHOLD) {
				bigClusterCount++;
				// Add this cluster to the plane_clusters.
				plane_clusters.emplace_back(all_clusters.at(i));
			}
		}
		
		boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration diff = stop - mid;
		cout << "segmentPlanes Filtered clusters down to " << bigClusterCount << " clusters. new plane_clusters.size = " << plane_clusters.size() << "." << " Taken " << diff.total_microseconds() << " us." << endl;
		
		// Find the normal coefficients for the plane_clusters.
		// Do this by using the normal coefficients from the mode point in the cluster.
		/*plane_clusters_normals.erase(plane_clusters_normals.begin(), plane_clusters_normals.end());
		for (size_t i = 0; i < plane_clusters.size(); i++) {
			size_t index = plane_clusters[i].indices[(uint)plane_clusters[i].indices.size()/2];
			plane_clusters_normals.emplace_back(input_cloud_normals[index].a, input_cloud_normals[index].b, input_cloud_normals[index].c, input_cloud_normals[index].d);
			cout << "segmentPlanes set cluster " << i << " normals to A = " << input_cloud_normals[index].a << "  B = " << input_cloud_normals[index].b << "  C = " << input_cloud_normals[index].c << "  D = " << input_cloud_normals[index].d << " for an index of " << index << "." << endl;
		}*/
	}/* END validDepthData */
}

void ObjectDetection::findDominantPlane()
{
	if (validDepthData) {
		
		cout << "findDominantPlane started." << endl;
		boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
		
		// Now find the plane cluster that has the most
		// points and is most centre within the frame.
		
		if (plane_clusters.size() == 0) {
			
			// There are no planes! This is not very good.
			// So let's not do any dominant plane finding.
			// Just set the dominant plane as unassigned.
			dominant_plane.index = DOMINANT_PLANE_UNASSIGNED;
			
		} else if (plane_clusters.size() == 1) {
			
			// There is only one plane! This helps a lot.
			// So there is no need to find the dominant plane. Just
			// set the dominant plane as this one.
			dominant_plane.index = 0;
			
		} else {
			
			// Then there are many planes. Iterate through them all
			// searching for the plane cluster that contains the most points
			// and is most centre within the frame.
			
			// Now give each cluster an approval rating. The better the plane, the
			// higher the rating. Then at the end set the plane with the highest
			// approval as the dominant.
			
			vector<size_t> cluster_approval_ratings;
			cluster_approval_ratings.resize(plane_clusters.size(), 0);
			
			int highest_approval = 0;
			size_t highest_approval_cluster = DOMINANT_PLANE_UNASSIGNED;
			
			for (size_t i = 0; i < plane_clusters.size(); i++) {
				
				size_t pointCount = plane_clusters[i].indices.size();
				
				double avgX = 0.0, avgY = 0.0;
				for (size_t j = 0; j < plane_clusters[i].indices.size(); j++) {
					
					avgX += input_cloud[ plane_clusters[i].indices[j] ].x;
					avgY += input_cloud[ plane_clusters[i].indices[j] ].y;
					
				}
				avgX /= pointCount;
				avgY /= pointCount;
				
				// Now find out how good the plane cluster is as a dominant plane.
				
				int current_approval = (int)pointCount/PLANE_CLUSTER_THRESHOLD;
				current_approval -= fabs(avgX)/10;
				current_approval -= fabs(avgY)/10;
				
				if (current_approval > highest_approval) {
					highest_approval = current_approval;
					highest_approval_cluster = i;
				}
				
			}
			
			// Now the plane_cluster that has the best fit will be set
			// as the dominant plane.
			
			dominant_plane.index = highest_approval_cluster;
			
		}
		
		boost::posix_time::ptime mid = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration middiff = mid - start;
		cout << "findDominantPlane found dominant plane." << " Taken " << middiff.total_microseconds() << " us." << endl;
		
		// Now that we've found the dominant plane, find out its plane
		// equation and perform the convex hull of the points, to find
		// its bounds.
		
		if (dominant_plane.index != DOMINANT_PLANE_UNASSIGNED) {
			
			RANSAC sac (RANSAC_DISTANCE_TOLERANCE);	// ransac with mm distance tolerance.
			sac.setData(&input_cloud, &(plane_clusters[dominant_plane.index]));
			sac.run();
			dominant_plane.confidence = sac.confidence;
			dominant_plane.coefficients = sac.coefficients;
			
			ConvexHull hull (CONVEXHULL_DISTANCE_TOLERANCE);	// convex hull with mm distance tolerance.
			hull.setData(&input_cloud, &plane_clusters[dominant_plane.index], &dominant_plane.coefficients);
			hull.run();
			dominant_plane.hull = hull.hull_indices;
			
		}
		
		boost::posix_time::ptime stop = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration diff = stop - mid;
		cout << "findDominantPlane completed." << " Taken " << diff.total_microseconds() << " us." << endl;
		
	}/* END validDepthData */
	
}

void ObjectDetection::segmentObjects()
{
	if (validDepthData) {
		
		objects.erase(objects.begin(), objects.end());
		
		// What needs to be done:
		// - Get the indices of all the points above the table.
		// - Do a K-Means clustering of the points to find the object clusters.
		// - Do some basic shape fitting of objects to find the diameter, height and location.
		
		// Get the indices of all the points above the table.
		PointInclusion inclusion (INCLUSION_MAX_HEIGHT);	// Set the tolerance to include points that are up to 500mm above the table.
		inclusion.setMinDistance(INCLUSION_MIN_HEIGHT);		// Set the minimum distance off the plane to 10mm. All objects less than will be ignored.
		inclusion.setCloud(&input_cloud);
		inclusion.setPlane(&dominant_plane.coefficients);
		inclusion.setHull(&dominant_plane.hull);
		// We can assume that dominant plane points are not part of the objects.
		inclusion.excludeIndices(&plane_clusters[dominant_plane.index]);
		inclusion.run();
		objects_points = inclusion.included_indices;
		
		// Do a K-Means clustering of the points to find the object clusters.
		KMeansClustering kmeans;
		kmeans.setCloud(&input_cloud);
		kmeans.useIndices(objects_points);
		kmeans.setClusterCount(KMEANS_CLUSTER_COUNT);
		kmeans.run();
		kmeans.filterClusters(KMEANS_FILTER_DISTANCE);
		
		// Get all the objects info.
		objects.resize(kmeans.clusters.size());
		for (size_t o = 0; o < kmeans.clusters.size(); o++) {
			objects[o].indices.indices = kmeans.clusters[o].indices;
			objects[o].centroid = kmeans.clusters_centroids[o];
			objects[o].radius = kmeans.clusters_radii[o];
		}
		
	}/* END validDepthData */
}

// DONE: ABOVE: Also get the plane equation and convex hull of the table (dominant) plane.
// DONE: THEN: Get the indices of all the points above the table.
// DONE: THEN: Do K-Means clustering to find the object clouds indices.
// KINDA DONE: THEN: Maybe to some shape-fitting and store the diameter/position of objects on table.


