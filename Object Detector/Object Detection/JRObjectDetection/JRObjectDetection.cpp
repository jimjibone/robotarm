//
//  JRObjectDetection.cpp
//  Kinect Object Detector
//
//  Created by James Reuss on 11/04/2013.
//	jamesreuss.co.uk
//  Copyright (c) 2013 James Reuss. All rights reserved.
//

#include "JRObjectDetection.h"

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
	
	//http://keisan.casio.com/has10/SpecExec.cgi# or
	//http://www.easycalculation.com/analytical/cartesian-plane-equation.php
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
	const float thresholdDistance = 5; // 5mm.
	const float thresholdAngle = PI_VALUE/36; // 5 deg in radians.
	
	// Compare the difference between the d component of the two points.
	bool distance = fabs(input_cloud_normals[a].d - input_cloud_normals[b].d) < thresholdDistance;
	// Compare the angle between normal vectors by doing the dot product.
	// Also this but not used [ theta = acos ( a . b / ||a|| ||b|| ) ].
	bool angle = dot(input_cloud_normals[a], input_cloud_normals[b]) < thresholdAngle;
	return distance && angle;
}





#pragma mark -
#pragma mark Object Detection Functions

void ObjectDetection::setZPoints(uint16_t* zPoints, size_t size)
{
	// Remove all points from the input_cloud.
	input_cloud.erase(input_cloud.begin(), input_cloud.end());
	
	// Iterate through all the new points and add them to the input_cloud
	// also calculate the world x and y values on input.
	for (size_t i = 0; i < size; i++) {
		input_cloud.emplace_back(worldFromIndex(i, zPoints[i]));
	}
}

void ObjectDetection::calculateSurfaceNormals()
{
	cout << "calculateSurfaceNormals started." << endl;
	
	// Iterate through all the points in the input_cloud and calcualte
	// the surface normals of each point.
	// First, remove all the current normals.
	input_cloud_normals.erase(input_cloud_normals.begin(), input_cloud_normals.end());
	
	// Some points will need to be skipped as the normals are calculated
	// by using 2 neighboring points to the right and below. Therefore the
	// right-most and bottom-most pixels will not have a complete set of
	// neighbours.
	for (size_t i = 0; i < input_cloud.size(); i++) {
		uint x = 0, y = 0;
		frameXYfromIndex(&x, &y, (uint)i);
		
		if (x == 0 && i > 0 // is right-hand side (and not i=0)
			&& y == FREENECT_FRAME_H // and is bottom
			) {
			// This point is out-of-bounds! Set the normal to 0.
			input_cloud_normals.emplace_back(PlaneCoefficients(0, 0, 0, 0));
		} else {
			// This point is within bounds. Calculate the normal.
			input_cloud_normals.emplace_back(getPlaneCoefficients(input_cloud[i],
																  input_cloud[i+1],
																  input_cloud[i+FREENECT_FRAME_W]));
		}
	}
	
	cout << "calculateSurfaceNormals completed." << endl;
}

void ObjectDetection::segmentPlanes_old_old()
{
	cout << "segmentPlanes started." << endl;
	
	// Create a vector containing all the indices to be clustered.
	vector<size_t> non_clustered_indices;
	for (size_t i = 0; i < input_cloud_normals.size(); i++) {
		non_clustered_indices.emplace_back(i);
	}
	
	// Remove all the previous plane clusters.
	plane_clusters.erase(plane_clusters.begin(), plane_clusters.end());
	
	// Iterate through each point in non_clustered and find which cluster it belongs to.
	while (non_clustered_indices.size() > 0) {
		// We need to process all the points until all the points belong to a cluster.
		
		// Now we are looking at creating a new cluster. Let's add the first available point
		// to it.
		PointIndices current_cluster_indices;
		current_cluster_indices.indices.emplace_back(*non_clustered_indices.begin());
		non_clustered_indices.erase(non_clustered_indices.begin());
		
		// Then compare all the other non_clustered points to the points in the current cluster.
		for (vector<size_t>::iterator nonc = non_clustered_indices.begin(); nonc != non_clustered_indices.end(); ++nonc) {
			//cout << "\t\t nonc start = " << *nonc << endl;
			bool neighbourFound = false;
			
			for (vector<size_t>::iterator curc = current_cluster_indices.indices.begin(); curc != current_cluster_indices.indices.end(); ++curc) {
				//cout << "\t\t\t nonc = " << *nonc << "   curc = " << *curc << "   :   nonc.size = " << non_clustered_indices.size() << "   curc.size = " << current_cluster_indices.indices.size() << endl;
				bool isNeighbour = compareNormalAngle(*nonc, *curc);
				
				if (isNeighbour) {
					//cout << "\t\t\t ^^ is neighbour" << endl;
					// This point in non_clustered is a neighbour to a point in current_cluster.
					// Therefore let's move it from non_clustered and into current_cluster.
					current_cluster_indices.indices.emplace_back(*nonc);
					non_clustered_indices.erase(nonc);
					neighbourFound = true;
					break;
				}
				
			}
			
			//cout << "\t\t nonc end = " << *nonc << endl;
			if (neighbourFound) {
				//something
			}
		}
		
		// We now should of found all the neighbours in current_cluster.
		// So now let's push back this current cluster to the plane_clusters vector.
		plane_clusters.emplace_back(current_cluster_indices);
		cout << "\tFinished cluster " << plane_clusters.size() << "." << endl;
	}
	
	cout << "segmentPlanes completed. Found " << plane_clusters.size() << " clusters." << endl;
}

void ObjectDetection::segmentPlanes_old()
{
	cout << "segmentPlanes started." << endl;
	
	// Create a vector containing all the indices to be clustered.
	vector<size_t> non_clustered_indices;
	for (size_t i = 0; i < input_cloud_normals.size(); i++) {
		non_clustered_indices.emplace_back(i);
	}
	
	// Iterate through each point in non_clustered and find which cluster it belongs to.
	while (non_clustered_indices.size() > 0) {
		
		PointIndices current_cluster;
		vector<size_t> check_list_indices;
		
		// Start by adding the first available non-clustered point to be checked.
		check_list_indices.emplace_back(*non_clustered_indices.begin());
		non_clustered_indices.erase(non_clustered_indices.begin());
		
		/*if (current_cluster.indices.size() > 1000) {
			cout << "\tStarting cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
		} else if (plane_clusters.size() % 1000 == 0) {
			cout << "\tStarting cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
		}*/
			
		// Now compare each point in check_list to its neighbours.
		// If a point is a neighbour then add it to the check list.
		// Once a point in the check_list has finished comparing neighbours then
		// and it to the current_cluster.
		size_t currentCount = 0;
		while (check_list_indices.size() > 0) {
			
			size_t currentIndex = *check_list_indices.begin();
			size_t aboveIndex   = *check_list_indices.begin()-FREENECT_FRAME_W;
			size_t belowIndex   = *check_list_indices.begin()+FREENECT_FRAME_W;
			size_t rightIndex   = *check_list_indices.begin()+1;
			size_t leftIndex    = *check_list_indices.begin()-1;
			
			bool aboveIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), aboveIndex) != non_clustered_indices.end();
			bool belowIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), belowIndex) != non_clustered_indices.end();
			bool rightIsValid = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), rightIndex) != non_clustered_indices.end();
			bool leftIsValid  = std::find(non_clustered_indices.begin(), non_clustered_indices.end(), leftIndex)  != non_clustered_indices.end();
			
			bool aboveIsNeighbour = compareNormalAngle(currentIndex, aboveIndex);
			bool belowIsNeighbour = compareNormalAngle(currentIndex, belowIndex);
			bool rightIsNeighbour = compareNormalAngle(currentIndex, rightIndex);
			bool leftIsNeighbour  = compareNormalAngle(currentIndex, leftIndex);
			
			//cout << "\t\tCurrent check_list_index = " << currentIndex << "  above = " << aboveIndex << "  below = " << belowIndex << "  right = " << rightIndex << "  left = " << leftIndex << "." << endl;
			//cout << "\t\t\t\t isValid  :" << "  above = " << aboveIsValid << "  below = " << belowIsValid << "  right = " << rightIsValid << "  left = " << leftIsValid << "." << endl;
			//cout << "\t\t\t\t isNeighbour  :";
			if (aboveIsValid) {
				if (aboveIsNeighbour) {
					//cout << "  above is  ";
					check_list_indices.emplace_back(aboveIndex);
					non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), aboveIndex), non_clustered_indices.end());
				}
			}
			
			if (belowIsValid) {
				if (belowIsNeighbour) {
					//cout << "  below is  ";
					check_list_indices.emplace_back(belowIndex);
					non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), belowIndex), non_clustered_indices.end());
				}
			}
			
			if (rightIsValid) {
				if (rightIsNeighbour) {
					//cout << "  right is  ";
					check_list_indices.emplace_back(rightIndex);
					non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), rightIndex), non_clustered_indices.end());
				}
			}
			
			if (leftIsValid) {
				if (leftIsNeighbour) {
					//cout << "  left is   ";
					check_list_indices.emplace_back(leftIndex);
					non_clustered_indices.erase(std::remove(non_clustered_indices.begin(), non_clustered_indices.end(), leftIndex), non_clustered_indices.end());
				}
			}
			//cout << "." << endl;
			
			// Now move the current_index into the current_cluster.
			current_cluster.indices.emplace_back(currentIndex);
			check_list_indices.erase(std::remove(check_list_indices.begin(), check_list_indices.end(), currentIndex), check_list_indices.end());
			
			//cout << "\t\tCurrent check_list.size = " << check_list_indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "   current_cluster.size = " << current_cluster.indices.size() << ".\n------------------------" << endl;
			if (currentCount > 100) {
				currentCount = 0;
				cout << "\t\tCurrent check_list.size = " << check_list_indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "   current_cluster.size = " << current_cluster.indices.size() << "  :  Neighbours  above=" << aboveIsNeighbour << "  below=" << belowIsNeighbour << "  left=" << leftIsNeighbour << "  right=" << rightIsNeighbour << "." << endl;
			} else {
				currentCount++;
			}
			
		}
		
		if (current_cluster.indices.size() > 1000) {
			cout << "\tFinished cluster " << plane_clusters.size() << "   current_cluster.size = " << current_cluster.indices.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
		} else if (plane_clusters.size() % 1000 == 0) {
			cout << "\tFinished cluster " << plane_clusters.size() << "    non_clustered.size = " << non_clustered_indices.size() << "." << endl;
		}
		
		// We have now finished with the current_cluster. So add it to the plane_clusters.
		plane_clusters.emplace_back(current_cluster);
		
	}
	
	size_t goodClusters = 0;
	//for (vector<PointIndices>::iterator cluster = plane_clusters.begin(); cluster != plane_clusters.end(); ++cluster) {
	for (size_t i = 0; i < plane_clusters.size(); i++) {
		size_t clusterCount = plane_clusters[i].indices.size();
		if (clusterCount > 1000) {
			goodClusters++;
		}
	}
	
	cout << "segmentPlanes completed. Found " << plane_clusters.size() << " clusters, of which " << goodClusters << "are over the threshold of 1000 points." << endl;
}


// Mark all nodes unassigned.
//
// ITERATE through all nodes:
//		IF node unassigned:
//			ASSIGN node to a new component id C
//			DO a depth-first-search for all nodes connected to this one:
//				mark them with same component id C

void ObjectDetection::segmentPlanes()
{
	cout << "segmentPlanes started." << endl;
	
	// Remove all the current points from the plane_cluster_nodes and reserve the size we need.
	// Also, set all the node values to -1 to show that they are all unassigned.
	plane_cluster_nodes.erase(plane_cluster_nodes.begin(), plane_cluster_nodes.end());
	plane_cluster_nodes.assign(FREENECT_FRAME_PIX, NODE_UNASSIGNED);
	
	int current_segment_id = -1;
	
	// Iterate through all nodes.
	for (size_t i = 0; i < FREENECT_FRAME_PIX; i++) {
		
		// Check if the current node is unassigned.
		if (plane_cluster_nodes[i] == NODE_UNASSIGNED) {
			
			//cout << "\tCurrent node is UNASSIGNED. Node = " << i << "   Current Segment = " << current_segment_id << "." << endl;
			
			// The node is unassigned then assign it a new segment_id.
			current_segment_id++;
			plane_cluster_nodes[i] = current_segment_id;
			
			// Find out if the current node has a neighbour.
			uint currentX, currentY = 0;
			frameXYfromIndex(&currentX, &currentY, (uint)i);
			
			bool rightIsNeighbour = (currentX < FREENECT_FRAME_W-1)&&(i < FREENECT_FRAME_PIX-1)				   ? compareNormalAngle(i, i+1)				   : false;
			bool belowIsNeighbour = (currentY < FREENECT_FRAME_H-1)&&(i < FREENECT_FRAME_PIX-FREENECT_FRAME_W) ? compareNormalAngle(i, i+FREENECT_FRAME_W) : false;
			
			// If it has neighbours then set their segment id to the current points id.
			if (rightIsNeighbour) {
				plane_cluster_nodes[i+1] = plane_cluster_nodes[i];
			}
			if (belowIsNeighbour) {
				plane_cluster_nodes[i+FREENECT_FRAME_W] = plane_cluster_nodes[i];
			}
			
		} else {
			
			// This node is assigned, so find its neighbours.
			// Find out if the current node has a neighbour.
			uint currentX, currentY = 0;
			frameXYfromIndex(&currentX, &currentY, (uint)i);
			
			bool rightIsNeighbour = (currentX < FREENECT_FRAME_W-1)&&(i < FREENECT_FRAME_PIX-1)				   ? compareNormalAngle(i, i+1)				   : false;
			bool belowIsNeighbour = (currentY < FREENECT_FRAME_H-1)&&(i < FREENECT_FRAME_PIX-FREENECT_FRAME_W) ? compareNormalAngle(i, i+FREENECT_FRAME_W) : false;
			
			//cout << "\tCurrent node is assigned. Node = " << i << "   Current Segment = " << current_segment_id << ".  rightIsNeighbour=" << rightIsNeighbour << "   belowIsNeighbour=" << belowIsNeighbour << "." << endl;
			
			// If it has neighbours then set their segment id to the current points id.
			if (rightIsNeighbour) {
				plane_cluster_nodes[i+1] = plane_cluster_nodes[i];
			}
			if (belowIsNeighbour) {
				plane_cluster_nodes[i+FREENECT_FRAME_W] = plane_cluster_nodes[i];
			}
			
		}
		
	}
	
	cout << "segmentPlanes completed. Found " << current_segment_id << "." << endl;
	
	// FILTERING
	
	// Run through all the points in plane_cluster_nodes and count how many
	// points belong to each one.
	vector<PointIndices> all_clusters;
	all_clusters.reserve(current_segment_id);
	
	/*for (size_t i = 0; i <= plane_cluster_nodes.size(); i++) {
		size_t segment_id = (size_t)plane_cluster_nodes[i];
		all_clusters[segment_id].indices.emplace_back(i);
	}
	
	size_t bigClusterCount = 0;
	for (size_t i = 0; i < all_clusters.size(); i++) {
		if (all_clusters[i].indices.size() > 1000) {
			bigClusterCount++;
		}
	}*/
	//cout << "segmentPlanes Filtered clusters down to " << bigClusterCount << " clusters." << endl;
}

void ObjectDetection::filterPlanes()
{
	
}


