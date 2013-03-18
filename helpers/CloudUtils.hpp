//
//  cloud_utils.h
//  robotarm
//
//  Created by James Reuss on 17/03/2013.
//	Based on the code by Nicolas Burrus from the
//	book Hacking the Kinect.
//

#ifndef __robotarm__cloud_utils__
#define __robotarm__cloud_utils__

#include <fstream>
#include <iostream>
#include <string>
#include <cassert>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_traits.h>

#include <Eigen/Geometry>

// DEFINITIONS
void removePointsWithNanNormals(pcl::PointCloud<pcl::PointNormal>& cloud);
pcl::PointCloud<pcl::PointNormal>::Ptr preprocessImageCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud);
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud);




// IMPLEMENTATIONS
void removePointsWithNanNormals(pcl::PointCloud<pcl::PointNormal>& cloud)
{
    pcl::PointCloud<pcl::PointNormal> tmp;
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        if (pcl_isfinite(cloud.points[i].normal_x))
            tmp.push_back(cloud.points[i]);
    }
    cloud = tmp;
}

pcl::PointCloud<pcl::PointNormal>::Ptr preprocessImageCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr image_cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    image_cloud_with_normals = computeNormals(image_cloud);
	
    pcl::PassThrough<pcl::PointNormal> bbox_filter;
    bbox_filter.setFilterFieldName("z");
    bbox_filter.setFilterLimits(-1.5, -0.5);
    bbox_filter.setInputCloud(image_cloud_with_normals->makeShared());
    bbox_filter.filter(*image_cloud_with_normals);
	
    removePointsWithNanNormals(*image_cloud_with_normals);
    return image_cloud_with_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (image_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    ne.compute (*normals);
	
    pcl::PointCloud<pcl::PointNormal>::Ptr image_cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*image_cloud, *normals, *image_cloud_with_normals);
    return image_cloud_with_normals;
}

#endif /* defined(__robotarm__cloud_utils__) */
