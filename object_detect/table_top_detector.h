// Author: Nicolas Burrus
// Hacking the Kinect
#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

template <class PointT>
class TableTopDetector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef PointT Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<Point>::Ptr KdTreePtr;

public:
    TableTopDetector();
    void initialize();

public:
    float voxelSize() const { return object_voxel_size_; }
    void setObjectVoxelSize(float s = 0.003) { object_voxel_size_ = s; }
    void setBackgroundVoxelSize(float s = 0.01) { background_voxel_size_ = s; }
    void setDepthLimits(float min_z = -1.6, float max_z = -0.4) { min_z_bounds_ = min_z; max_z_bounds_ = max_z; }
    void setObjectHeightLimits(float min_h = 0.01, float max_h = 0.5) { object_min_height_ = min_h;  object_max_height_ = max_h; }

public:
    /*! Returns true if at least one object and plane are detected. */
    bool detect(PointCloud& cloud);

public:
    const Eigen::Vector4f& plane() const { return plane_; }
    const std::vector < PointCloudPtr >& objects() const { return objects_; }
    PointCloudConstPtr tableHull() const { return table_hull_; }
    PointCloudConstPtr tableCloud() const { return table_cloud_; }
    pcl::PolygonMeshConstPtr tableHullMesh() const { return table_hull_mesh_; }
    PointCloudConstPtr nonTableCloud() const { return non_table_cloud_; }
    PointCloudConstPtr objectCloud() const { return cloud_objects_; }

private:
    // PCL objects
    KdTreePtr normals_tree_;
    KdTreePtr clusters_tree_;
    pcl::PassThrough<Point> pass_through_filter_;
    pcl::VoxelGrid<Point> grid_;
    pcl::VoxelGrid<Point> grid_objects_;
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimator_;
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> segmentor_;
    pcl::ProjectInliers<Point> project_inliers_;
    pcl::ConvexHull<Point> hull_;
    pcl::ExtractPolygonalPrismData<Point> prism_;
    pcl::EuclideanClusterExtraction<Point> cluster_;

    double background_voxel_size_, object_voxel_size_;
    double min_z_bounds_, max_z_bounds_;
    double plane_distance_threshold_;
    double normal_distance_weight_;

    // Min/Max height from the table plane object points will be considered from/to
    double object_min_height_, object_max_height_;

    // Object cluster tolerance and minimum cluster size
    double object_cluster_tolerance_, object_cluster_min_size_;

    PointCloudConstPtr cloud_;
    PointCloudConstPtr cloud_filtered_, cloud_downsampled_;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    pcl::PointIndices::ConstPtr table_inliers_;
    PointCloudConstPtr table_cloud_;
    pcl::ModelCoefficients::ConstPtr table_coefficients_;
    PointCloudConstPtr table_projected_;
    PointCloudConstPtr table_hull_;
    pcl::PolygonMeshPtr table_hull_mesh_;
    PointCloudPtr non_table_cloud_;
    PointCloudConstPtr cloud_objects_, cloud_objects_downsampled_;

    Eigen::Vector4f plane_;
    std::vector< PointCloudPtr > objects_;
};
