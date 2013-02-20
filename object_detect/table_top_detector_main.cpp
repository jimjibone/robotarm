// Author: Nicolas Burrus
// Hacking the Kinect
// Listings 9-1 to 9-4

#include "table_top_detector.h"

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/msac.h>

#include "utils.h"

#include <boost/thread/thread.hpp>

// Options.
bool show_clusters = true;
bool show_table_hull = true;
bool show_table_cloud = false;
bool fit_spheres = false;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: table_top_detector_main cloud_file" << std::endl;
        return -1;
    }

    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1], *cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr image_cloud = preprocessImageCloud(cloud);

    TableTopDetector<pcl::PointNormal> detector;
    detector.initialize();
    detector.detect(*image_cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);

    for (int i = 0; i < detector.objects().size(); ++i)
    {
        std::string cloud_name = cv::format("model%02d.pcd", i);
        pcl::io::savePCDFile(cloud_name, *detector.objects()[i]);
    }

    if (show_table_cloud)
    {
        cv::Vec3b color (255,0,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(detector.tableCloud(), color[0], color[1], color[2]);
        viewer->addPointCloud<pcl::PointNormal> (detector.tableCloud(), single_color, "table");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "table");
    }

    if (show_table_hull)
    {
        viewer->addPolygonMesh(*detector.tableHullMesh(), "table");
    }

    if (show_clusters)
    {
        cv::RNG rng;
        for (int i = 0; i < detector.objects().size(); ++i)
        {
            std::string cloud_name = cv::format("cloud %d", i);
            cv::Vec3b color (rng(255), rng(255), rng(255));
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(detector.objects()[i], color[0], color[1], color[2]);
            viewer->addPointCloud<pcl::PointNormal> (detector.objects()[i], single_color, cloud_name);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
        }
    }


    // Optional sphere fitting.
    if (fit_spheres)
    {
        for (int cluster_id = 0; cluster_id < detector.objects().size(); ++cluster_id)
        {
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud = detector.objects()[cluster_id];

            pcl::SampleConsensusModelSphere<pcl::PointNormal>::Ptr
                    sphere_model(new pcl::SampleConsensusModelSphere<pcl::PointNormal> (cloud));

            pcl::MEstimatorSampleConsensus<pcl::PointNormal> ransac (sphere_model);
            ransac.setDistanceThreshold (.004);
            ransac.computeModel();
            Eigen::VectorXf sphere_parameters;
            ransac.getModelCoefficients(sphere_parameters);
            pcl::ModelCoefficients model_coeffs;
            model_coeffs.values.resize(sphere_parameters.size());
            for (int i = 0; i < sphere_parameters.size(); ++i)
                model_coeffs.values[i] = sphere_parameters[i];
            viewer->addSphere(model_coeffs, cv::format("sphere %d", cluster_id));
        }
    }

    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
