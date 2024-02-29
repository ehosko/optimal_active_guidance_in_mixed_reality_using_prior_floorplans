#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>



int main()
{
    std::string folder = "/home/michbaum/Projects/optag_EH/data/floorplan/";

    // Read pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder + "TargetCloud.pcd", *target_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder + "SourceCloud.pcd", *source_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder + "AligendCloud.pcd", *aligned_cloud);



    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Registration Viewer"));

    // Add the original target cloud in blue
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, "target_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "target_cloud");

    // Add the original source cloud in red
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, "source_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "source_cloud");

    // Add the transformed source cloud in green
    viewer->addPointCloud<pcl::PointXYZ>(aligned_cloud, "transformed_source_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "transformed_source_cloud");

    // Set the background color
    viewer->setBackgroundColor(0.7, 0.7, 0.7);

    // Set up the camera position and orientation (optional)
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);

    // Spin the viewer
    viewer->spin();

    return 0;
}
