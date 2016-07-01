#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <type_traits>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGB PointTC;
typedef pcl::PointCloud<PointTC> PointCloudTC;

using namespace pcl;

namespace utils{

template<typename PT>
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (typename pcl::PointCloud<PT>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PT> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(-5,0,2,1,0,0);
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (typename pcl::PointCloud<PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<PointXYZRGB> (cloud, rgb, "cloud2");
  // viewer->addPointCloud<PointTC> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(-5,0,2,1,0,0);
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<PointT>::ConstPtr cloud1, pcl::PointCloud<PointT>::ConstPtr cloud2)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addPointCloud<PointT> (cloud1, "cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);
    viewer->addPointCloud<PointT> (cloud2, "cloud2", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(-5,0,2,1,0,0);
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<PointTC>::ConstPtr cloud1, pcl::PointCloud<PointTC>::ConstPtr cloud2)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointTC> rgb(cloud1);
    viewer->addPointCloud<PointTC> (cloud1, rgb, "cloud2", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);
    pcl::visualization::PointCloudColorHandlerRGBField<PointTC> rgb2(cloud2);
    viewer->addPointCloud<PointTC> (cloud2, rgb2, "cloud2", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(-5,0,2,1,0,0);
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<PointT>::ConstPtr cloud1, pcl::PointCloud<PointTC>::ConstPtr cloud2)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addPointCloud<PointT> (cloud1, "cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);
    pcl::visualization::PointCloudColorHandlerRGBField<PointTC> rgb(cloud2);
    viewer->addPointCloud<PointTC> (cloud2, rgb, "cloud2", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(-5,0,2,1,0,0);
    return (viewer);
}




}

#endif