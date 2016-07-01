// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// clustering.h
// Purpose: Utility namespace grouping PCL methods together for easier readability
//  and shorter code.

// @author Unnar Þór Axelsson
// @version 1.0 18/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>

#include <moment_of_inertia/moment_of_inertia_estimation.h>

using namespace pcl;
using namespace std;

namespace utils{

struct OBBcube{
    Eigen::Vector3f position;
    Eigen::Quaternionf quat;
    float x;
    float y;
    float z;
};

// Euclidean Cluster Extraction from PCL.
// Inputs:
//  cloud, single PointCloud that we wan't to extract clusters from.
//  tolerance, maximum distance between points so that they belong to same cluster.
//  min_size, minimum number of points in a cluster.
//  max_size, maximum number of points in a cluster.
// Output:
//  outvec, vector containing PointClouds where each PointCloud represents a single cluster.
template<typename PT>
void ecClustering(const typename PointCloud<PT>::Ptr cloud, const float tolerance, const float min_size, const float max_size,
    std::vector<typename PointCloud<PointXYZ>::Ptr> &outvec){

    typename pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
    tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PT> ec;
    ec.setClusterTolerance(tolerance); // 2cm
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointXYZ p;
        PointCloud<PointXYZ>::Ptr tmp_cloud (new PointCloud<PointXYZ>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            p.x = cloud->points[*pit].x;
            p.y = cloud->points[*pit].y;
            p.z = cloud->points[*pit].z;
            tmp_cloud->points.push_back(p);
        }
        tmp_cloud->width = tmp_cloud->points.size ();
        tmp_cloud->height = 1;
        tmp_cloud->is_dense = true;
        outvec.push_back(tmp_cloud);
    }
};

// Colors a PointCloud using rgb color values.
// Inputs:
//  cloud, single PointCloud without color information.
//  r/g/b, RGB color values in range [0,1].
template<typename PT>
PointCloud<PointXYZRGB>::Ptr colorClusters(const typename PointCloud<PT>::Ptr &cloud, float r, float g, float b){
    PointCloud<PointXYZRGB>::Ptr out (new PointCloud<PointXYZRGB>());
    PointXYZRGB p;
    for(auto point : cloud->points){
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.r = r;
        p.g = g;
        p.b = b;
        out->points.push_back(p);
    }
    return out;
};

// Colors a set of PointClouds randomly.
// Input:
//  vec, vector containing multiple Pointclouds.
// Output:
//  returns a single PointCloud where each cloud from vec has a random color.
template<typename PT>
PointCloud<PointXYZRGB>::Ptr colorClusters(const std::vector<typename PointCloud<PT>::Ptr> &vec){
    PointCloud<PointXYZRGB>::Ptr out (new PointCloud<PointXYZRGB>());
    PointXYZRGB p;
    for(auto cloud : vec){
        float r = rand() % 255;
        float g = rand() % 255;
        float b = rand() % 255;
        for(auto point : cloud->points){
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.r = r;
            p.g = g;
            p.b = b;
            out->points.push_back(p);
        }
    }
    return out;
};

// Difference of normal calculations from PCL.
// Input:
//  cloud, PointCloud to perform DON on.
//  scale1, radius used for the first normal calculation.
//  scale2, radius used for the second normal calculation.
//  threshold, The threshold to remove points based on normal difference.
// Output:
//  out, PointCloud containing those points that survived thresholding (hopefully the ground is removed).
void donFilter(const PointCloud<PointXYZI>::Ptr cloud,
    const float scale1, const float scale2, const float threshold,
    PointCloud<PointNormal>::Ptr out){

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<PointXYZI>::Ptr tree;
    if (cloud->isOrganized ()){
        tree.reset (new pcl::search::OrganizedNeighbor<PointXYZI> ());
    } else {
        tree.reset (new pcl::search::KdTree<PointXYZI> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (cloud);

    if (scale1 >= scale2){
        exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<PointXYZI, PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);

    /**
    * NOTE: setting viewpoint is very important, so that we can ensure
    * normals are all pointed in the same direction!
    */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    copyPointCloud<PointXYZI, PointNormal>(*cloud, *doncloud);

    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZI, PointNormal, PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ()){
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Save DoN features
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

    // Build the condition for filtering
    pcl::ConditionOr<PointNormal>::Ptr range_cond (
        new pcl::ConditionOr<PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
        new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
    );
    // Build the filter
    //pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
    pcl::ConditionalRemoval<PointNormal> condrem;
    condrem.setCondition(range_cond);
    
    condrem.setInputCloud (doncloud);

    // Apply filter
    condrem.filter (*out);
};

// Calculates the minimum oriented bounding box with relation to x and y directions, that is the bounding
// box is aligned to the z axis.
// Input:
//  clouds, vector containing PointClouds where we want to extract the bounding box.
// Output:
//  obbvec, vector containing oriented bounding boxes for all PointClouds.
void getOBB(const std::vector<PointCloud<PointXYZ>::Ptr> &clouds, std::vector<OBBcube> &obbvec){
    pcl::MomentOfInertiaEstimation<PointXYZ> feature_extractor;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ min_point_OBB_original;
    pcl::PointXYZ max_point_OBB_original;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    OBBcube obb_cube;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    for(auto cloud : clouds){

        pcl::getMinMax3D(*cloud, min_point_OBB_original, max_point_OBB_original);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1;
        coefficients->values[3] = 0;

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        feature_extractor.setInputCloud(cloud_projected);
        feature_extractor.compute();

        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


        obb_cube.position = Eigen::Vector3f(position_OBB.x, position_OBB.y, (max_point_OBB_original.z+min_point_OBB_original.z)/2.0);
        obb_cube.quat = Eigen::Quaternionf(rotational_matrix_OBB);
        obb_cube.x = max_point_OBB.x - min_point_OBB.x;
        obb_cube.y = max_point_OBB.y - min_point_OBB.y;
        obb_cube.z = max_point_OBB_original.z - min_point_OBB_original.z;
        obbvec.push_back(obb_cube);
    }
};

// Outlier removal based on radius search around each point. Point is removed if it doesn't have
// enough neighbors within a certain distance from it.
// Inputs:
//  cloud, the PointCloud where we want to remove points from.
//  rad, the radius size.
//  N, minimum number of neighbors.
void roRemoval(PointCloud<PointNormal>::Ptr cloud, float rad, int N){
    pcl::RadiusOutlierRemoval<PointNormal> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(rad);
    outrem.setMinNeighborsInRadius(N);
    // apply filter
    outrem.filter (*cloud);
}

// Same as above but with default values rad=0.5 and N = 8.
void roRemoval(PointCloud<PointNormal>::Ptr cloud){
    roRemoval(cloud, 0.5, 8);
}

}
#endif
