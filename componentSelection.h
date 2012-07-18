#ifndef COMPONENTSELECTION_H
#define COMPONENTSELECTION_H

#include <cmath>
#include <vector>
#include <string>
#include <sstream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/features/pfh.h>
//#include <pcl/features/pfhrgb.h>
//#include <pcl/features/3dsc.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/common/transforms.h>
//#include <pcl/surface/grid_projection.h>

using namespace std;

void
segmentCluster
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    double threshold
);




void
segmentColor
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    int threshold
);


void
intersectIndices
(
    pcl::PointIndices::Ptr first,
    pcl::PointIndices::Ptr second,
    pcl::PointIndices::Ptr intersection
);

//pcl::PointIndices
//segmentVolume
//(
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
//    float positionX,
//    float positionY,
//    float positionZ,
//    float sizeX,
//    float sizeY,
//    float sizeZ,
//    float rotation
//);
#endif // REGISTRATION_H












