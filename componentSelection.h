#ifndef COMPONENTSELECTION_H
#define COMPONENTSELECTION_H

#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

// performs a cluster segmentation on the input cloud from the selectedPointIndex and returns the pointIndices of the cluster found
void segmentCluster
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    double threshold
);

// performs a color segmentation on the input cloud from the selectedPointIndex and returns the pointIndices of the cluster found
void segmentColor
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointIndices::Ptr output,
    int selectedPointIndex,
    int threshold
);

// gives the intersection of two point indices vectors.
void intersectIndices
(
    pcl::PointIndices::Ptr first,
    pcl::PointIndices::Ptr second,
    pcl::PointIndices::Ptr intersection
);

//pcl::PointIndices segmentVolume
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












