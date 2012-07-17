#ifndef COMPONENTMATCH_H
#define COMPONENTMATCH_H

#include "registration.h"

using namespace std;


//void
//matchComponent
//(
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
//        pcl::PointIndices::Ptr inputComponentIndices,
//        int clickedIndex,
//        double cluThreshold,
//        int colThreshold,
//        pcl::PointIndices::Ptr outputComponentIndices
//);

void
componentMatch
(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
        pcl::PointIndices::Ptr inputComponentIndices,
        int clickedIndex,
        double cluThreshold,
        int colThreshold,
        pcl::PointIndices::Ptr outputComponentIndices
);

bool colorOk(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2);

float dist (pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2);
#endif // COMPONENTMATCH_H
