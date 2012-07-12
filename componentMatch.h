#ifndef COMPONENTMATCH_H
#define COMPONENTMATCH_H

#include "registration.h"

using namespace std;


void
matchComponent
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
    pcl::PointIndices::Ptr inputTargetIndices,
    pcl::PointIndices::Ptr outputSourceIndices
);

bool colorOk(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2);

float dist (pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2);
#endif // COMPONENTMATCH_H
