#ifndef SEGMENTDIFFERENCES_H
#define SEGMENTDIFFERENCES_H

#include <pcl/segmentation/segment_differences.h>

using namespace std;

void
segmentDiff
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
    double distance_threshold,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud
);

#endif // SEGMENTDIFFERENCES_H
