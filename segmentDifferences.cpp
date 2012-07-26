#include "segmentDifferences.h"

using namespace std;

void
segmentDiff
(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
        double distance_threshold,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud
        )
{
    //Differences segmentation
    pcl::SegmentDifferences<pcl::PointXYZRGB> p;
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
    p.setInputCloud (source_cloud);
//    p.setSearchMethod(search_tree);
    p.setTargetCloud (target_cloud);
    p.setDistanceThreshold (distance_threshold);
    p.segment(*diff_cloud);
}
