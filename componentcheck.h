#ifndef COMPONENTCHECK_H
#define COMPONENTCHECK_H

#include "math.h";

class componentCheck
{
public:
    componentCheck(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trg_);

    double CheckPlanarity(pcl::PointIndices::Ptr indices_src, pcl::PointIndices::Ptr indices_trg, double thr);
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_;
};

#endif // COMPONENTCHECK_H
