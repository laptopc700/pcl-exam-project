#ifndef COMPONENTCHECK_H
#define COMPONENTCHECK_H

#include "math.h";
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class componentCheck
{
public:
    componentCheck(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trg_);

    bool IsPlanar(pcl::PointIndices::Ptr indices_src, pcl::PointIndices::Ptr indices_trg, double thr_plan, double thr_angle);
private:
    double CheckPlanarity(pcl::PointIndices::Ptr indices_src, pcl::PointIndices::Ptr indices_trg, double thr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trg_;
};

#endif // COMPONENTCHECK_H
