#include "componentcheck.h"

componentCheck::componentCheck(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trg_)
{
    trg_ = cloud_trg_;
    src_ = cloud_src_;
}

componentCheck::CheckPlanarity(pcl::PointIndices::Ptr indices_src, pcl::PointIndices::Ptr indices_trg, double thr)
{
    //INIZIATE RANSAC MODEL PLAN SEARCH FOR SRC
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_src;
    pcl::copyPointCloud (*src_, indices_src, *temp_src);

    pcl::ModelCoefficients::Ptr coef_src (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_src (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (thr);
    seg.setInputCloud (temp_src);
    seg.segment (*inliers_src, *coef_src);
    pcl::console::print_info ("Num of points of source plan segm.extimation:  %zu \n", inliers_src->points.size());

    //INIZIATE RANSAC MODEL PLAN SEARCH FOR TRG
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_trg;
    pcl::copyPointCloud (*trg_, indices_trg, *temp_trg);

    pcl::ModelCoefficients::Ptr coef_trg (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_trg (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (thr);
    seg.setInputCloud (temp_trg);
    seg.segment (*inliers_trg, *coef_trg);
    pcl::console::print_info ("Num of points of source plan segm.extimation:  %zu \n", inliers_trg->points.size());

    //CALCULATE ANGLE BETWEEN PLANES
    double num, norm_src, norm_trg;
    num = (coef_src->values[0]*coef_trg->values[0])+(coef_src->values[1]*coef_trg->values[1])+(coef_src->values[2]*coef_trg->values[2]);
    norm_src = sqrt((coef_src->values[0]*coef_src->values[0])+(coef_src->values[1]*coef_src->values[1])+(coef_src->values[2]*coef_src->values[2]));
    norm_trg = sqrt((coef_trg->values[0]*coef_trg->values[0])+(coef_trg->values[1]*coef_trg->values[1])+(coef_trg->values[2]*coef_trg->values[2]));

    return acos(num / (norm_src * norm_trg));

}
