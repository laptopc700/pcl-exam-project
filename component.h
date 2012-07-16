
#ifndef COMPONENT_H
#define COMPONENT_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>


class Component
{

public:
    //CONSTRUCTOR
    Component(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, pcl::PointIndices::Ptr componentIndices_,
              int generatingIndex_, double clusterThreshold_, int colorThreshold_);

    //GETTERS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
    pcl::PointIndices::Ptr getIndices();
    int getGeneratingIndex();
    int getColorThreshold();
    double getClusterThreshold();
    int getColR();
    int getColG();
    int getColB();
    float getPosX();
    float getPosY();
    float getPosZ();
    int getSize();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointIndices::Ptr componentIndices;
    int generatingIndex;
    double clusterThreshold;
    int colorThreshold;



};
#endif // COMPONENT_H
