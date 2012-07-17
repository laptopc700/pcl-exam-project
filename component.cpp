#include "component.h"

Component::Component()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    componentIndices.reset(new pcl::PointIndices);
    generatingIndex=0;
    clusterThreshold=0;
    colorThreshold=0;
}

Component::Component(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, pcl::PointIndices::Ptr componentIndices_,
          int generatingIndex_, double clusterThreshold_, int colorThreshold_)
{
    cloud= cloud_;
    componentIndices=componentIndices_;
    generatingIndex=generatingIndex_;
    clusterThreshold=clusterThreshold_;
    colorThreshold=colorThreshold_;
}

//GETTERS
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Component::getCloud()
{
    return cloud;
}
pcl::PointIndices::Ptr Component::getIndices()
{
    return componentIndices;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Component::getComponentCloud()
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr componentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::copyPointCloud(*cloud,*componentIndices,*componentCloud);
   return componentCloud;
}
int Component::getGeneratingIndex()
{
    return generatingIndex;
}
int Component::getColorThreshold()
{
    return colorThreshold;
}
double Component::getClusterThreshold()
{
    return clusterThreshold;
}
int Component::getColR()
{
    return cloud->at(generatingIndex).r;
}
int Component::getColG()
{
    return cloud->at(generatingIndex).g;
}
int Component::getColB()
{
    return cloud->at(generatingIndex).b;
}
float Component::getPosX()
{
    return cloud->at(generatingIndex).x;
}
float Component::getPosY()
{
    return cloud->at(generatingIndex).y;
}
float Component::getPosZ()
{
    return cloud->at(generatingIndex).z;
}
int Component::getSize()
{
    return componentIndices->indices.size();
}







