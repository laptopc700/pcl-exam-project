#include "component.h"

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







