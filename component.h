
#ifndef COMPONENT_H
#define COMPONENT_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//TO DO: Considerare la possibilità di rendere questa classe astratta e di ereditarla nelle classi TargetComponent e SourceComponent, o nei tipi di componenti specifici.

class Component
{
public:
    //CONSTRUCTORS
    Component(); //empty
    Component(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, pcl::PointIndices::Ptr componentIndices_, int generatingIndex_, double clusterThreshold_, int colorThreshold_); // target component
    Component(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, pcl::PointIndices::Ptr componentIndices_); //source component

    //GETTERS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(); // returns the component cloud to which the component is referred (target or source).
    pcl::PointIndices::Ptr getIndices(); // returns the component's pointIndices.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getComponentCloud(); // returns the component's point cloud.
    int getGeneratingIndex(); // returns the index of the point clicked on the target/source cloud to segment this component.
    int getColorThreshold(); // returns the color threshold used to segment this component.
    double getClusterThreshold(); // returns the clustering threshold used tosegment this component.
    int getColR();
    int getColG();
    int getColB();
    float getPosX();
    float getPosY();
    float getPosZ();
    int getSize(); // returns the size of the component's pointIndices vector.

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // pointer to the cloud from which the component was segmented.
    pcl::PointIndices::Ptr componentIndices; // indices of the component's points referred to "cloud" point cloud.
    int generatingIndex; // index of the point clicked on the target cloud to segment this component.
    double clusterThreshold; // clustering threshold used tosegment this component.
    int colorThreshold; // color threshold used to segment this component.
};
#endif // COMPONENT_H
