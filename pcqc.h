#ifndef PCQC_H
#define PCQC_H

#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include <QString>
#include <QColor>

class Pcqc
{
public:
    //INIT
    Pcqc();
    bool loadTargetCloud(QString path); // returns true if file was loaded correctly
    bool loadSourceCloud(QString path); // returns true if file was loaded correctly

    //GETTERS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTargetCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSourceCloud();
    QColor* getPointColor(int pointIndex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNewComponentCloud();

    //SETTERS
    void setClusterSegThreshold(int threshold); // set the euclidea threshold for the clustering
    void setColorSegThreshold(int threshold); // set the color threshold (0-255) of tolerance for the color segmentation

    //FUNCTIONS
    void componentSelection(int selectedPointIndex); // select a component with the specified thresholds, and color it in green in the newComponentCloud
    bool componentSegmentation(); // segment the new component from the target cloud with the newComponentPointIndices and save it in the newComponentCloud
    bool componentSave(QString componentName); // add segmented component to component list (maybe dictionary?)
    bool componentDelete(QString componentName); // delete component from component list

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud;
    int colThreshold;
    int cluThreshold;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newComponentCloud;
    pcl::PointIndices::Ptr newComponentPointIndices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud;
};

#endif // PCQC_H
