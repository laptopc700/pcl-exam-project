#ifndef PCQC_H
#define PCQC_H

#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include <QMap>
#include <QString>
#include <QColor>
// the class can easily be converted to use std libraries instead of QT, for QMap (std::map) as for QString (std::string) and color (triple of integers)

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRegisteredCloud();
    QColor* getPointColor(int pointIndex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNewComponentCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getComponentCloud(QString componentName);

    //SETTERS
    void setClusterSegThreshold(int threshold); // set the euclidean threshold for the clustering
    void setColorSegThreshold(int threshold); // set the color threshold (0-255) of tolerance for the color segmentation

    //FUNCTIONS
    void componentSelection(int selectedPointIndex); // select a component with the specified thresholds, and color it in green in the newComponentCloud
    bool componentSegmentation(); // segment the new component from the target cloud with the newComponentPointIndices and save it in the newComponentCloud
    bool componentSave(QString componentName); // add segmented component to component list (maybe dictionary?)
    bool componentDelete(QString componentName); // delete component from component list
    void registration();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafSize, int verbosity);
    void segmentation (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented, int verbosity);
    void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud; // the new clout to be registered and to be checked
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud; // the reference cloud that is correct in every detail
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud; // the source cloud registered to the target cloud
    int colThreshold; // the euclidean threshold for the clustering
    int cluThreshold; // the color threshold (0-255) of tolerance for the color segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newComponentCloud;
    pcl::PointIndices::Ptr newComponentPointIndices;
    QMap<QString, pcl::PointIndices> componentsList; // dictonary that maps a name of a component with a point indices of the target cloud that define that component


};

#endif // PCQC_H
