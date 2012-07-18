#ifndef PCQC_H
#define PCQC_H

#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include "component.h"
#include <qiterator.h>
#include <QMap>
#include <QString>
#include <QColor>
#include <iterator>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
// the class can easily be converted to use std libraries instead of QT, for QMap (std::map) as for QString (std::string) and color (triple of integers).

class Pcqc
{
public:
    //INIT
    Pcqc();
    bool loadTargetCloud(QString path); // returns true if file was loaded correctly.
    bool loadSourceCloud(QString path); // returns true if file was loaded correctly.

    //GETTERS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTargetCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSourceCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRegisteredCloud();
    QColor* getPointColor(int pointIndex); // da modificare per renderlo utile anche su altre cloud (prendere come argomento un cloud pointer).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNewComponentCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getComponentCloud(QString componentName);

    //SETTERS
    void setClusterSegThreshold(int threshold); // set the euclidean threshold for the clustering.
    void setColorSegThreshold(int threshold); // set the color threshold (0-255) of tolerance for the color segmentation.

    //FUNCTIONS
    void colorIndices( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointIndices::Ptr indices,int r,int g, int b); // color in r,g,b the specified indices of the input cloud.
    void colorComponents( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,int r, int g,int b); //color in r,g,b the specified indices of the input cloud.
    void componentSelection(int selectedPointIndex); // select a component with the specified thresholds, and color it in green in the newComponentCloud.
    bool componentSegmentation(); // segment the new component from the target cloud with the newComponentPointIndices and save it in the newComponentCloud.
    bool componentSave(QString componentName); // add segmented component to component list (maybe dictionary?).
    bool componentDelete(QString componentName); // delete component from component list.
    void registration(); // performs registration from source to target cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafSize);
    void segmentation (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented); // performs segmentation of the main plane from the cloud.
    void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud); // removes outliers from the cloud.
    void findSourceComponent(); // finds a component specified on the target cloud inn the source cloud.

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud; // the new cloud to be registered and to be checked.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud; // the reference cloud that is correct in every detail.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud; // the source cloud registered to the target cloud.
    int colThreshold; // the euclidean threshold for the clustering.
    int cluThreshold; // the color threshold (0-255) of tolerance for the color segmentation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newComponentCloud;
    pcl::PointIndices::Ptr newComponentPointIndices;
    int lastClickedPointIndex; // the index of the last point clicked (used to define the component selection).
    QMap<QString, Component> componentsList; // dictonary that maps a name of a component with a point indices of the target cloud that define that component.
    QMap<QString, Component> sourceComponentsList; // list of the components found in the source cloud corresponding to the componentsList ones.


};

#endif // PCQC_H
