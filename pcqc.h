#ifndef PCQC_H
#define PCQC_H

#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include "component.h"
#include "segmentDifferences.h"
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSourceDiffCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTargetDiffCloud();
    QColor* getPointColor(int pointIndex); // TO DO: da modificare per renderlo utile anche su altre cloud (prendere come argomento un cloud pointer).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNewComponentCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTargetComponentCloud(QString componentName);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSourceComponentCloud(QString componentName); // TO DO: DA IMPLEMENTARE
    QMap<QString, Component>* getTargetComponentsList();
    QMap<QString, Component>* getSourceComponentsList();

    //SETTERS
    void setClusterSegThreshold(int threshold); // set the euclidean threshold for the clustering.
    void setColorSegThreshold(int threshold); // set the color threshold (0-255) of tolerance for the color segmentation.
    void setSegDiffThreshold(double threshold); //set the distance threshold for differences segmentation.

    //FUNCTIONS
    void colorIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr indices,int r,int g, int b); // color in r,g,b the specified indices of the input cloud.
    void colorComponents(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QMap<QString, Component> *components, int r, int g,int b); // TO DO: prendere in input la lista così da poter colorare anche la source list.
    void componentSelection(int selectedPointIndex); // select a component with the specified thresholds, and color it in green in the newComponentCloud.
    bool componentSegmentation(); // segment the new component from the target cloud with the newComponentPointIndices and save it in the newComponentCloud.
    bool componentSave(QString componentName); // add segmented component to target components list.
    bool componentDelete(QString componentName); // delete component from target components list.
    void registration(); // performs registration from source to target cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafSize);
    void segmentation (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented); // performs segmentation of the main plane from the cloud.
    void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud); // removes outliers from the cloud.
    int findSourceComponents(); // finds components in the source cloud corresponding to the componends of the target cloud specified in componentsList. Returns the number of components found.
    void segmentDifferences();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud; // the new cloud to be registered and to be checked.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud; // the reference cloud that is correct in every detail.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud; // the source cloud registered to the target cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceDiffCloud; //the output cloud from target to source differences segmentation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetDiffCloud; //the output cloud from source to target differences segmentation.
    int colThreshold; // the euclidean threshold for the clustering.
    int cluThreshold; // the color threshold (0-255) of tolerance for the color segmentation.
    double segDiffThreshold; //the distance threshold for differences segmentation.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newComponentCloud;
    pcl::PointIndices::Ptr newComponentPointIndices;
    int lastClickedPointIndex; // the index of the last point clicked (used to define the component selection).
    QMap<QString, Component> targetComponentsList; // dictonary that maps a name of a component with a point indices of the target cloud that define that component.
    QMap<QString, Component> sourceComponentsList; // list of the components found in the source cloud corresponding to the componentsList ones.

};

#endif // PCQC_H
